// arduino i2c slave - robot car controller with encoder support

#include <Wire.h>
#include <Servo.h>

#define I2C_ADDRESS 0x08

// motor pins (l298n)
#define ENA 5  // left motor pwm
#define IN1 2  // left motor dir 1
#define IN2 4  // left motor dir 2
#define ENB 6  // right motor pwm
#define IN3 7  // right motor dir 1
#define IN4 8  // right motor dir 2

// sensor pins
#define TRIG_PIN 9
#define ECHO_PIN 10
#define SERVO_PIN A2

// encoder pins (must be A0 and A1 for pin change interrupts)
#define LEFT_ENCODER_PIN A0   // PCINT8 - left wheel encoder
#define RIGHT_ENCODER_PIN A1  // PCINT9 - right wheel encoder

// encoder specifications
const float WHEEL_DIAMETER = 65.0;  // mm
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI;  // ~204.2mm
const int PULSES_PER_ROTATION = 21;
const float MM_PER_PULSE = WHEEL_CIRCUMFERENCE / PULSES_PER_ROTATION;  // ~9.72mm
const float WHEELBASE = 145.0;  // mm center-to-center distance

// motor calibration - disabled until Phase 2 adds encoder-based correction
// previous values (1.05 / 0.90) were backwards and caused right motor starvation

// turn calibration - compensates for wheel slippage during rotation
// increase if turns are too shallow, decrease if turns are too far
const float TURN_CALIBRATION_FACTOR = 1.08;  // 8% more rotation to account for slippage

// L298N dead zone - PWM below this won't turn the motor, just buzzes
// MEASURE EMPIRICALLY: ramp PWM from 30 up until each motor shaft turns
// These are placeholder values - typical range is 30-60 for small DC motors
const int MOTOR_DEAD_ZONE = 75;

// Feedforward: static boost for weaker right motor (applied in setMotors)
// Tune empirically: increase if car still drifts right, decrease if it drifts left
const int RIGHT_MOTOR_BOOST = 15;

// Feedback: PI-controller for fine-tuning straight-line driving
const int ENCODER_DEAD_BAND = 1;   // Don't correct small differences (pulses)
const int KP = 8;                    // Proportional gain (moderate — feedforward handles gross imbalance)
const int KI = 2;                    // Integral gain (conservative — corrects persistent drift)
const int MAX_INTEGRAL = 30;         // Anti-windup clamp (max integral contribution = KI*MAX_INTEGRAL = 60 PWM)
int integralError = 0;               // Accumulated error (reset each movement)

// Adaptive speed reduction near movement target
const int MIN_MOVEMENT_SPEED = MOTOR_DEAD_ZONE + 20;  // 95 PWM minimum during deceleration
const int DECEL_THRESHOLD_PULSES = 3;  // Start decelerating within this many pulses of target

// encoder variables (volatile for ISR access)
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile byte lastLeftState = HIGH;
volatile byte lastRightState = HIGH;

// Encoder debouncing — filters phantom pulses from vibration
// 1500us is well below the minimum real pulse interval (5.7ms at 500 RPM)
volatile unsigned long lastLeftPulseTime = 0;
volatile unsigned long lastRightPulseTime = 0;
const unsigned long ENCODER_DEBOUNCE_US = 1500;  // 1.5ms debounce window

// movement timeout constants
const unsigned long BASE_MOVEMENT_TIMEOUT = 3000;  // base timeout
const unsigned long TIMEOUT_PER_PULSE = 100;       // add 100ms per pulse needed
const unsigned long MAX_MOVEMENT_TIMEOUT = 15000;  // max 15 seconds

// Stall detection — fires if motors running but no encoder progress
const unsigned long STALL_TIMEOUT = 500;  // 500ms with no encoder change = stall

// non-blocking movement state machine (replaces blocking while loops)
enum MovementState : uint8_t {
  MOVE_IDLE,       // no movement active
  MOVE_RUNNING,    // moving with encoder monitoring
  MOVE_SETTLING,   // motors stopped, waiting 100ms for settling
  MOVE_COMPLETE    // ready to report done, transitions to IDLE
};

struct MovementContext {
  MovementState state;
  long targetPulses;
  int baseSpeed;
  int currentLeftPWM;
  int currentRightPWM;
  unsigned long startTime;
  unsigned long settlingStart;
  unsigned long timeout;
  bool isRotation;
  int8_t leftDirection;   // +1 or -1
  int8_t rightDirection;  // +1 or -1
  bool timedOut;          // true if movement ended due to timeout
  unsigned long lastEncoderChangeTime;  // millis() when encoder last incremented
  long lastLeftCount;                   // encoder count at last stall check
  long lastRightCount;
};

MovementContext movement = { MOVE_IDLE, 0, 0, 0, 0, 0, 0, 0, false, 1, 1, false, 0, 0, 0 };

// PWM-change tracking: only call setMotors() when values differ
int lastSetLeftPWM = 0;
int lastSetRightPWM = 0;

Servo cameraServo;

int leftSpeed = 0;
int rightSpeed = 0;
int servoAngle = 90;
int lastServoAngle = 90;  // track last written angle
int distance = 0;
char motorStatusStr[10] = "STOPPED";
unsigned long lastServoMove = 0;
const unsigned long SERVO_DETACH_TIMEOUT = 2000; // detach servo after 2s of inactivity

char cmdBuffer[32];
int cmdIndex = 0;
bool newCmd = false;

// Pre-formatted I2C response buffer (written in loop, read in ISR)
char responseBuffer[32] = "0,0,0,STOPPED\n";
volatile uint8_t responseLen = 14;

unsigned long lastSensorRead = 0;
const int SENSOR_INTERVAL = 100;      // 100ms

// forward declarations
void readDistance(bool force = false);
void performScan();

void setup() {
  Serial.begin(115200);
  Serial.println("=== arduino robot car - i2c slave with encoders ===");
  Serial.println("i2c address: 0x08 (sda=a4, scl=a5)");
  Serial.print("encoder pins: left=A0, right=A1, ");
  Serial.print(PULSES_PER_ROTATION);
  Serial.println(" pulses/rotation");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // motor pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();

  // sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // encoder pins - internal pullups for optical sensors
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);

  // read initial encoder states
  lastLeftState = digitalRead(LEFT_ENCODER_PIN);
  lastRightState = digitalRead(RIGHT_ENCODER_PIN);

  // setup pin change interrupts for A0 (PCINT8) and A1 (PCINT9)
  setupEncoderInterrupts();

  // servo setup
  cameraServo.attach(SERVO_PIN);
  cameraServo.write(servoAngle);
  lastServoAngle = servoAngle;
  lastServoMove = millis();

  // i2c setup
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

  Serial.println("ready! waiting for i2c commands...");

  // test blink
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

// setup pin change interrupts for encoder pins
void setupEncoderInterrupts() {
  // enable pin change interrupt for port c (analog pins)
  PCICR |= (1 << PCIE1);

  // enable interrupts for A0 (PCINT8) and A1 (PCINT9)
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9);

  Serial.println("encoder interrupts enabled");
}

// pin change interrupt service routine (with debouncing)
ISR(PCINT1_vect) {
  unsigned long now = micros();  // call once, reuse — safe in AVR ISRs

  // read current states
  byte leftState = digitalRead(LEFT_ENCODER_PIN);
  byte rightState = digitalRead(RIGHT_ENCODER_PIN);

  // detect rising edges (LOW to HIGH transitions) with debounce filtering
  if (leftState == HIGH && lastLeftState == LOW) {
    if (now - lastLeftPulseTime >= ENCODER_DEBOUNCE_US) {
      leftEncoderCount++;
      lastLeftPulseTime = now;
    }
  }

  if (rightState == HIGH && lastRightState == LOW) {
    if (now - lastRightPulseTime >= ENCODER_DEBOUNCE_US) {
      rightEncoderCount++;
      lastRightPulseTime = now;
    }
  }

  // save states for next comparison
  lastLeftState = leftState;
  lastRightState = rightState;
}

void updateResponseBuffer() {
  char temp[32];

  cli();
  long leftEnc = leftEncoderCount;
  long rightEnc = rightEncoderCount;
  sei();

  int len = snprintf(temp, sizeof(temp), "%d,%ld,%ld,%s\n",
                     distance, leftEnc, rightEnc, motorStatusStr);
  if (len >= (int)sizeof(temp)) {
    len = sizeof(temp) - 1;
  }

  cli();
  memcpy(responseBuffer, temp, len + 1);
  responseLen = (uint8_t)len;
  sei();
}

void loop() {
  if (newCmd) {
    // copy to local buffer to prevent overwrite during processing
    char localCmd[32];
    cli();
    strcpy(localCmd, cmdBuffer);
    newCmd = false;
    sei();

    processCommand(localCmd);
  }

  // non-blocking movement state machine (replaces blocking while loops)
  updateMovement();

  // Keep sensor data and response buffer current
  readDistance();
  updateResponseBuffer();

  // detach servo after timeout to prevent twitching and save power
  if (cameraServo.attached() && (millis() - lastServoMove > SERVO_DETACH_TIMEOUT)) {
    cameraServo.detach();
  }
}

void onReceive(int bytes) {
  digitalWrite(LED_BUILTIN, HIGH);

  cmdIndex = 0;
  memset(cmdBuffer, 0, sizeof(cmdBuffer));

  while (Wire.available() && cmdIndex < 31) {
    char c = Wire.read();
    cmdBuffer[cmdIndex++] = c;
  }

  while (Wire.available()) {
    Wire.read();
  }

  newCmd = true;

  digitalWrite(LED_BUILTIN, LOW);
}

void onRequest() {
  Wire.write((const uint8_t*)responseBuffer, responseLen);
}

void processCommand(const char* command) {
  String cmd = String(command);
  cmd.trim();

  Serial.print("i2c rx: ");
  Serial.println(cmd);

  if (cmd.startsWith("M ")) {
    // standard motor control: "M leftSpeed rightSpeed"
    int space = cmd.indexOf(' ', 2);
    if (space > 0) {
      leftSpeed = cmd.substring(2, space).toInt();
      rightSpeed = cmd.substring(space + 1).toInt();

      leftSpeed = constrain(leftSpeed, -255, 255);
      rightSpeed = constrain(rightSpeed, -255, 255);

      Serial.print("motors: L=");
      Serial.print(leftSpeed);
      Serial.print(" R=");
      Serial.println(rightSpeed);

      movement.state = MOVE_IDLE;  // cancel any in-progress movement
      setMotors(leftSpeed, rightSpeed);
    }
  }
  else if (cmd.startsWith("D ")) {
    // distance-based movement: "D distance_mm direction speed"
    // Example: "D 200 F 150" (move 200mm forward at speed 150)
    int space1 = cmd.indexOf(' ', 2);
    int space2 = cmd.indexOf(' ', space1 + 1);
    if (space1 > 0 && space2 > 0) {
      int distanceMM = cmd.substring(2, space1).toInt();
      char direction = cmd.charAt(space1 + 1);
      int speed = cmd.substring(space2 + 1).toInt();
      startMoveDistance(distanceMM, direction, speed);
    }
  }
  else if (cmd.startsWith("R ")) {
    // rotation-based turning: "R degrees direction speed"
    // Example: "R 90 L 150" (turn 90° left at speed 150)
    int space1 = cmd.indexOf(' ', 2);
    int space2 = cmd.indexOf(' ', space1 + 1);
    if (space1 > 0 && space2 > 0) {
      int degrees = cmd.substring(2, space1).toInt();
      char direction = cmd.charAt(space1 + 1);
      int speed = cmd.substring(space2 + 1).toInt();
      startRotateDegrees(degrees, direction, speed);
    }
  }
  else if (cmd == "E") {
    // reset encoder counts to zero
    cli();
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    sei();
    movement.state = MOVE_IDLE;  // cancel any in-progress movement
    Serial.println("encoders reset to 0");
  }
  else if (cmd == "Q") {
    // query encoder values (will be sent in next I2C transmission)
    Serial.print("encoder query: L=");
    Serial.print(leftEncoderCount);
    Serial.print(" R=");
    Serial.println(rightEncoderCount);
  }
  // Legacy command support for backward compatibility
  else if (cmd.startsWith("MOVE ")) {
    // encoder-based forward/backward: "MOVE distance_mm speed"
    int space = cmd.indexOf(' ', 5);
    if (space > 0) {
      int distanceMM = cmd.substring(5, space).toInt();
      int speed = cmd.substring(space + 1).toInt();
      char dir = (distanceMM >= 0) ? 'F' : 'B';
      startMoveDistance(abs(distanceMM), dir, speed);
    }
  }
  else if (cmd.startsWith("TURN ")) {
    // encoder-based turning: "TURN degrees speed"
    int space = cmd.indexOf(' ', 5);
    if (space > 0) {
      int degrees = cmd.substring(5, space).toInt();
      int speed = cmd.substring(space + 1).toInt();
      char dir = (degrees >= 0) ? 'R' : 'L';
      startRotateDegrees(abs(degrees), dir, speed);
    }
  }
  else if (cmd == "RESET_ENCODERS") {
    // reset encoder counts to zero (legacy)
    cli();
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    sei();
    movement.state = MOVE_IDLE;  // cancel any in-progress movement
    Serial.println("encoders reset to 0");
  }
  else if (cmd.startsWith("S ")) {
    int newAngle = cmd.substring(2).toInt();
    newAngle = constrain(newAngle, 0, 180);

    // only update servo if angle changed by more than 1 degree (deadband)
    if (abs(newAngle - lastServoAngle) > 1) {
      servoAngle = newAngle;

      // attach servo if it was detached
      if (!cameraServo.attached()) {
        cameraServo.attach(SERVO_PIN);
        delay(15); // small delay for servo to initialize
      }

      cameraServo.write(servoAngle);
      lastServoAngle = servoAngle;
      lastServoMove = millis();

      Serial.print("servo: ");
      Serial.println(servoAngle);
    }
  }
  else if (cmd == "SCAN") {
    performScan();
  }
  else if (cmd == "READ_DISTANCE") {
    // Read distance only when explicitly requested - force fresh reading
    readDistance(true);
  }
}

void setMotors(int left, int right) {
  if (left == 0 && right == 0) {
    strcpy(motorStatusStr, "STOPPED");
  } else if (left > 0 && right > 0) {
    strcpy(motorStatusStr, "FORWARD");
  } else if (left < 0 && right < 0) {
    strcpy(motorStatusStr, "BACKWARD");
  } else if (left > right) {
    strcpy(motorStatusStr, "RIGHT");
  } else {
    strcpy(motorStatusStr, "LEFT");
  }

  int calibratedLeft = left;
  int calibratedRight = right;

  // constrain to valid PWM range
  calibratedLeft = constrain(calibratedLeft, -255, 255);
  calibratedRight = constrain(calibratedRight, -255, 255);

  // Dead zone: PWM too low to turn motors just causes buzzing
  if (calibratedLeft != 0 && abs(calibratedLeft) < MOTOR_DEAD_ZONE) {
    calibratedLeft = 0;
  }
  if (calibratedRight != 0 && abs(calibratedRight) < MOTOR_DEAD_ZONE) {
    calibratedRight = 0;
  }

  // IMPORTANT: Set PWM to 0 FIRST before changing direction
  // This prevents brief full-speed pulses when switching directions
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delayMicroseconds(100);  // brief settling time

  // left motor - set direction then PWM
  if (calibratedLeft > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (calibratedLeft < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  // right motor - set direction then PWM
  if (calibratedRight > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (calibratedRight < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  // Now apply PWM after directions are set
  analogWrite(ENA, abs(calibratedLeft));
  analogWrite(ENB, abs(calibratedRight));
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  leftSpeed = 0;
  rightSpeed = 0;
  lastSetLeftPWM = 0;
  lastSetRightPWM = 0;
  strcpy(motorStatusStr, "STOPPED");
}

// force parameter bypasses minimum interval (used by scan)
void readDistance(bool force = false) {
  static unsigned long lastReadTime = 0;
  const unsigned long MIN_READ_INTERVAL = 60; // 60ms minimum between reads

  // enforce minimum interval to prevent echo interference (unless forced)
  if (!force && millis() - lastReadTime < MIN_READ_INTERVAL) {
    return;
  }
  lastReadTime = millis();

  // ensure trigger pin is low before starting
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);

  // send 10us trigger pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // wait for echo with timeout (25ms = ~4m max range)
  long duration = pulseIn(ECHO_PIN, HIGH, 25000);

  if (duration > 0) {
    // calculate distance: duration in microseconds
    // speed of sound = 343 m/s = 0.0343 cm/us
    // round trip, so divide by 2: distance = duration / 58.3
    int newDist = duration / 58;

    // validate reading (2cm min, 300cm max)
    if (newDist >= 2 && newDist <= 300) {
      distance = newDist;
    }
  }
  // if invalid/timeout, keep previous distance value
}

void performScan() {
  Serial.println("scanning...");

  // ensure servo is attached for scan
  if (!cameraServo.attached()) {
    cameraServo.attach(SERVO_PIN);
    delay(15);
  }

  for (int angle = 0; angle <= 180; angle += 30) {
    cameraServo.write(angle);
    lastServoMove = millis();
    delay(250);  // give servo time to settle
    readDistance(true);  // force fresh reading

    Serial.print(angle);
    Serial.print("°: ");
    Serial.print(distance);
    Serial.println("cm");
  }

  // return to center
  servoAngle = 90;
  cameraServo.write(servoAngle);
  lastServoAngle = servoAngle;
  lastServoMove = millis();
}

// Adaptive speed reduction near movement target
// Reduces speed linearly in the last 30% of movement (or last DECEL_THRESHOLD_PULSES)
int calculateAdaptiveSpeed(int baseSpeed, long remainingPulses, long totalPulses) {
  // For very short moves, use minimum speed from the start
  if (totalPulses <= 5) {
    return MIN_MOVEMENT_SPEED;
  }

  // Deceleration zone: last 30% of movement or last DECEL_THRESHOLD_PULSES, whichever is larger
  long decelZone = max((long)DECEL_THRESHOLD_PULSES, totalPulses * 3 / 10);

  if (remainingPulses <= decelZone) {
    // Linear interpolation between baseSpeed and MIN_MOVEMENT_SPEED
    int speed = MIN_MOVEMENT_SPEED +
      (int)(((long)(baseSpeed - MIN_MOVEMENT_SPEED) * remainingPulses) / decelZone);
    return max(speed, (int)MIN_MOVEMENT_SPEED);
  }

  return baseSpeed;
}

// PI encoder correction for straight-line driving
// Compares left/right encoder counts and adjusts PWM to keep wheels synchronized
// P-term handles immediate correction, I-term compensates persistent motor bias
void applyEncoderCorrection(int speed, long leftCount, long rightCount) {
  long error = abs(leftCount) - abs(rightCount);  // positive = left ahead

  // Feedforward: right motor starts with static boost to compensate for known weakness
  int leftPWM = speed;
  int rightPWM = speed + RIGHT_MOTOR_BOOST;

  if (abs(error) > ENCODER_DEAD_BAND) {
    // Accumulate integral with anti-windup clamping
    integralError = constrain(integralError + (int)error, -MAX_INTEGRAL, MAX_INTEGRAL);

    // PI correction: positive = slow left, speed up right
    int correction = KP * (int)error + KI * integralError;
    leftPWM = speed - correction / 2;
    rightPWM = speed + correction / 2 + RIGHT_MOTOR_BOOST;  // BUG FIX: preserve boost in correction path

    // Redistribute correction if one side hits the dead zone floor
    // (prevents saturation during low-speed deceleration)
    if (leftPWM < MOTOR_DEAD_ZONE) {
      int deficit = MOTOR_DEAD_ZONE - leftPWM;
      leftPWM = MOTOR_DEAD_ZONE;
      rightPWM += deficit;
    }
    if (rightPWM < MOTOR_DEAD_ZONE) {
      int deficit = MOTOR_DEAD_ZONE - rightPWM;
      rightPWM = MOTOR_DEAD_ZONE;
      leftPWM += deficit;
    }
  }

  // Clamp to valid range
  leftPWM = constrain(leftPWM, MOTOR_DEAD_ZONE, 255);
  rightPWM = constrain(rightPWM, MOTOR_DEAD_ZONE, 255);

  // Apply direction from movement context
  int finalLeft = leftPWM * movement.leftDirection;
  int finalRight = rightPWM * movement.rightDirection;

  // Only call setMotors() if values changed (avoids overhead from delayMicroseconds settling)
  if (finalLeft != lastSetLeftPWM || finalRight != lastSetRightPWM) {
    setMotors(finalLeft, finalRight);
    lastSetLeftPWM = finalLeft;
    lastSetRightPWM = finalRight;
  }
}

// non-blocking movement functions (replace blocking moveDistance/rotateDegrees)

void startMoveDistance(int distanceMM, char direction, int speed) {
  // cancel any in-progress movement
  stopMotors();
  movement.state = MOVE_IDLE;
  movement.timedOut = false;

  long pulsesNeeded = abs(distanceMM) / MM_PER_PULSE;
  if (pulsesNeeded == 0) return;  // nothing to do

  // reset encoders atomically
  cli();
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  sei();

  // reset PI controller integral for fresh movement
  integralError = 0;

  speed = constrain(abs(speed), MOTOR_DEAD_ZONE, 255);

  // calculate dynamic timeout based on pulses needed
  unsigned long timeout = BASE_MOVEMENT_TIMEOUT + (pulsesNeeded * TIMEOUT_PER_PULSE);
  timeout = min(timeout, MAX_MOVEMENT_TIMEOUT);

  // set up movement context
  movement.targetPulses = pulsesNeeded;
  movement.baseSpeed = speed;
  movement.startTime = millis();
  movement.timeout = timeout;
  movement.isRotation = false;

  // initialize stall tracking (grace period for motor spin-up)
  movement.lastEncoderChangeTime = millis();
  movement.lastLeftCount = 0;
  movement.lastRightCount = 0;

  if (direction == 'F') {
    movement.leftDirection = 1;
    movement.rightDirection = 1;
    strcpy(motorStatusStr, "FORWARD");
  } else {
    movement.leftDirection = -1;
    movement.rightDirection = -1;
    strcpy(motorStatusStr, "BACKWARD");
  }

  // start motors
  setMotors(speed * movement.leftDirection, speed * movement.rightDirection);
  movement.currentLeftPWM = speed;
  movement.currentRightPWM = speed;
  lastSetLeftPWM = speed * movement.leftDirection;
  lastSetRightPWM = speed * movement.rightDirection;

  Serial.print("move ");
  Serial.print(distanceMM);
  Serial.print("mm ");
  Serial.print(direction == 'F' ? "forward" : "backward");
  Serial.print(" (");
  Serial.print(pulsesNeeded);
  Serial.print(" pulses, timeout ");
  Serial.print(timeout);
  Serial.print("ms) at speed ");
  Serial.println(speed);

  // activate state machine last
  movement.state = MOVE_RUNNING;
}

void startRotateDegrees(int degrees, char direction, int speed) {
  // cancel any in-progress movement
  stopMotors();
  movement.state = MOVE_IDLE;
  movement.timedOut = false;

  float arcLength = (PI * WHEELBASE * abs(degrees)) / 360.0;
  long pulsesNeeded = (arcLength / MM_PER_PULSE) * TURN_CALIBRATION_FACTOR;
  if (pulsesNeeded == 0) return;  // nothing to do

  // reset encoders atomically
  cli();
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  sei();

  // reset PI controller integral for fresh movement
  integralError = 0;

  speed = constrain(abs(speed), MOTOR_DEAD_ZONE, 255);

  // calculate dynamic timeout - turns need more time than linear movement
  unsigned long timeout = BASE_MOVEMENT_TIMEOUT + (pulsesNeeded * TIMEOUT_PER_PULSE * 2);
  timeout = min(timeout, MAX_MOVEMENT_TIMEOUT);

  // set up movement context
  movement.targetPulses = pulsesNeeded;
  movement.baseSpeed = speed;
  movement.startTime = millis();
  movement.timeout = timeout;
  movement.isRotation = true;

  // initialize stall tracking (grace period for motor spin-up)
  movement.lastEncoderChangeTime = millis();
  movement.lastLeftCount = 0;
  movement.lastRightCount = 0;

  if (direction == 'L') {
    movement.leftDirection = -1;
    movement.rightDirection = 1;
    strcpy(motorStatusStr, "LEFT");
  } else {
    movement.leftDirection = 1;
    movement.rightDirection = -1;
    strcpy(motorStatusStr, "RIGHT");
  }

  // start motors with differential drive
  setMotors(speed * movement.leftDirection, speed * movement.rightDirection);
  movement.currentLeftPWM = speed;
  movement.currentRightPWM = speed;
  lastSetLeftPWM = speed * movement.leftDirection;
  lastSetRightPWM = speed * movement.rightDirection;

  Serial.print("rotate ");
  Serial.print(degrees);
  Serial.print("deg ");
  Serial.print(direction == 'L' ? "left" : "right");
  Serial.print(" (");
  Serial.print(pulsesNeeded);
  Serial.print(" pulses, timeout ");
  Serial.print(timeout);
  Serial.print("ms) at speed ");
  Serial.println(speed);

  // activate state machine last
  movement.state = MOVE_RUNNING;
}

void updateMovement() {
  if (movement.state == MOVE_IDLE) return;  // fast path

  unsigned long now = millis();

  // read encoder counts atomically once
  cli();
  long leftCount = leftEncoderCount;
  long rightCount = rightEncoderCount;
  sei();

  // timeout check applies to all active states
  if (now - movement.startTime > movement.timeout) {
    stopMotors();
    movement.timedOut = true;
    movement.settlingStart = now;
    movement.state = MOVE_SETTLING;
    Serial.print("movement timeout after ");
    Serial.print(movement.timeout);
    Serial.println("ms!");
    return;
  }

  switch (movement.state) {
    case MOVE_RUNNING: {
      // Stall detection: if motors are powered but encoders show no progress
      if (abs(leftCount) != movement.lastLeftCount || abs(rightCount) != movement.lastRightCount) {
        // encoder changed — reset stall timer
        movement.lastLeftCount = abs(leftCount);
        movement.lastRightCount = abs(rightCount);
        movement.lastEncoderChangeTime = now;
      } else if (now - movement.lastEncoderChangeTime > STALL_TIMEOUT) {
        // no encoder progress for STALL_TIMEOUT ms — wheel(s) blocked
        stopMotors();
        movement.timedOut = true;  // reuse flag for non-success path
        movement.settlingStart = now;
        movement.state = MOVE_SETTLING;
        strcpy(motorStatusStr, "STALL");
        Serial.print("STALL detected after ");
        Serial.print(STALL_TIMEOUT);
        Serial.print("ms! L=");
        Serial.print(abs(leftCount));
        Serial.print(" R=");
        Serial.println(abs(rightCount));
        return;
      }

      // calculate progress: average for linear, max for rotation
      long progress = movement.isRotation
        ? max(abs(leftCount), abs(rightCount))
        : (abs(leftCount) + abs(rightCount)) / 2;

      long remaining = movement.targetPulses - progress;

      if (remaining <= 0) {
        // target reached - stop motors and enter settling
        stopMotors();
        movement.timedOut = false;
        movement.settlingStart = now;
        movement.state = MOVE_SETTLING;
        Serial.print("movement complete: L=");
        Serial.print(abs(leftCount));
        Serial.print(" R=");
        Serial.println(abs(rightCount));
      } else {
        // Calculate adaptive speed based on distance to target
        int speed = calculateAdaptiveSpeed(movement.baseSpeed, remaining, movement.targetPulses);

        if (!movement.isRotation) {
          // Straight-line: apply P-controller encoder correction
          applyEncoderCorrection(speed, leftCount, rightCount);
        } else {
          // Rotation: no encoder correction, just adaptive speed with PWM tracking
          int finalLeft = speed * movement.leftDirection;
          int finalRight = speed * movement.rightDirection;
          if (finalLeft != lastSetLeftPWM || finalRight != lastSetRightPWM) {
            setMotors(finalLeft, finalRight);
            lastSetLeftPWM = finalLeft;
            lastSetRightPWM = finalRight;
          }
        }
      }
      break;
    }

    case MOVE_SETTLING:
      // wait 100ms for motors to physically stop (non-blocking)
      if (now - movement.settlingStart >= 100) {
        movement.state = MOVE_COMPLETE;
      }
      break;

    case MOVE_COMPLETE:
      // STALL status was already set in MOVE_RUNNING — don't overwrite
      if (strcmp(motorStatusStr, "STALL") != 0) {
        strcpy(motorStatusStr, movement.timedOut ? "TIMEOUT" : "DONE");
      }
      movement.state = MOVE_IDLE;
      Serial.print("movement ");
      if (strcmp(motorStatusStr, "STALL") == 0) {
        Serial.println("stall");
      } else {
        Serial.println(movement.timedOut ? "timeout" : "done");
      }
      break;

    default:
      break;
  }
}