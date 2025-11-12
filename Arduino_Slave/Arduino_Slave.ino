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

// motor calibration - adjust if robot drifts
// values > 1.0 speed up, < 1.0 slow down
const float LEFT_MOTOR_FACTOR = 1.05;   // increase if drifting right
const float RIGHT_MOTOR_FACTOR = 0.90;  // decrease if drifting right - MORE AGGRESSIVE

// turn calibration - compensates for wheel slippage during rotation
// increase if turns are too shallow, decrease if turns are too far
const float TURN_CALIBRATION_FACTOR = 1.12;  // 12% more rotation to account for slippage

// encoder variables (volatile for ISR access)
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile byte lastLeftState = HIGH;
volatile byte lastRightState = HIGH;

// movement control variables
long targetLeftCount = 0;
long targetRightCount = 0;
bool encoderMovementActive = false;
unsigned long movementStartTime = 0;
const unsigned long MOVEMENT_TIMEOUT = 5000;  // 5 seconds timeout
const int POSITION_TOLERANCE = 2;  // allow 2 pulses overshoot

Servo cameraServo;

int leftSpeed = 0;
int rightSpeed = 0;
int servoAngle = 90;
int lastServoAngle = 90;  // track last written angle
int distance = 0;
String motorStatus = "STOPPED";
unsigned long lastServoMove = 0;
const unsigned long SERVO_DETACH_TIMEOUT = 2000; // detach servo after 2s of inactivity

char cmdBuffer[32];
int cmdIndex = 0;
bool newCmd = false;

unsigned long lastSensorRead = 0;
const int SENSOR_INTERVAL = 100;      // 100ms

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

// pin change interrupt service routine
ISR(PCINT1_vect) {
  // read current states
  byte leftState = digitalRead(LEFT_ENCODER_PIN);
  byte rightState = digitalRead(RIGHT_ENCODER_PIN);

  // detect rising edges (LOW to HIGH transitions)
  if (leftState == HIGH && lastLeftState == LOW) {
    leftEncoderCount++;
  }

  if (rightState == HIGH && lastRightState == LOW) {
    rightEncoderCount++;
  }

  // save states for next comparison
  lastLeftState = leftState;
  lastRightState = rightState;
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

  // legacy encoder movement check
  if (encoderMovementActive) {
    checkEncoderMovement();
  }

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

  Serial.print("i2c rx (");
  Serial.print(bytes);
  Serial.print(" bytes): ");
  Serial.println(cmdBuffer);

  digitalWrite(LED_BUILTIN, LOW);
}

void onRequest() {
  // Simplified response - only essentials
  // "distance,leftEncoder,rightEncoder,status"
  String response = String(distance) + "," +
                   String(leftEncoderCount) + "," +
                   String(rightEncoderCount) + "," +
                   motorStatus + "\n";  // Add newline for parsing

  if (response.length() > 31) {
    response = response.substring(0, 31);
  }

  Wire.write(response.c_str());
}

void processCommand(const char* command) {
  String cmd = String(command);
  cmd.trim();

  Serial.print("cmd: ");
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

      encoderMovementActive = false;  // cancel encoder movement
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
      moveDistance(distanceMM, direction, speed);
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
      rotateDegrees(degrees, direction, speed);
    }
  }
  else if (cmd == "E") {
    // reset encoder counts to zero
    cli();
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    encoderMovementActive = false;
    sei();
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
      moveDistance(abs(distanceMM), dir, speed);
    }
  }
  else if (cmd.startsWith("TURN ")) {
    // encoder-based turning: "TURN degrees speed"
    int space = cmd.indexOf(' ', 5);
    if (space > 0) {
      int degrees = cmd.substring(5, space).toInt();
      int speed = cmd.substring(space + 1).toInt();
      char dir = (degrees >= 0) ? 'R' : 'L';
      rotateDegrees(abs(degrees), dir, speed);
    }
  }
  else if (cmd == "RESET_ENCODERS") {
    // reset encoder counts to zero (legacy)
    cli();
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    encoderMovementActive = false;
    sei();
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
    // Read distance only when explicitly requested
    readDistance();
  }
}

void setMotors(int left, int right) {
  if (left == 0 && right == 0) {
    motorStatus = "STOPPED";
  } else if (left > 0 && right > 0) {
    motorStatus = "FORWARD";
  } else if (left < 0 && right < 0) {
    motorStatus = "BACKWARD";
  } else if (left > right) {
    motorStatus = "RIGHT";
  } else {
    motorStatus = "LEFT";
  }

  // apply motor calibration factors
  int calibratedLeft = left * LEFT_MOTOR_FACTOR;
  int calibratedRight = right * RIGHT_MOTOR_FACTOR;

  // constrain to valid PWM range
  calibratedLeft = constrain(calibratedLeft, -255, 255);
  calibratedRight = constrain(calibratedRight, -255, 255);

  // left motor
  if (calibratedLeft > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, abs(calibratedLeft));
  } else if (calibratedLeft < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(calibratedLeft));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }

  // right motor
  if (calibratedRight > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, abs(calibratedRight));
  } else if (calibratedRight < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, abs(calibratedRight));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
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
  motorStatus = "STOPPED";
}

void readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  int newDist = duration * 0.017; // cm

  if (newDist > 2 && newDist < 400) {
    distance = newDist;
  }
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
    delay(200);
    readDistance();

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

// encoder-based movement functions

void moveDistance(int distanceMM, char direction, int speed) {
  long pulsesNeeded = abs(distanceMM) / MM_PER_PULSE;

  // reset encoders atomically
  cli();
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  sei();

  // set direction: 'F' = forward, 'B' = backward
  speed = constrain(abs(speed), 0, 255);
  if (direction == 'B') {
    speed = -speed;
  }

  Serial.print("move ");
  Serial.print(distanceMM);
  Serial.print("mm ");
  Serial.print(direction == 'F' ? "forward" : "backward");
  Serial.print(" (");
  Serial.print(pulsesNeeded);
  Serial.print(" pulses) at speed ");
  Serial.println(abs(speed));

  setMotors(speed, speed);
  unsigned long startTime = millis();

  // blocking loop until target reached
  while (true) {
    cli();
    long leftCount = leftEncoderCount;
    long rightCount = rightEncoderCount;
    sei();

    long avgPulses = (abs(leftCount) + abs(rightCount)) / 2;

    if (avgPulses >= (pulsesNeeded - POSITION_TOLERANCE)) {
      stopMotors();
      Serial.print("movement complete: L=");
      Serial.print(leftCount);
      Serial.print(" R=");
      Serial.println(rightCount);
      break;
    }

    if (millis() - startTime > MOVEMENT_TIMEOUT) {
      stopMotors();
      Serial.println("movement timeout!");
      break;
    }

    delay(5);
  }

  encoderMovementActive = false;
}

void rotateDegrees(int degrees, char direction, int speed) {
  // calculate arc length with calibration for slippage
  float arcLength = (PI * WHEELBASE * abs(degrees)) / 360.0;
  long pulsesNeeded = (arcLength / MM_PER_PULSE) * TURN_CALIBRATION_FACTOR;

  // reset encoders atomically
  cli();
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  sei();

  speed = constrain(abs(speed), 0, 255);

  Serial.print("rotate ");
  Serial.print(degrees);
  Serial.print("° ");
  Serial.print(direction == 'L' ? "left" : "right");
  Serial.print(" (");
  Serial.print(pulsesNeeded);
  Serial.print(" pulses) at speed ");
  Serial.println(speed);

  // differential drive: opposite wheel directions
  if (direction == 'L') {
    setMotors(-speed, speed);
  } else {
    setMotors(speed, -speed);
  }

  unsigned long startTime = millis();

  // blocking loop until target reached
  while (true) {
    cli();
    long leftCount = abs(leftEncoderCount);
    long rightCount = abs(rightEncoderCount);
    sei();

    long maxPulses = max(leftCount, rightCount);

    if (maxPulses >= (pulsesNeeded - POSITION_TOLERANCE)) {
      stopMotors();
      Serial.print("rotation complete: L=");
      Serial.print(leftCount);
      Serial.print(" R=");
      Serial.println(rightCount);
      break;
    }

    if (millis() - startTime > MOVEMENT_TIMEOUT) {
      stopMotors();
      Serial.println("rotation timeout!");
      break;
    }

    delay(5);
  }

  encoderMovementActive = false;
}

void checkEncoderMovement() {
  // check if targets reached
  bool leftReached = (leftEncoderCount >= targetLeftCount - POSITION_TOLERANCE);
  bool rightReached = (rightEncoderCount >= targetRightCount - POSITION_TOLERANCE);

  // check timeout
  bool timeout = (millis() - movementStartTime > MOVEMENT_TIMEOUT);

  if ((leftReached && rightReached) || timeout) {
    // stop motors
    stopMotors();
    encoderMovementActive = false;
  }

  // No collision detection here - only when explicitly requested
  // This removes the overhead of constant distance checking
}