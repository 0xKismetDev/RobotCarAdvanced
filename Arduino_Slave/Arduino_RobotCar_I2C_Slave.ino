// arduino i2c slave - robot car controller

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

#define TRIG_PIN 9
#define ECHO_PIN 10
#define SERVO_PIN A2

// battery monitor
#define BATTERY_PIN A0
const float VOLTAGE_DIVIDER_RATIO = 2.0;  // r1=10k, r2=10k
const float ARDUINO_REFERENCE_VOLTAGE = 5.0;
const int ADC_RESOLUTION = 1024;

// battery thresholds (2s lipo)
const float BATTERY_FULL = 8.4;
const float BATTERY_NOMINAL = 7.4;
const float BATTERY_LOW = 7.0;
const float BATTERY_CRITICAL = 6.4;

Servo cameraServo;

int leftSpeed = 0;
int rightSpeed = 0;
int servoAngle = 90;
int distance = 0;
float batteryVoltage = 0.0;
int batteryPercent = 0;
String motorStatus = "STOPPED";

char cmdBuffer[32];
int cmdIndex = 0;
bool newCmd = false;

unsigned long lastSensorRead = 0;
unsigned long lastBatteryRead = 0;
const int SENSOR_INTERVAL = 100;      // 100ms
const int BATTERY_INTERVAL = 5000;    // 5s

void setup() {
  Serial.begin(115200);
  Serial.println("=== arduino robot car - i2c slave ===");
  Serial.println("i2c address: 0x08 (sda=a4, scl=a5)");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  cameraServo.attach(SERVO_PIN);
  cameraServo.write(servoAngle);

  pinMode(BATTERY_PIN, INPUT);

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

void loop() {
  if (newCmd) {
    processCommand();
    newCmd = false;
  }

  if (millis() - lastSensorRead > SENSOR_INTERVAL) {
    readDistance();
    lastSensorRead = millis();
  }

  if (!newCmd && millis() - lastBatteryRead > BATTERY_INTERVAL) {
    readBatteryVoltage();
    lastBatteryRead = millis();
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
  // send "distance,servo,battery,status"
  String response = String(distance) + "," +
                   String(servoAngle) + "," +
                   String(batteryPercent) + "," +
                   motorStatus;

  if (response.length() > 31) {
    response = response.substring(0, 31);
  }
  response += "\n";

  Wire.write(response.c_str());
}

void processCommand() {
  String cmd = String(cmdBuffer);
  cmd.trim();

  Serial.print("cmd: ");
  Serial.println(cmd);

  if (cmd.startsWith("M ")) {
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

      setMotors(leftSpeed, rightSpeed);
    }
  }
  else if (cmd.startsWith("S ")) {
    servoAngle = cmd.substring(2).toInt();
    servoAngle = constrain(servoAngle, 0, 180);
    cameraServo.write(servoAngle);
    Serial.print("servo: ");
    Serial.println(servoAngle);
  }
  else if (cmd == "SCAN") {
    performScan();
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

  // left motor
  if (left > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, abs(left));
  } else if (left < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(left));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }

  // right motor
  if (right > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, abs(right));
  } else if (right < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, abs(right));
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

  for (int angle = 0; angle <= 180; angle += 30) {
    cameraServo.write(angle);
    delay(200);
    readDistance();

    Serial.print(angle);
    Serial.print("°: ");
    Serial.print(distance);
    Serial.println("cm");
  }

  servoAngle = 90;
  cameraServo.write(servoAngle);
}

void readBatteryVoltage() {
  bool motorsRunning = (leftSpeed != 0 || rightSpeed != 0);
  static unsigned long motorStopTime = 0;
  static bool motorsWereStopped = true;

  // track when motors stop
  if (!motorsRunning) {
    if (!motorsWereStopped) {
      motorStopTime = millis();
      motorsWereStopped = true;
    }
  } else {
    motorsWereStopped = false;
    return; // skip reading while motors running
  }

  // only update if motors stopped for 2+ seconds (voltage stabilized)
  if (millis() - motorStopTime < 2000) {
    return;
  }

  // average 5 readings for stability
  float totalVoltage = 0;
  const int numReadings = 5;

  for (int i = 0; i < numReadings; i++) {
    int analogValue = analogRead(BATTERY_PIN);
    float pinVoltage = (analogValue * ARDUINO_REFERENCE_VOLTAGE) / ADC_RESOLUTION;
    totalVoltage += (pinVoltage * VOLTAGE_DIVIDER_RATIO);
    delay(2);
  }

  batteryVoltage = totalVoltage / numReadings;

  // calculate percentage
  if (batteryVoltage >= BATTERY_FULL) {
    batteryPercent = 100;
  } else if (batteryVoltage <= BATTERY_CRITICAL) {
    batteryPercent = 0;
  } else {
    batteryPercent = ((batteryVoltage - BATTERY_CRITICAL) / (BATTERY_FULL - BATTERY_CRITICAL)) * 100;
  }

  Serial.print("battery: ");
  Serial.print(batteryVoltage, 2);
  Serial.print("V (");
  Serial.print(batteryPercent);
  Serial.print("%) - ");

  if (batteryVoltage >= BATTERY_NOMINAL) {
    Serial.println("good");
  } else if (batteryVoltage >= BATTERY_LOW) {
    Serial.println("low");
  } else if (batteryVoltage >= BATTERY_CRITICAL) {
    Serial.println("critical!");
  } else {
    Serial.println("empty!");
  }
}