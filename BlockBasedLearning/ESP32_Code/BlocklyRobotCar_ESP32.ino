// esp32 robot car blockly controller
// websocket server for block-based programming

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Wire.h>

// wifi configuration
const char* AP_SSID = "RobotCar-Blockly";
const char* AP_PASSWORD = "12345678";

// websocket server
WebSocketsServer webSocket = WebSocketsServer(81);

// i2c configuration
#define I2C_ADDRESS 0x08
#define SDA_PIN 21
#define SCL_PIN 22

// status led
#define LED_PIN 2

// connection state
bool clientConnected = false;
bool arduinoConnected = false;
unsigned long lastPing = 0;
unsigned long lastSensorRequest = 0;

// current sensor values
int currentDistance = 0;
int currentServo = 90;
int batteryPercent = 100;
String motorStatus = "STOPPED";

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Blockly Robot Car ESP32 Starting ===");

  // setup led
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // setup i2c
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 100khz for reliability

  // create wifi access point
  setupWiFiAP();

  // start websocket server
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  // check arduino connection
  checkArduino();

  Serial.println("\n=== Ready for Blockly Programming! ===");
  Serial.printf("WiFi Network: %s\n", AP_SSID);
  Serial.printf("Password: %s\n", AP_PASSWORD);
  Serial.printf("IP Address: %s\n", WiFi.softAPIP().toString().c_str());
  Serial.printf("WebSocket: ws://%s:81\n", WiFi.softAPIP().toString().c_str());
  Serial.println("=====================================");
}

void loop() {
  webSocket.loop();

  // update status led
  updateLED();

  // request sensor data from arduino periodically
  if (millis() - lastSensorRequest > 100) {
    lastSensorRequest = millis();
    requestSensorData();
  }

  // send sensor updates to connected clients
  static unsigned long lastUpdate = 0;
  if (clientConnected && millis() - lastUpdate > 250) {
    lastUpdate = millis();
    sendSensorData();
  }
}

void setupWiFiAP() {
  Serial.println("Setting up WiFi Access Point...");

  WiFi.mode(WIFI_AP);

  // configure ap
  if (WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    Serial.println("Access Point created successfully!");

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

    // print connection instructions
    Serial.println("\n----- CONNECTION INSTRUCTIONS -----");
    Serial.println("1. Connect your computer to WiFi network:");
    Serial.printf("   Network: %s\n", AP_SSID);
    Serial.printf("   Password: %s\n", AP_PASSWORD);
    Serial.println("2. Open the Blockly interface");
    Serial.printf("3. Enter IP: %s\n", IP.toString().c_str());
    Serial.println("4. Click Connect");
    Serial.println("-----------------------------------\n");
  } else {
    Serial.println("Failed to create Access Point!");
  }
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Client disconnected\n", num);
      clientConnected = false;
      // stop motors when client disconnects
      sendToArduino("M 0 0");
      break;

    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Client connected from %s\n", num, ip.toString().c_str());
        clientConnected = true;

        // send initial status
        sendStatus(num);
      }
      break;

    case WStype_TEXT:
      handleCommand(num, (char*)payload);
      break;
  }
}

void handleCommand(uint8_t num, char* payload) {
  Serial.print("Received: ");
  Serial.println(payload);

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    Serial.print("JSON parsing error: ");
    Serial.println(error.c_str());
    return;
  }

  const char* type = doc["type"];

  if (strcmp(type, "command") == 0) {
    const char* action = doc["action"];

    if (strcmp(action, "differential") == 0) {
      // motor control command
      int leftSpeed = doc["leftSpeed"];
      int rightSpeed = doc["rightSpeed"];

      // constrain speeds
      leftSpeed = constrain(leftSpeed, -255, 255);
      rightSpeed = constrain(rightSpeed, -255, 255);

      // send to arduino
      char cmd[32];
      sprintf(cmd, "M %d %d", leftSpeed, rightSpeed);
      sendToArduino(cmd);

      // send acknowledgment
      sendAck(num, true);

    } else if (strcmp(action, "servo") == 0) {
      // servo control command
      int angle = doc["angle"];
      angle = constrain(angle, 0, 180);

      // send to arduino
      char cmd[32];
      sprintf(cmd, "S %d", angle);
      sendToArduino(cmd);
      currentServo = angle;

      // send acknowledgment
      sendAck(num, true);

    } else if (strcmp(action, "scan") == 0) {
      // scan command
      sendToArduino("SCAN");
      sendAck(num, true);
    }

  } else if (strcmp(type, "ping") == 0) {
    // respond to ping
    sendPong(num);
  }
}

void sendToArduino(const char* command) {
  if (!arduinoConnected) {
    Serial.println("Warning: Arduino not connected!");
    return;
  }

  Serial.print("To Arduino: ");
  Serial.println(command);

  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write((uint8_t*)command, strlen(command));
  byte error = Wire.endTransmission();

  if (error != 0) {
    Serial.print("I2C error: ");
    Serial.println(error);
    arduinoConnected = false;
  }
}

void checkArduino() {
  Serial.println("Checking Arduino connection...");

  Wire.beginTransmission(I2C_ADDRESS);
  byte error = Wire.endTransmission();

  arduinoConnected = (error == 0);

  if (arduinoConnected) {
    Serial.println("Arduino: Connected!");
    // initialize - stop motors
    sendToArduino("M 0 0");
  } else {
    Serial.println("Arduino: Not found!");
    Serial.println("Check wiring:");
    Serial.println("  ESP32 SDA(21) -> Arduino A4");
    Serial.println("  ESP32 SCL(22) -> Arduino A5");
    Serial.println("  ESP32 GND -> Arduino GND");
  }
}

void requestSensorData() {
  if (!arduinoConnected) return;

  // request data from arduino
  Wire.requestFrom(I2C_ADDRESS, 32);

  String data = "";
  while (Wire.available()) {
    char c = Wire.read();
    if (c == '\n') break;
    data += c;
  }

  // parse response: "distance,servo,battery,status"
  if (data.length() > 0) {
    int comma1 = data.indexOf(',');
    int comma2 = data.indexOf(',', comma1 + 1);
    int comma3 = data.indexOf(',', comma2 + 1);

    if (comma1 > 0 && comma2 > comma1 && comma3 > comma2) {
      currentDistance = data.substring(0, comma1).toInt();
      currentServo = data.substring(comma1 + 1, comma2).toInt();
      batteryPercent = data.substring(comma2 + 1, comma3).toInt();
      motorStatus = data.substring(comma3 + 1);
    }
  }
}

void sendSensorData() {
  if (!clientConnected) return;

  StaticJsonDocument<256> doc;
  doc["type"] = "sensor_data";
  doc["distance"] = currentDistance;
  doc["servo_position"] = currentServo;
  doc["battery_percent"] = batteryPercent;
  doc["motor_status"] = motorStatus;
  doc["arduino_connected"] = arduinoConnected;
  doc["timestamp"] = millis();

  String json;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);
}

void sendStatus(uint8_t num) {
  StaticJsonDocument<256> doc;
  doc["type"] = "status";
  doc["arduino_connected"] = arduinoConnected;
  doc["esp_ip"] = WiFi.softAPIP().toString();

  String json;
  serializeJson(doc, json);
  webSocket.sendTXT(num, json);
}

void sendAck(uint8_t num, bool success) {
  StaticJsonDocument<128> doc;
  doc["type"] = "command_ack";
  doc["success"] = success;
  doc["timestamp"] = millis();

  String json;
  serializeJson(doc, json);
  webSocket.sendTXT(num, json);
}

void sendPong(uint8_t num) {
  StaticJsonDocument<128> doc;
  doc["type"] = "pong";
  doc["timestamp"] = millis();

  String json;
  serializeJson(doc, json);
  webSocket.sendTXT(num, json);
}

void updateLED() {
  if (clientConnected && arduinoConnected) {
    // solid on - fully connected
    digitalWrite(LED_PIN, HIGH);
  } else if (clientConnected || arduinoConnected) {
    // slow blink - partial connection
    digitalWrite(LED_PIN, (millis() / 1000) % 2);
  } else {
    // fast blink - no connection
    digitalWrite(LED_PIN, (millis() / 250) % 2);
  }
}