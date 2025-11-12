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
unsigned long lastClientActivity = 0;
const unsigned long CLIENT_TIMEOUT = 15000; // 15 seconds timeout
uint8_t connectedClientNum = 255; // track which client number is connected

// current sensor values
int currentDistance = 0;
int currentServo = 90;
int batteryPercent = 100;
String motorStatus = "STOPPED";
long leftEncoderCount = 0;
long rightEncoderCount = 0;

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

  // check for client timeout (stale connections)
  if (clientConnected && millis() - lastClientActivity > CLIENT_TIMEOUT) {
    Serial.println("Client timeout - disconnecting stale connection");
    if (connectedClientNum != 255) {
      webSocket.disconnect(connectedClientNum);
    }
    clientConnected = false;
    connectedClientNum = 255;
    sendToArduino("M 0 0"); // stop motors for safety
  }

  // Only request sensor data occasionally - reduce overhead
  if (millis() - lastSensorRequest > 500) {  // Changed from 100ms to 500ms
    lastSensorRequest = millis();
    requestSensorData();
  }

  // send sensor updates less frequently
  static unsigned long lastUpdate = 0;
  if (clientConnected && millis() - lastUpdate > 1000) {  // Changed from 250ms to 1000ms
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
      if (num == connectedClientNum) {
        clientConnected = false;
        connectedClientNum = 255;
        // stop motors when client disconnects
        sendToArduino("M 0 0");
      }
      break;

    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Client connected from %s\n", num, ip.toString().c_str());

        // if there's already a connected client, disconnect it
        if (clientConnected && connectedClientNum != 255 && connectedClientNum != num) {
          Serial.printf("Disconnecting old client [%u]\n", connectedClientNum);
          webSocket.disconnect(connectedClientNum);
        }

        clientConnected = true;
        connectedClientNum = num;
        lastClientActivity = millis();

        // send initial status
        sendStatus(num);
      }
      break;

    case WStype_TEXT:
      if (num == connectedClientNum) {
        lastClientActivity = millis();
      }
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

    } else if (strcmp(action, "read_distance") == 0) {
      // read distance on demand
      sendToArduino("READ_DISTANCE");
      sendAck(num, true);

    } else if (strcmp(action, "moveDistance") == 0) {
      // distance-based movement: {"action": "moveDistance", "distance": 100, "direction": "forward", "speed": 150}
      int distance = doc["distance"];
      const char* direction = doc["direction"];
      int speed = doc["speed"];

      // constrain values
      distance = constrain(abs(distance), 0, 1000);  // max 1 meter
      speed = constrain(speed, 0, 255);

      // convert direction to single char
      char dir = (strcmp(direction, "forward") == 0) ? 'F' : 'B';

      char cmd[32];
      sprintf(cmd, "D %d %c %d", distance, dir, speed);
      sendToArduino(cmd);
      sendAck(num, true);

    } else if (strcmp(action, "rotateDegrees") == 0) {
      // rotation-based turning: {"action": "rotateDegrees", "degrees": 90, "direction": "left", "speed": 150}
      int degrees = doc["degrees"];
      const char* direction = doc["direction"];
      int speed = doc["speed"];

      // constrain values
      degrees = constrain(abs(degrees), 0, 360);
      speed = constrain(speed, 0, 255);

      // convert direction to single char
      char dir = (strcmp(direction, "left") == 0) ? 'L' : 'R';

      char cmd[32];
      sprintf(cmd, "R %d %c %d", degrees, dir, speed);
      sendToArduino(cmd);
      sendAck(num, true);

    } else if (strcmp(action, "resetEncoders") == 0) {
      // reset encoder counts
      sendToArduino("E");
      leftEncoderCount = 0;
      rightEncoderCount = 0;
      sendAck(num, true);

    } else if (strcmp(action, "getEncoders") == 0) {
      // query encoder values
      sendToArduino("Q");
      sendAck(num, true);

    // Legacy command support for backward compatibility
    } else if (strcmp(action, "move") == 0) {
      // encoder-based movement: {"action": "move", "distance": 100, "speed": 150}
      int distance = doc["distance"];
      int speed = doc["speed"];

      // constrain values
      speed = constrain(speed, 0, 255);
      char dir = (distance >= 0) ? 'F' : 'B';
      distance = abs(constrain(distance, -1000, 1000));

      char cmd[32];
      sprintf(cmd, "D %d %c %d", distance, dir, speed);
      sendToArduino(cmd);
      sendAck(num, true);

    } else if (strcmp(action, "turn") == 0) {
      // encoder-based turning: {"action": "turn", "degrees": 90, "speed": 150}
      int degrees = doc["degrees"];
      int speed = doc["speed"];

      // constrain values
      speed = constrain(speed, 0, 255);
      char dir = (degrees >= 0) ? 'R' : 'L';
      degrees = abs(constrain(degrees, -360, 360));

      char cmd[32];
      sprintf(cmd, "R %d %c %d", degrees, dir, speed);
      sendToArduino(cmd);
      sendAck(num, true);

    } else if (strcmp(action, "reset_encoders") == 0) {
      // reset encoder counts (legacy)
      sendToArduino("E");
      leftEncoderCount = 0;
      rightEncoderCount = 0;
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

  // parse response: "distance,leftEncoder,rightEncoder,status"
  if (data.length() > 0) {
    int comma1 = data.indexOf(',');
    int comma2 = data.indexOf(',', comma1 + 1);
    int comma3 = data.indexOf(',', comma2 + 1);

    if (comma1 > 0 && comma2 > comma1 && comma3 > comma2) {
      currentDistance = data.substring(0, comma1).toInt();
      leftEncoderCount = data.substring(comma1 + 1, comma2).toInt();
      rightEncoderCount = data.substring(comma2 + 1, comma3).toInt();
      motorStatus = data.substring(comma3 + 1);
    }
  }
}

void sendSensorData() {
  if (!clientConnected) return;

  StaticJsonDocument<256> doc;
  doc["type"] = "sensor_data";
  doc["distance"] = currentDistance;
  doc["motor_status"] = motorStatus;
  doc["left_encoder"] = leftEncoderCount;
  doc["right_encoder"] = rightEncoderCount;
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