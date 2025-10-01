// esp32 robot car websocket server - i2c master to arduino

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <string.h>

// wifi - try to connect to camera, fallback to own ap
const char* CAM_SSID = "ESP32-CAM-STREAM";
const char* CAM_PASSWORD = "12345678";
const char* FALLBACK_SSID = "RobotCar-AP";
const char* FALLBACK_PASSWORD = "12345678";
String cameraIP = "192.168.4.1";

WebSocketsServer webSocket = WebSocketsServer(81);

// i2c to arduino
#define I2C_ADDRESS 0x08
#define SDA_PIN 21
#define SCL_PIN 22

#define LED_PIN 2
#define FLASH_CONTROL_PIN 25

bool clientConnected = false;
bool arduinoConnected = false;
bool cameraConnected = false;
bool flashEnabled = false;
bool isAPMode = false;
unsigned long lastWiFiCheck = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 Robot Car Master Starting ===");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(FLASH_CONTROL_PIN, OUTPUT);
  digitalWrite(FLASH_CONTROL_PIN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  connectToCamera();

  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  checkArduino();

  Serial.println("=== Ready! ===");
  if (isAPMode) {
    Serial.printf("AP Mode - IP: %s\n", WiFi.softAPIP().toString().c_str());
  } else {
    Serial.printf("Station Mode - IP: %s\n", WiFi.localIP().toString().c_str());
  }
  Serial.printf("WebSocket: ws://%s:81\n", isAPMode ? WiFi.softAPIP().toString().c_str() : WiFi.localIP().toString().c_str());
  if (cameraConnected) {
    Serial.printf("Camera Stream: http://%s/stream\n", cameraIP.c_str());
  }
}

void loop() {
  webSocket.loop();
  updateLED();

  // send sensor data every 250ms
  static unsigned long lastSensorTime = 0;
  if (millis() - lastSensorTime > 250) {
    lastSensorTime = millis();
    sendSensorData();
  }

  // check wifi if in station mode
  if (!isAPMode && millis() - lastWiFiCheck > 5000) {
    lastWiFiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("wifi lost, reconnecting...");
      connectToCamera();
    }
  }
}

void connectToCamera() {
  // try connecting to camera first
  Serial.println("trying to connect to camera wifi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(CAM_SSID, CAM_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nconnected to camera!");
    Serial.print("ip: ");
    Serial.println(WiFi.localIP());
    cameraConnected = true;
    isAPMode = false;
  } else {
    // camera not found, fallback to own ap mode
    Serial.println("\ncamera not found, starting own ap...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(FALLBACK_SSID, FALLBACK_PASSWORD);
    Serial.print("ap started - ip: ");
    Serial.println(WiFi.softAPIP());
    cameraConnected = false;
    isAPMode = true;
    cameraIP = "";
  }
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] disconnected\n", num);
      clientConnected = false;
      sendToArduino("M 0 0"); // stop motors for safety
      break;

    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] connected from %s\n", num, ip.toString().c_str());
        clientConnected = true;
        sendStatus(num);
      }
      break;

    case WStype_TEXT:
      handleCommand(num, (char*)payload);
      break;
  }
}

void handleCommand(uint8_t num, char* payload) {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    Serial.print("json error: ");
    Serial.println(error.c_str());
    return;
  }

  const char* type = doc["type"];

  if (strcmp(type, "command") == 0) {
    const char* action = doc["action"];

    if (strcmp(action, "differential") == 0) {
      int leftSpeed = doc["leftSpeed"];
      int rightSpeed = doc["rightSpeed"];
      char cmd[32];
      sprintf(cmd, "M %d %d", leftSpeed, rightSpeed);
      sendToArduino(cmd);
      sendAck(num, true);

    } else if (strcmp(action, "servo") == 0) {
      int angle = doc["angle"];
      char cmd[32];
      sprintf(cmd, "S %d", angle);
      sendToArduino(cmd);
      sendAck(num, true);

    } else if (strcmp(action, "scan") == 0) {
      sendToArduino("SCAN");
      sendAck(num, true);

    } else if (strcmp(action, "flash") == 0) {
      bool state = doc["state"] | false;
      controlCameraFlash(state);
      sendAck(num, true);
    }

  } else if (strcmp(type, "ping") == 0) {
    sendPong(num);
  }
}

void sendToArduino(const char* command) {
  if (!arduinoConnected) {
    Serial.println("arduino not connected!");
    return;
  }

  Serial.print("-> arduino: ");
  Serial.println(command);

  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write((uint8_t*)command, strlen(command));
  Wire.endTransmission();
}

void checkArduino() {
  Wire.beginTransmission(I2C_ADDRESS);
  byte error = Wire.endTransmission();

  arduinoConnected = (error == 0);

  if (arduinoConnected) {
    Serial.println("arduino: connected");
    sendToArduino("M 0 0"); // stop motors on startup
  } else {
    Serial.println("arduino: not found! check i2c wiring (sda=21->a4, scl=22->a5)");
  }
}

void sendSensorData() {
  if (!clientConnected) return;

  Wire.requestFrom(I2C_ADDRESS, 32);

  String data = "";
  while (Wire.available()) {
    char c = Wire.read();
    if (c == '\n') break;
    data += c;
  }

  // parse "distance,servo,battery,status"
  int distance = -1;
  int servo = 90;
  int battery = 0;
  String status = "STOPPED";

  int comma1 = data.indexOf(',');
  int comma2 = data.indexOf(',', comma1 + 1);
  int comma3 = data.indexOf(',', comma2 + 1);

  if (comma1 > 0 && comma2 > comma1 && comma3 > comma2) {
    distance = data.substring(0, comma1).toInt();
    servo = data.substring(comma1 + 1, comma2).toInt();
    battery = data.substring(comma2 + 1, comma3).toInt();
    status = data.substring(comma3 + 1);
  }

  StaticJsonDocument<256> doc;
  doc["type"] = "sensor_data";
  doc["distance"] = distance;
  doc["servo_position"] = servo;
  doc["battery_percent"] = battery;
  doc["motor_status"] = status;
  doc["arduino_connected"] = arduinoConnected;
  doc["camera_connected"] = cameraConnected;
  doc["camera_ip"] = cameraIP;
  doc["timestamp"] = millis();

  String json;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);
}

void sendStatus(uint8_t num) {
  StaticJsonDocument<256> doc;
  doc["type"] = "status";
  doc["arduino_connected"] = arduinoConnected;
  doc["camera_connected"] = cameraConnected;
  doc["camera_ip"] = cameraIP;

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
    digitalWrite(LED_PIN, HIGH); // solid on
  } else if (clientConnected || arduinoConnected) {
    digitalWrite(LED_PIN, (millis() / 1000) % 2); // slow blink
  } else {
    digitalWrite(LED_PIN, (millis() / 250) % 2); // fast blink
  }
}

void controlCameraFlash(bool state) {
  flashEnabled = state;
  digitalWrite(FLASH_CONTROL_PIN, state ? HIGH : LOW);
  Serial.printf("flash gpio: %s\n", state ? "HIGH" : "LOW");
}