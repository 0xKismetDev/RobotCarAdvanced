// esp32 robot car blockly controller
// websocket server for block-based programming
// Phase 3 completion signaling + OLED debug display

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_wifi.h"  // for disabling power save
#include "esp_task_wdt.h"  // watchdog timer

// OLED display on separate I2C bus (Wire1)
#define OLED_SDA 16
#define OLED_SCL 17
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

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
unsigned long lastServerPing = 0;
unsigned long lastSensorRequest = 0;
unsigned long lastClientActivity = 0;
unsigned long lastArduinoCheck = 0;
unsigned long lastPongReceived = 0;
int missedPongs = 0;

// timing constants - tuned for reliability
const unsigned long CLIENT_TIMEOUT = 10000;      // 10 seconds - faster detection
const unsigned long SERVER_PING_INTERVAL = 2000; // ping client every 2 seconds
const unsigned long SENSOR_REQUEST_INTERVAL = 250; // 250ms sensor polling
const unsigned long SENSOR_BROADCAST_INTERVAL = 500; // 500ms broadcasts (acts as keepalive)
const unsigned long ARDUINO_CHECK_INTERVAL = 5000; // check arduino every 5 seconds
const unsigned long MEMORY_CHECK_INTERVAL = 30000; // check memory every 30 seconds
const unsigned long I2C_READ_TIMEOUT = 50; // max 50ms for I2C read
const int MAX_MISSED_PONGS = 3; // disconnect after 3 missed pongs
const uint32_t MIN_FREE_HEAP = 15000; // restart if heap below 15KB

uint8_t connectedClientNum = 255; // track which client number is connected
unsigned long lastMemoryCheck = 0;
uint32_t minFreeHeap = UINT32_MAX;

// current sensor values
int currentDistance = 0;
int currentServo = 90;
int batteryPercent = 100;
float currentBatteryVoltage = 0.0;  // 0 = not available (no voltage divider)
String motorStatus = "STOPPED";
String lastMotorStatus = "STOPPED";
long leftEncoderCount = 0;
long rightEncoderCount = 0;

// forward declarations
void sendAck(uint8_t num, bool success, int commandId = -1);
void sendMovementComplete(const String& reason);
void checkMemoryHealth();

// OLED debug display task — runs on core 0 to keep main loop responsive
void oledTask(void* parameter) {
  Wire1.begin(OLED_SDA, OLED_SCL);
  Wire1.setClock(400000);  // 400kHz for OLED

  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED: init failed!");
    vTaskDelete(NULL);
    return;
  }

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println("RobotCar");
  oled.println("Starting...");
  oled.display();
  Serial.println("OLED: OK on Wire1 (GPIO16/17)");

  for (;;) {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);

    // Row 1: title + WiFi status
    oled.setCursor(0, 0);
    oled.print("RobotCar ");
    oled.print(clientConnected ? "[CONN]" : "[----]");

    // Row 2: motor status (larger text)
    oled.setTextSize(2);
    oled.setCursor(0, 12);
    oled.print(motorStatus);

    // Row 3: encoder counts
    oled.setTextSize(1);
    oled.setCursor(0, 32);
    oled.print("L:");
    oled.print(leftEncoderCount);
    oled.print("  R:");
    oled.print(rightEncoderCount);

    // Row 4: ultrasonic distance + battery voltage
    oled.setCursor(0, 44);
    oled.print("Dist:");
    oled.print(currentDistance);
    oled.print("cm ");
    if (currentBatteryVoltage > 0) {
      oled.print(currentBatteryVoltage, 1);
      oled.print("V");
    }

    // Row 5: heap + arduino status
    oled.setCursor(0, 56);
    oled.print("Heap:");
    oled.print(ESP.getFreeHeap() / 1024);
    oled.print("K Ard:");
    oled.print(arduinoConnected ? "OK" : "NO");

    oled.display();
    vTaskDelay(pdMS_TO_TICKS(200));  // 5Hz refresh
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Blockly Robot Car ESP32 Starting ===");
  Serial.println("Phase 3 + OLED Debug Active");

  // setup watchdog timer - ESP32 Arduino 3.x API
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 15000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
  Serial.println("Watchdog timer: ENABLED (15s)");

  // setup led
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // setup i2c
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 100khz for reliability

  // create wifi access point
  setupWiFiAP();

  // CRITICAL: Disable WiFi power save mode
  // This is the #1 cause of random disconnections
  esp_wifi_set_ps(WIFI_PS_NONE);
  Serial.println("WiFi power save: DISABLED");

  // start websocket server with reliability settings
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  // enable websocket library heartbeat (ping/pong at protocol level)
  // interval, timeout, disconnect on fail
  webSocket.enableHeartbeat(3000, 2000, 2);
  Serial.println("WebSocket heartbeat: ENABLED (3s interval, 2s timeout)");

  // check arduino connection
  checkArduino();

  // start OLED debug display on core 0
  xTaskCreatePinnedToCore(
    oledTask,   // function
    "OLED",     // name
    4096,       // stack size (bytes)
    NULL,       // parameters
    1,          // priority (low — display is non-critical)
    NULL,       // task handle
    0           // core 0 (keeps main loop on core 1)
  );

  Serial.println("\n=== Ready for Blockly Programming! ===");
  Serial.printf("WiFi Network: %s\n", AP_SSID);
  Serial.printf("Password: %s\n", AP_PASSWORD);
  Serial.printf("IP Address: %s\n", WiFi.softAPIP().toString().c_str());
  Serial.printf("WebSocket: ws://%s:81\n", WiFi.softAPIP().toString().c_str());
  Serial.println("Reliability features: power_save=OFF, heartbeat=ON, OLED=ON");
  Serial.println("=====================================");
}

void loop() {
  // feed watchdog timer
  esp_task_wdt_reset();

  // process websocket - call frequently for best responsiveness
  webSocket.loop();

  unsigned long now = millis();

  // update status led
  updateLED();

  // periodic memory health check
  if (now - lastMemoryCheck > MEMORY_CHECK_INTERVAL) {
    lastMemoryCheck = now;
    checkMemoryHealth();
  }

  // server-side ping - actively check if client is alive
  if (clientConnected && now - lastServerPing > SERVER_PING_INTERVAL) {
    lastServerPing = now;
    sendServerPing();
  }

  // check for client timeout (no activity)
  if (clientConnected && now - lastClientActivity > CLIENT_TIMEOUT) {
    Serial.println("Client timeout - no activity for 10 seconds");
    disconnectClient("timeout");
  }

  // check for missed pongs (client not responding to our pings)
  if (clientConnected && missedPongs >= MAX_MISSED_PONGS) {
    Serial.printf("Client unresponsive - %d missed pongs\n", missedPongs);
    disconnectClient("missed_pongs");
  }

  // request sensor data from arduino - more frequent for responsiveness
  if (now - lastSensorRequest > SENSOR_REQUEST_INTERVAL) {
    lastSensorRequest = now;
    requestSensorData();
  }

  // broadcast sensor updates - acts as keepalive
  static unsigned long lastBroadcast = 0;
  if (clientConnected && now - lastBroadcast > SENSOR_BROADCAST_INTERVAL) {
    lastBroadcast = now;
    sendSensorData();
  }

  // periodic arduino reconnection check
  if (!arduinoConnected && now - lastArduinoCheck > ARDUINO_CHECK_INTERVAL) {
    lastArduinoCheck = now;
    Serial.println("Attempting Arduino reconnection...");
    checkArduino();
  }
}

// WiFi event handler for debugging connection issues
void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      Serial.println("[WiFi] Station connected to AP");
      break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
      Serial.println("[WiFi] Station disconnected from AP");
      // if websocket client was from this station, they're gone
      if (clientConnected) {
        Serial.println("[WiFi] WebSocket client likely lost");
      }
      break;
    case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
      Serial.println("[WiFi] Station assigned IP");
      break;
    default:
      break;
  }
}

void setupWiFiAP() {
  Serial.println("Setting up WiFi Access Point...");

  // register wifi event handler for debugging
  WiFi.onEvent(onWiFiEvent);

  WiFi.mode(WIFI_AP);

  // configure AP with optimized settings
  // channel 1, not hidden, max 4 connections
  if (WiFi.softAP(AP_SSID, AP_PASSWORD, 1, 0, 4)) {
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
  unsigned long now = millis();

  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client %u disconnected\n", num);
      if (num == connectedClientNum) {
        clientConnected = false;
        connectedClientNum = 255;
        missedPongs = 0;
        // stop motors when client disconnects
        sendToArduino("M 0 0");
        Serial.println("[WS] Motors stopped for safety");
      }
      break;

    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[WS] Client %u connected from %s\n", num, ip.toString().c_str());

        // if there's already a connected client, disconnect it
        if (clientConnected && connectedClientNum != 255 && connectedClientNum != num) {
          Serial.printf("[WS] Disconnecting old client %u\n", connectedClientNum);
          webSocket.disconnect(connectedClientNum);
        }

        clientConnected = true;
        connectedClientNum = num;
        lastClientActivity = now;
        lastPongReceived = now;
        missedPongs = 0;

        // send initial status
        sendStatus(num);
        Serial.println("[WS] Connection established, status sent");
      }
      break;

    case WStype_TEXT:
      // any text message counts as activity
      lastClientActivity = now;
      handleCommand(num, (char*)payload);
      break;

    case WStype_PONG:
      // client responded to our ping (or library heartbeat)
      lastClientActivity = now;
      lastPongReceived = now;
      missedPongs = 0;
      // Serial.println("[WS] Pong received"); // uncomment for debugging
      break;

    case WStype_PING:
      // client sent us a ping - activity indicator
      lastClientActivity = now;
      break;

    case WStype_ERROR:
      Serial.printf("[WS] Error on client %u\n", num);
      break;

    default:
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

    // extract command ID for tracking (browser uses this to match ACKs)
    int cmdId = doc["id"] | -1;

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

      // send acknowledgment with command ID
      sendAck(num, true, cmdId);

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
      sendAck(num, true, cmdId);

    } else if (strcmp(action, "scan") == 0) {
      // scan command
      sendToArduino("SCAN");
      sendAck(num, true, cmdId);

    } else if (strcmp(action, "read_distance") == 0) {
      // read distance on demand - tell Arduino to take fresh reading
      sendToArduino("READ_DISTANCE");
      sendAck(num, true, cmdId);

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
      sendAck(num, true, cmdId);

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
      sendAck(num, true, cmdId);

    } else if (strcmp(action, "resetEncoders") == 0) {
      // reset encoder counts
      sendToArduino("E");
      leftEncoderCount = 0;
      rightEncoderCount = 0;
      sendAck(num, true, cmdId);

    } else if (strcmp(action, "getEncoders") == 0) {
      // query encoder values
      sendToArduino("Q");
      sendAck(num, true, cmdId);

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
      sendAck(num, true, cmdId);

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
      sendAck(num, true, cmdId);

    } else if (strcmp(action, "reset_encoders") == 0) {
      // reset encoder counts (legacy)
      sendToArduino("E");
      leftEncoderCount = 0;
      rightEncoderCount = 0;
      sendAck(num, true, cmdId);
    }

  } else if (strcmp(type, "ping") == 0) {
    // respond to ping from client
    sendPong(num);

  } else if (strcmp(type, "pong") == 0) {
    // client responded to our ping
    lastPongReceived = millis();
    missedPongs = 0;
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

  // request data from arduino with timeout protection
  Wire.requestFrom(I2C_ADDRESS, 32);

  String data = "";
  unsigned long readStart = millis();
  int charCount = 0;

  // read with timeout and character limit to prevent freeze
  while (millis() - readStart < I2C_READ_TIMEOUT && charCount < 32) {
    if (Wire.available()) {
      char c = Wire.read();
      charCount++;
      if (c == '\n' || c == '\0') break;
      data += c;
    }
  }

  // flush any remaining bytes
  while (Wire.available()) {
    Wire.read();
  }

  // parse response: "distance,leftEncoder,rightEncoder,status[,battMV]"
  if (data.length() > 0) {
    int comma1 = data.indexOf(',');
    int comma2 = (comma1 > 0) ? data.indexOf(',', comma1 + 1) : -1;
    int comma3 = (comma2 > 0) ? data.indexOf(',', comma2 + 1) : -1;

    // validate first 3 commas found before parsing
    if (comma1 > 0 && comma2 > comma1 && comma3 > comma2) {
      int newDistance = data.substring(0, comma1).toInt();
      long newLeftEncoder = data.substring(comma1 + 1, comma2).toInt();
      long newRightEncoder = data.substring(comma2 + 1, comma3).toInt();

      // Check for optional 5th field (battery millivolts)
      int comma4 = data.indexOf(',', comma3 + 1);
      String newMotorStatus;

      if (comma4 > comma3) {
        // New 5-field format: distance,leftEnc,rightEnc,status,battMV
        newMotorStatus = data.substring(comma3 + 1, comma4);
        int batteryMV = data.substring(comma4 + 1).toInt();
        if (batteryMV > 0 && batteryMV < 15000) {
          currentBatteryVoltage = batteryMV / 1000.0;
        }
      } else {
        // Old 4-field format: distance,leftEnc,rightEnc,status
        newMotorStatus = data.substring(comma3 + 1);
      }

      // validate parsed values before accepting
      if (newDistance >= 0 && newDistance <= 500) {
        currentDistance = newDistance;
      }
      leftEncoderCount = newLeftEncoder;
      rightEncoderCount = newRightEncoder;
      if (newMotorStatus.length() > 0 && newMotorStatus.length() < 20) {
        motorStatus = newMotorStatus;

        // Detect movement completion transition (DONE, TIMEOUT, or STALL)
        bool isComplete = (newMotorStatus == "DONE" || newMotorStatus == "TIMEOUT" || newMotorStatus == "STALL");
        bool wasComplete = (lastMotorStatus == "DONE" || lastMotorStatus == "TIMEOUT" || lastMotorStatus == "STALL");

        if (isComplete && !wasComplete) {
          String reason = "done";
          if (newMotorStatus == "TIMEOUT") reason = "timeout";
          else if (newMotorStatus == "STALL") reason = "stall";
          sendMovementComplete(reason);
        }

        lastMotorStatus = newMotorStatus;
      }
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
  if (currentBatteryVoltage > 0) {
    doc["battery_voltage"] = currentBatteryVoltage;
  }
  doc["timestamp"] = millis();

  String json;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);
}

void sendMovementComplete(const String& reason) {
  if (!clientConnected) return;

  StaticJsonDocument<128> doc;
  doc["type"] = "movement_complete";
  doc["success"] = (reason == "done");
  doc["reason"] = reason;
  doc["timestamp"] = millis();

  String json;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);

  Serial.print("Event: movement_complete (reason: ");
  Serial.print(reason);
  Serial.println(")");
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

void sendAck(uint8_t num, bool success, int commandId) {
  StaticJsonDocument<128> doc;
  doc["type"] = "command_ack";
  doc["success"] = success;
  doc["timestamp"] = millis();
  if (commandId >= 0) {
    doc["id"] = commandId;
  }

  String json;
  serializeJson(doc, json);
  webSocket.sendTXT(num, json);
}

// memory health monitoring
void checkMemoryHealth() {
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t minHeap = ESP.getMinFreeHeap();

  if (freeHeap < minFreeHeap) {
    minFreeHeap = freeHeap;
  }

  Serial.printf("[MEM] Free: %u, Min: %u, Lowest: %u\n", freeHeap, minHeap, minFreeHeap);

  // restart if memory critically low (prevents crashes)
  if (freeHeap < MIN_FREE_HEAP) {
    Serial.println("[MEM] CRITICAL: Restarting due to low memory!");
    delay(100);
    ESP.restart();
  }
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

// send application-level ping to client
void sendServerPing() {
  if (!clientConnected || connectedClientNum == 255) return;

  // check if previous ping was answered
  if (lastPongReceived < lastServerPing) {
    missedPongs++;
    Serial.printf("[WS] Ping not answered (missed: %d)\n", missedPongs);
  }

  // send ping at protocol level (more reliable than JSON ping)
  // the library's enableHeartbeat handles this, but we track it too
  StaticJsonDocument<64> doc;
  doc["type"] = "ping";
  doc["ts"] = millis();

  String json;
  serializeJson(doc, json);
  webSocket.sendTXT(connectedClientNum, json);
}

// cleanly disconnect client with reason
void disconnectClient(const char* reason) {
  if (connectedClientNum != 255) {
    Serial.printf("[WS] Disconnecting client: %s\n", reason);
    webSocket.disconnect(connectedClientNum);
  }
  clientConnected = false;
  connectedClientNum = 255;
  missedPongs = 0;
  sendToArduino("M 0 0"); // safety stop
}