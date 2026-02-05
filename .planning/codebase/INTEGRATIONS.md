# External Integrations

**Analysis Date:** 2026-02-05

## APIs & External Services

**None Configured:**
This is a self-contained embedded system with no external API integrations. All communication occurs over local WiFi or wired I2C.

## Data Storage

**Databases:**
- Not used. This is an embedded robotics system with no persistent data backend.

**File Storage:**
- Local filesystem only - Flutter app uses `shared_preferences` plugin to store connection settings (IP address, port) in device storage
  - Location: iOS Keychain, Android SharedPreferences (platform-dependent)
  - No cloud storage integration

**Caching:**
- None configured. Real-time sensor data is streamed continuously (250-500ms intervals) without caching.

## Authentication & Identity

**Auth Provider:**
- None. WiFi networks use basic password protection (hardcoded in firmware).
- No user accounts, login system, or identity provider.

**Security Model:**
- WiFi passwords hardcoded in firmware: `12345678` (development/test credentials)
- No TLS/SSL on WebSocket connections (unencrypted)
- No API key authentication
- Network is private (local WiFi AP only)

## Monitoring & Observability

**Error Tracking:**
- None. No external error reporting service (Sentry, Rollbar, etc.)

**Logs:**
- Serial Monitor output (Arduino IDE) - accessible via USB
  - Arduino prints: encoder counts, motor status, sensor readings, I2C communication errors
  - ESP32 prints: WiFi connection status, WebSocket client connections, Arduino I2C heartbeat
- Browser Console - JavaScript errors from Blockly interface and WebSocket client
- Python server logs to stderr for HTTP requests (CustomHTTPRequestHandler)

**Health Monitoring:**
- Arduino: No external monitoring (code includes 5-second movement timeout as safety feature)
- ESP32: Watchdog timer (esp_task_wdt.h) configured to restart on deadlock. Memory monitoring logs to Serial when < 15KB free heap.

## CI/CD & Deployment

**Hosting:**
- Embedded Device - ESP32 runs WebSocket server at `192.168.4.1:81`
- No cloud hosting, load balancers, or distributed infrastructure

**CI Pipeline:**
- None detected. No GitHub Actions, GitLab CI, Jenkins, or automated testing.
- Manual flashing via Arduino IDE or command-line tools

**Firmware Updates:**
- Over-the-wire update NOT implemented
- Manual update process: recompile firmware, erase ESP32/Arduino flash, re-upload via USB

## Environment Configuration

**Required env vars:**
- None. All configuration is hardcoded in firmware source files.
- WiFi credentials, IP addresses, I2C addresses, GPIO pins all defined as C++ constants

**Secrets location:**
- Hardcoded in source:
  - `const char* FALLBACK_SSID = "RobotCar-AP"` (`ESP32_Master/ESP32_Master.ino`)
  - `const char* AP_SSID = "RobotCar-Blockly"` (`BlockBasedLearning/ESP32_Code/ESP32_Code.ino`)
  - `const char* CAM_PASSWORD = "12345678"` (hardcoded across all .ino files)
- Flutter app stores user-entered IP address in `SharedPreferences` (on device only)

## Communication Protocols

**WebSocket (Client ↔ ESP32):**
- Protocol: WebSocket (ws://, not wss://)
- Server: Runs on ESP32 at `192.168.4.1:81`
- Clients: Flutter app, JavaScript Blockly interface
- Message format: JSON
- Update frequency: Client sends commands on demand; ESP32 broadcasts sensor data every 250-500ms

**Message Types:**
```
// Command (Client → ESP32)
{
  "type": "command",
  "action": "moveDistance|rotateDegrees|differential|servo|read_distance|scan|resetEncoders|getEncoders",
  "distance": number,        // for moveDistance
  "direction": "forward|backward|left|right",
  "degrees": number,         // for rotateDegrees
  "leftSpeed": number,       // for differential
  "rightSpeed": number,      // for differential
  "angle": number            // for servo
}

// Ping (Client → ESP32)
{ "type": "ping" }

// Sensor Data (ESP32 → Client, broadcast)
{
  "type": "sensorData",
  "distance": number,        // cm from ultrasonic
  "leftEncoder": number,     // pulse count
  "rightEncoder": number,    // pulse count
  "servo": number,           // current angle
  "motorStatus": "STOPPED|FORWARD|BACKWARD|LEFT|RIGHT|MOVING",
  "battery": number          // percentage
}

// Response (ESP32 → Client)
{
  "type": "response",
  "success": boolean,
  "message": string
}

// Pong (ESP32 → Client)
{ "type": "pong" }
```

**I2C (ESP32 ↔ Arduino):**
- Protocol: I2C (TWI), 400kHz clock speed
- Master: ESP32 (GPIO21=SDA, GPIO22=SCL)
- Slave: Arduino UNO at address 0x08 (A4=SDA, A5=SCL)
- Message format: String commands followed by binary response
- Update frequency: Commands on-demand; Arduino responds synchronously

**I2C Commands (ESP32 → Arduino):**
```
// Encoder-based movement
"D <distance_mm> <direction> <speed>"
  direction: F (forward), B (backward)
  Example: "D 200 F 150"

"R <degrees> <direction> <speed>"
  direction: L (left), R (right)
  Example: "R 90 L 150"

// Encoder control
"E"                    // Reset encoders to 0
"Q"                    // Query encoder values

// Direct motor control
"M <leftSpeed> <rightSpeed>"
  Speed range: -255 to +255
  Example: "M 150 150"

// Servo and sensors
"S <angle>"           // Set servo (0-180 degrees)
"SCAN"                // Perform 180° servo sweep, return distances
"READ_DISTANCE"       // Single ultrasonic reading

// Legacy commands (backward compatible)
"MOVE <distance_mm> <speed>"
"TURN <degrees> <speed>"
"RESET_ENCODERS"
```

**I2C Response (Arduino → ESP32):**
```
CSV format: "distance,leftEncoder,rightEncoder,status\n"
Example: "25,42,43,MOVING"
- distance: HC-SR04 ultrasonic distance in cm
- leftEncoder: Accumulated pulse count from left encoder
- rightEncoder: Accumulated pulse count from right encoder
- status: Motor state (STOPPED, FORWARD, BACKWARD, LEFT, RIGHT, MOVING)
```

**WiFi Connectivity:**
- Standard 802.11 b/g/n (2.4GHz)
- Three separate WiFi networks offered:
  1. `RobotCar-Blockly` (BlockBasedLearning/ESP32_Code) - for web interface
  2. `RobotCar-AP` (ESP32_Master) - fallback AP if ESP32-CAM not found
  3. `ESP32-CAM-STREAM` (ESP32_Camera) - camera streaming network
- All use password: `12345678`
- IP address: `192.168.4.1` (subnet: 192.168.4.0/24)
- WebSocket endpoint: `ws://192.168.4.1:81`

## Hardware Interfaces

**Encoder Input (Arduino):**
- Pin Change Interrupts on PCINT8 (A0, left) and PCINT9 (A1, right)
- Interrupt-driven counting for wheel pulses
- Resolution: 21 pulses per wheel rotation
- Distance per pulse: ~9.72mm (65mm wheel diameter)

**Motor Output (Arduino):**
- PWM on pins 5 (ENA, left) and 6 (ENB, right)
- Direction control via IN1/IN2 (left) and IN3/IN4 (right) digital pins
- L298N motor driver with 2 DC motors

**Ultrasonic Sensor (Arduino):**
- Trigger: GPIO9, Echo: GPIO10
- Single shot or continuous scanning (servo sweep)
- Range: 2cm-400cm (HC-SR04 specification)

**Servo Motor (Arduino):**
- Pin A2 (PWM)
- Angle range: 0-180 degrees
- Auto-detach after 2 seconds of inactivity (code-implemented power saving)

**Camera Streaming (Optional ESP32-CAM):**
- HTTP server on port 80
- Stream endpoint: `/stream` - MJPEG format
- Operates independently of main robot control (no data integration)

## Webhooks & Callbacks

**Incoming:**
- WebSocket connections (client connects to server)
- No traditional HTTP webhooks

**Outgoing:**
- WebSocket messages (sensor broadcasts)
- No external callbacks or push notifications

## Scaling & Load

**Single Client Design:**
- WebSocket server supports multiple simultaneous connections but only one primary "active" client for command control
- ESP32 firmware limits connected client tracking to single `connectedClientNum` variable
- Sensor data broadcasts to all connected clients but command processing prioritizes single active connection

**Update Rates:**
- Sensor polling: 250ms (I2C reads from Arduino)
- Sensor broadcast: 500ms (WebSocket message to all clients)
- Client heartbeat/ping: 2000ms (ESP32 → client)
- Arduino movement timeout: 5000ms (safety feature)

---

*Integration audit: 2026-02-05*
