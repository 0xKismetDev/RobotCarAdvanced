# Architecture

**Analysis Date:** 2026-02-05

## Pattern Overview

**Overall:** Distributed dual-microcontroller system with hierarchical control layers.

The system implements a **master-slave I2C architecture** with WebSocket clients controlling an ESP32 master, which relays commands to an Arduino slave via I2C. Multiple independent client interfaces (Flutter mobile app, Blockly web UI) can connect to the same ESP32 master through separate WebSocket servers.

**Key Characteristics:**
- Dual-microcontroller: ESP32 (WiFi/WebSocket) + Arduino UNO (motor control)
- I2C command-response protocol with CSV sensor data
- Multiple independent control interfaces (mobile, web block-based)
- Encoder-based precise movement with blocking control loops
- JSON over WebSocket for client-server communication
- Pin Change Interrupt (PCINT) driven encoder counting on Arduino

## Layers

**Client Layer (Presentation):**
- Purpose: User interaction and control interface
- Location: `MobileControlApp/lib/screens/` (Flutter), `BlockBasedLearning/WebInterface/` (Blockly)
- Contains: UI components, joystick controls, block definitions
- Depends on: WebSocket protocol, command protocols
- Used by: End users and programmers

**WebSocket Transport Layer:**
- Purpose: Real-time bidirectional communication between clients and ESP32
- Location: `MobileControlApp/lib/services/websocket_service.dart` (Flutter client), `ESP32_Master/ESP32_Master.ino` line 16-49 (ESP32 server), `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` line 17-101 (Blockly server)
- Contains: WebSocket server setup, JSON message handling, heartbeat/ping-pong protocols
- Depends on: WebSocketsServer library (Arduino), web_socket_channel (Flutter)
- Used by: Client applications, I2C handler

**Master Control Layer (ESP32):**
- Purpose: WiFi AP/Station management, client connection handling, command routing
- Location: `ESP32_Master/ESP32_Master.ino` or `BlockBasedLearning/ESP32_Code/ESP32_Code.ino`
- Contains: WiFi connectivity, WebSocket event handlers, JSON command parsing, sensor data formatting
- Depends on: Arduino Wire library (I2C), WebSocketsServer, ArduinoJson
- Used by: WebSocket clients (indirectly via transport layer)

**I2C Protocol Layer:**
- Purpose: Serialize commands from ESP32 to Arduino, parse sensor responses
- Location: `ESP32_Master/ESP32_Master.ino` lines 186-212 (ESP32 send), `Arduino_Slave/Arduino_Slave.ino` lines 191-229 (Arduino I/O)
- Contains: String-based command formatting, CSV response parsing
- Depends on: Arduino Wire library
- Used by: Master control layer, Slave control layer

**Slave Control Layer (Arduino):**
- Purpose: Motor control, sensor reading, movement execution
- Location: `Arduino_Slave/Arduino_Slave.ino` lines 231-639
- Contains: Motor PWM control, encoder ISR, ultrasonic reading, servo control
- Depends on: Servo library, Wire library, timer interrupts
- Used by: I2C protocol layer (receives commands)

**Hardware Abstraction (Arduino Pins):**
- Purpose: Direct GPIO control for motors, sensors, encoders
- Location: `Arduino_Slave/Arduino_Slave.ino` lines 8-30 (pin definitions)
- Contains: Motor driver pins (L298N), encoder pins (A0/A1 with PCINT), ultrasonic pins, servo pin
- Depends on: Arduino hardware
- Used by: Slave control layer

## Data Flow

**Command Flow (Client → Arduino):**

1. User interacts with client UI (joystick, button, Blockly block)
2. Client creates JSON command: `{"type":"command","action":"differential","leftSpeed":150,"rightSpeed":150}`
3. WebSocketService sends over WebSocket (port 81)
4. ESP32 WebSocket handler receives, parses JSON with ArduinoJson
5. Command handler constructs I2C string: `M 150 150`
6. `sendToArduino()` sends via Wire.write() to address 0x08
7. Arduino onReceive() ISR captures command bytes into cmdBuffer
8. Main loop processes command in `processCommand()` function
9. Command executes (e.g., `setMotors()` for motor control, `moveDistance()` for encoder-based movement)

**Sensor Flow (Arduino → Client):**

1. Arduino maintains current state: distance, encoder counts, motor status
2. ESP32 periodically calls `Wire.requestFrom(0x08, 32)` every 250ms
3. Arduino onRequest() ISR returns CSV: `distance,leftEncoder,rightEncoder,status\n`
4. ESP32 parses response and creates JSON: `{"type":"sensor_data","distance":15,"left_encoder":42,...}`
5. WebSocket broadcasts JSON to all connected clients
6. Flutter/Blockly client receives via stream listener, updates UI

**State Management:**

**Arduino Slave State:**
- Encoder counts (volatile: leftEncoderCount, rightEncoderCount)
- Motor speeds (leftSpeed, rightSpeed)
- Motor status (STOPPED, FORWARD, BACKWARD, LEFT, RIGHT)
- Servo angle (servoAngle)
- Distance reading (distance)
- Command buffer (cmdBuffer, newCmd flag)

**ESP32 Master State:**
- Connected client tracking (clientConnected, connectedClientNum)
- Arduino connection status (arduinoConnected)
- Current sensor readings (distance, servo, encoders, motorStatus)
- WiFi mode (AP vs Station)
- Camera connection status (ESP32_Master only)

**Flutter Client State:**
- Connection status (isConnected, connectionStatus)
- Motor speeds (currentLeftSpeed, currentRightSpeed)
- Sensor data (distance, servo position, battery)
- Settings (IP address, port, auto-connect preference)

## Key Abstractions

**I2C Command Protocol:**
- Purpose: Text-based command format for sending motor and sensor commands
- Examples: `M 150 150` (motor), `D 200 F 150` (distance move), `R 90 L 150` (rotate), `S 90` (servo)
- Pattern: `<COMMAND_CHAR> <PARAM1> <PARAM2> ... <PARAMN>`

**JSON Message Protocol:**
- Purpose: Structured WebSocket messages between client and ESP32
- Pattern: All messages include `"type"` field for routing
- Command: `{"type":"command","action":"<action>","<param>":"<value>"}`
- Sensor: `{"type":"sensor_data","distance":<int>,...}`
- Acknowledgment: `{"type":"command_ack","success":<bool>,"id":<cmdId>}`

**Blockly Block Definitions:**
- Purpose: Visual programming blocks for block-based interface
- Location: `BlockBasedLearning/WebInterface/blockly-robot.js`
- Pattern: Each block type (robot_move_forward, robot_turn_degrees, etc.) maps to JSON command execution

**Encoder-Based Movement System:**
- Purpose: Precise movement using wheel encoder feedback
- Implementation: 21 pulses per rotation, ~9.72mm per pulse (65mm wheel diameter)
- Blocking control: moveDistance() and rotateDegrees() run blocking loops until encoder targets reached
- Calibration factors: LEFT_MOTOR_FACTOR, RIGHT_MOTOR_FACTOR, TURN_CALIBRATION_FACTOR

## Entry Points

**Mobile App (Flutter):**
- Location: `MobileControlApp/lib/main.dart`
- Triggers: User launches app
- Responsibilities: Initialize providers (WebSocketService, SettingsService), show SplashScreen → MainScreen

**Main Control Screen:**
- Location: `MobileControlApp/lib/screens/main_screen.dart`
- Triggers: SplashScreen navigation after 2-second animation
- Responsibilities: Joystick input handling, motor command sending, sensor display, connection management

**Blockly Web Interface:**
- Location: `BlockBasedLearning/WebInterface/index.html`
- Triggers: User opens in browser (served by `server.py` on localhost:8081)
- Responsibilities: Blockly workspace, block execution, WebSocket connection, console output

**ESP32 Master Firmware:**
- Location: `ESP32_Master/ESP32_Master.ino` or `BlockBasedLearning/ESP32_Code/ESP32_Code.ino`
- Triggers: Power on ESP32
- Responsibilities: WiFi setup (AP or Station mode), WebSocket server, client handling, I2C communication with Arduino

**Arduino Slave Firmware:**
- Location: `Arduino_Slave/Arduino_Slave.ino`
- Triggers: Power on Arduino
- Responsibilities: I2C slave initialization, encoder interrupt setup, command processing, motor/sensor control

## Error Handling

**Strategy:** Defensive programming with timeouts and fallback mechanisms.

**Patterns:**

**Timeout-Based Movement Control:**
- `moveDistance()` and `rotateDegrees()` run blocking loops with dynamic timeout calculation
- Formula: `timeout = BASE_MOVEMENT_TIMEOUT + (pulses * TIMEOUT_PER_PULSE)`
- Max timeout: 15 seconds
- Prevents infinite loops if encoder fails or motor stalls
- Location: `Arduino_Slave/Arduino_Slave.ino` lines 493-639

**I2C Error Detection:**
- `Wire.endTransmission()` returns error code
- Arduino checks for write errors in `sendToArduino()` function
- Sets `arduinoConnected = false` if I2C transmission fails
- Blockly code adds timeout protection for I2C reads (50ms max)
- Location: `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` lines 452-470, 493-542

**WebSocket Reliability:**
- Blockly implementation includes aggressive connection management
- Client timeout: 10 seconds inactivity
- Server-side pings every 2 seconds with missed-pong tracking
- Memory health check with ESP restart if heap < 15KB
- Location: `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` lines 37-50, 128-170

**Fallback WiFi Mode:**
- ESP32_Master attempts to connect to ESP32-CAM first
- Falls back to own AP (RobotCar-AP) if camera not found
- Location: `ESP32_Master/ESP32_Master.ino` lines 86-116

**Command Validation:**
- Speed values constrained to -255 to 255 range
- Servo angle constrained to 0-180 degrees
- Distance values validated before sensor reading acceptance
- Location: `Arduino_Slave/Arduino_Slave.ino` lines 358-412

## Cross-Cutting Concerns

**Logging:**
- Approach: Serial.println() debug output to USB connection
- Used for: Command tracing, encoder diagnostics, I2C error reporting, WiFi events
- Location: Throughout .ino files, all major functions log their actions

**Validation:**
- Approach: Range constraining (constrain() function) before motor control
- Patterns: Speed values 0-255, angles 0-180, distances 0-1000mm
- Location: `Arduino_Slave/Arduino_Slave.ino` lines 358-412, `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` lines 313-389

**Authentication:**
- Approach: None - assumes local WiFi only
- AP mode uses password (12345678) for WiFi access
- No credentials required for WebSocket connection
- Location: `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` line 13-14

**Motor Safety:**
- Approach: Automatic stop on client disconnect
- Implementation: WebSocket DISCONNECTED event sends `M 0 0` to Arduino
- Ensures motors stop if communication lost
- Location: `ESP32_Master/ESP32_Master.ino` line 123, `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` lines 229-237

**Servo Management:**
- Approach: Auto-detach after 2 seconds of inactivity to prevent twitching
- Reattach on demand before writing new position
- Prevents power drain and signal noise
- Location: `Arduino_Slave/Arduino_Slave.ino` lines 185-188, 327-347

---

*Architecture analysis: 2026-02-05*
