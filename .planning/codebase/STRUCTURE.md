# Codebase Structure

**Analysis Date:** 2026-02-05

## Directory Layout

```
RobotCarAdvanced/
├── Arduino_Slave/                      # Arduino UNO firmware (motor controller)
│   └── Arduino_Slave.ino
├── ESP32_Master/                       # ESP32 firmware (WiFi master, mobile app support)
│   └── ESP32_Master.ino
├── ESP32_Camera/                       # ESP32-CAM firmware (video streaming)
│   └── ESP32_Camera.ino
├── BlockBasedLearning/                 # Block-based programming interface
│   ├── ESP32_Code/                     # Standalone Blockly ESP32 firmware
│   │   └── ESP32_Code.ino
│   └── WebInterface/                   # Blockly web UI (runs locally, connects to ESP32)
│       ├── index.html                  # Main HTML page
│       ├── app.js                      # Blockly setup, WebSocket client, block execution
│       ├── blockly-robot.js            # Custom robot block definitions
│       ├── server.py                   # Local Python server (serves interface on :8081)
│       └── lib/
│           └── blockly.min.js          # Blockly library (minified)
├── MobileControlApp/                   # Flutter mobile app
│   ├── lib/
│   │   ├── main.dart                   # Entry point, provider setup
│   │   ├── models/
│   │   │   └── sensor_data.dart        # Data class for sensor readings
│   │   ├── screens/
│   │   │   ├── main_screen.dart        # Main control UI with joystick
│   │   │   └── settings_screen.dart    # Connection and calibration settings
│   │   ├── services/
│   │   │   ├── websocket_service.dart  # WebSocket client, command/sensor handling
│   │   │   └── settings_service.dart   # Persistent settings storage
│   │   ├── widgets/
│   │   │   ├── custom_joystick.dart    # Analog joystick widget
│   │   │   └── camera_view.dart        # Camera stream display
│   │   └── utils/
│   │       └── theme.dart              # Theme and styling
│   ├── pubspec.yaml                    # Flutter dependencies
│   ├── android/                        # Android native code
│   ├── ios/                            # iOS native code
│   ├── linux/                          # Linux platform support
│   ├── macos/                          # macOS platform support
│   ├── windows/                        # Windows platform support
│   └── test/
│       └── widget_test.dart
├── README.md                           # Project documentation
├── CLAUDE.md                           # Development guidelines (this project)
└── .planning/
    └── codebase/                       # GSD analysis documents
        ├── ARCHITECTURE.md
        └── STRUCTURE.md
```

## Directory Purposes

**Arduino_Slave:**
- Purpose: Firmware for Arduino UNO microcontroller
- Contains: Motor control, encoder interrupt handling, sensor reading, I2C slave implementation
- Key files: `Arduino_Slave.ino` (single monolithic file)

**ESP32_Master:**
- Purpose: WiFi-enabled control hub for mobile app
- Contains: WebSocket server, WiFi AP/Station mode, I2C master to Arduino, optional camera connection
- Key files: `ESP32_Master.ino` (single monolithic file)

**ESP32_Camera:**
- Purpose: Video streaming from ESP32-CAM module
- Contains: Camera initialization, MJPEG streaming server, flash LED control
- Key files: `ESP32_Camera.ino` (single monolithic file)

**BlockBasedLearning:**
- Purpose: Visual block-based programming environment
- Contains: Blockly interface, custom robot blocks, WebSocket client, local server
- Subdirectories: `ESP32_Code/` (firmware), `WebInterface/` (web UI)

**BlockBasedLearning/ESP32_Code:**
- Purpose: Standalone Blockly firmware for ESP32
- Contains: WebSocket server with enhanced reliability, I2C master, sensor polling, memory management
- Key files: `ESP32_Code.ino` (single monolithic file, ~659 lines)

**BlockBasedLearning/WebInterface:**
- Purpose: Browser-based Blockly editor
- Contains: HTML UI, block definitions, WebSocket communication, block code generators
- Key files:
  - `index.html` - Page structure and UI layout
  - `app.js` - WebSocket management, command execution, console logging
  - `blockly-robot.js` - Custom block definitions and code generators
  - `server.py` - Python HTTP server for local development

**MobileControlApp:**
- Purpose: Cross-platform Flutter mobile controller
- Contains: Joystick interface, sensor display, connection management, settings storage
- Key subdirectories:
  - `lib/models/` - Data classes
  - `lib/screens/` - Full-screen pages
  - `lib/services/` - Business logic (WebSocket, settings)
  - `lib/widgets/` - Reusable UI components
  - `lib/utils/` - Helpers and theming

## Key File Locations

**Entry Points:**
- `MobileControlApp/lib/main.dart` - Flutter app initialization, provider setup
- `BlockBasedLearning/WebInterface/index.html` - Blockly interface, served by server.py
- `Arduino_Slave/Arduino_Slave.ino` - Arduino board initialization (upload via Arduino IDE)
- `ESP32_Master/ESP32_Master.ino` - ESP32 master board initialization
- `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` - Alternative ESP32 for Blockly mode

**Configuration:**
- `MobileControlApp/pubspec.yaml` - Flutter dependencies (Provider, web_socket_channel, vibration, etc.)
- `Arduino_Slave/Arduino_Slave.ino` lines 8-40 - Pin definitions, calibration constants
- `ESP32_Master/ESP32_Master.ino` lines 9-31 - WiFi SSIDs, I2C pins, camera IP
- `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` lines 12-50 - WiFi config, timing constants

**Core Logic:**
- `Arduino_Slave/Arduino_Slave.ino` lines 231-639 - Command processing, motor control, encoder movement
- `ESP32_Master/ESP32_Master.ino` lines 141-184 - JSON command handling, I2C routing
- `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` lines 292-450 - Enhanced command handling with reliability
- `MobileControlApp/lib/services/websocket_service.dart` - WebSocket client and message handling
- `BlockBasedLearning/WebInterface/app.js` - Blockly execution engine and WebSocket manager

**Testing:**
- `MobileControlApp/test/widget_test.dart` - Flutter widget tests

## Naming Conventions

**Files:**
- Firmware files: `{Platform}_{Role}.ino` (Arduino_Slave.ino, ESP32_Master.ino)
- Dart files: `snake_case.dart` (sensor_data.dart, websocket_service.dart)
- JavaScript files: `kebab-case.js` (blockly-robot.js, app.js)
- Python files: `snake_case.py` (server.py)

**Directories:**
- Arduino: Single file per microcontroller, no nested structure
- Flutter: Layered by feature (screens, services, widgets, models, utils)
- Blockly: WebInterface for UI, ESP32_Code for firmware

**Functions/Methods:**
- Arduino C++: camelCase private functions prefixed with _ (e.g., setupEncoderInterrupts, _handleMessage)
- Dart: camelCase methods (e.g., sendDifferentialCommand, handleJoystickMove)
- JavaScript: camelCase functions (e.g., connectWebSocket, handleMessage)

**Variables:**
- Arduino: camelCase for simple variables, UPPERCASE for #define constants
- Dart: camelCase with _ prefix for private (e.g., _wsService, _currentLeftSpeed)
- JavaScript: camelCase for most, UPPERCASE for constants (e.g., MAX_RECONNECT_ATTEMPTS)

## Where to Add New Code

**New Motor Command (e.g., spiral movement):**
1. Add command name to I2C protocol documentation in CLAUDE.md
2. Implement handler in `Arduino_Slave/Arduino_Slave.ino` - add case in processCommand() around line 238-356
3. Add control method in `Arduino_Slave/Arduino_Slave.ino` - new function after rotateDegrees() (line 639)
4. Add Flutter command sender in `MobileControlApp/lib/services/websocket_service.dart` - new method after existing commands
5. Add Blockly block in `BlockBasedLearning/WebInterface/blockly-robot.js` - new Blockly.Blocks entry
6. Add block code generator in `BlockBasedLearning/WebInterface/blockly-robot.js` - RobotGenerators entry
7. Add app.js handler for new action type in `BlockBasedLearning/WebInterface/app.js`

**New Sensor (e.g., motion detector):**
1. Add pin definition in `Arduino_Slave/Arduino_Slave.ino` line 8-30
2. Add reading code in setup() and loop()
3. Add to sensor response CSV in onRequest() around line 216-229
4. Update response parsing in ESP32 `sendSensorData()` function (line 214-257 in Master, 493-542 in Blockly)
5. Update Flutter SensorData model in `MobileControlApp/lib/models/sensor_data.dart`
6. Update Flutter UI in `MobileControlApp/lib/screens/main_screen.dart` to display new sensor

**New Client Interface:**
1. Copy WebSocket client pattern from `MobileControlApp/lib/services/websocket_service.dart`
2. Use message protocol (JSON type-action pattern)
3. Target WebSocket at IP:81
4. Handle sensor_data, command_ack, error, pong message types

**New Control Mode (e.g., autonomous obstacle avoidance):**
1. Add Blockly block in `BlockBasedLearning/WebInterface/blockly-robot.js` (or skip if not block-based)
2. Implement execution in ESP32 firmware - modify command handler to run sensor-motor loop
3. Add safety timeouts to prevent infinite loops
4. Test with telemetry output (Serial.println in Arduino, console.log in Blockly)

**Reliability Enhancement:**
1. For connection issues: See `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` lines 37-50 for timeout patterns
2. For encoder issues: Adjust calibration factors in `Arduino_Slave/Arduino_Slave.ino` lines 34-39
3. For motor stuttering: Modify movement control in `Arduino_Slave/Arduino_Slave.ino` lines 493-562

## Special Directories

**.planning/codebase/:**
- Purpose: GSD (Generalized Software Development) analysis documents
- Generated: Yes (by /gsd:map-codebase commands)
- Committed: Yes - documents consumed by /gsd:plan-phase and /gsd:execute-phase commands

**MobileControlApp/android/, ios/, etc.:**
- Purpose: Platform-specific Flutter build artifacts
- Generated: Yes (by flutter command)
- Committed: Yes - includes configuration and native code

**MobileControlApp/build/, .dart_tool/:
- Purpose: Build artifacts and tool caches
- Generated: Yes (by flutter)
- Committed: No (in .gitignore)

**node_modules/, .flutter_plugins/:
- Purpose: Dependency caches
- Generated: Yes
- Committed: No

## Build and Upload Procedures

**Arduino Firmware:**
1. Open `Arduino_Slave.ino` in Arduino IDE
2. Select board: "Arduino Uno"
3. Select port: USB connection
4. Click Upload

**ESP32 Master Firmware:**
1. Open `ESP32_Master.ino` in Arduino IDE
2. Select board: "ESP32 Dev Module" or similar
3. Install ESP32 board support if needed
4. Install required libraries: WebSocketsServer, ArduinoJson
5. Click Upload

**ESP32 Blockly Firmware:**
1. Open `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` in Arduino IDE
2. Same ESP32 setup as Master
3. Click Upload

**Flutter Mobile App:**
```bash
cd MobileControlApp
flutter pub get
flutter run
# or: flutter build apk (Android), flutter build ios (iOS)
```

**Blockly Web Interface:**
```bash
cd BlockBasedLearning/WebInterface
python3 server.py
# Opens browser automatically on http://localhost:8081
```

---

*Structure analysis: 2026-02-05*
