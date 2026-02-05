# Coding Conventions

**Analysis Date:** 2026-02-05

## Naming Patterns

### Arduino C++ (`.ino` files)

**Constants:**
- All uppercase with underscores: `I2C_ADDRESS`, `LEFT_ENCODER_PIN`, `WHEEL_DIAMETER`
- Example: `#define ENA 5`, `const float WHEELBASE = 145.0`

**Variables:**
- camelCase for local variables and function parameters: `leftSpeed`, `rightSpeed`, `servoAngle`
- camelCase prefixed with underscore for volatile/ISR variables: `leftEncoderCount`, `rightEncoderCount`
- Example: `volatile long leftEncoderCount = 0`, `int cmdIndex = 0`

**Functions:**
- camelCase with descriptive action verbs: `setupEncoderInterrupts()`, `moveDistance()`, `readDistance()`, `performScan()`
- ISR handler functions follow pattern: `ISR(PCINT1_vect)`
- Wire event handlers: `onReceive()`, `onRequest()`

**Pins and Hardware:**
- Define as constants with descriptive names
- File: `Arduino_Slave/Arduino_Slave.ino` (lines 6-23): Motor pins use pattern `ENA`, `IN1`, `IN2`, `ENB`, `IN3`, `IN4`

### Dart/Flutter (`.dart` files)

**Classes:**
- PascalCase: `SensorData`, `CustomJoystick`, `RobotCarApp`, `SettingsService`, `WebSocketService`
- File: `MobileControlApp/lib/models/sensor_data.dart`

**Methods and Variables:**
- camelCase: `isConnected`, `connectionStatus`, `onConnected`, `_isConnecting`, `_reconnectAttempts`
- Private members prefixed with underscore: `_channel`, `_serverUrl`, `_isConnected`, `_onErrorController`
- Example from `websocket_service.dart` (lines 7-50): StreamController fields use leading underscore

**Constants:**
- camelCase or UPPER_CASE depending on context
- Example: `static const int _maxReconnectAttempts = 5`, `static const Duration _pingInterval = Duration(seconds: 5)`

**Widget/State Classes:**
- Pattern: `MyWidget extends StatelessWidget` or `MyWidgetState extends State<MyWidget>`
- File: `MobileControlApp/lib/main.dart` (lines 46-51): `SplashScreen` with `_SplashScreenState`

### JavaScript (`.js` files)

**Variables:**
- Global scope: `let workspace`, `let ws`, `let reconnectAttempts`
- Local scope: `const wsUrl`, `const data`, `const delay`
- Use `const` for constants, `let` for reassignable variables
- File: `BlockBasedLearning/WebInterface/app.js` (lines 1-23): All module-level variables use `let`

**Functions:**
- camelCase: `initBlockly()`, `connectWebSocket()`, `setupWebSocketHandlers()`, `handleWebSocketMessage()`
- Event handlers: `ws.onopen`, `ws.onmessage`, `ws.onerror`, `ws.onclose`

**Blockly Blocks:**
- Blocks use snake_case with prefix: `robot_move_forward`, `robot_turn_degrees`, `robot_servo`, `robot_get_distance`
- File: `BlockBasedLearning/WebInterface/blockly-robot.js` (lines 5-99)

### Python (`.py` files)

**Functions:**
- snake_case: `open_browser()`, `custom_logging_format()`
- File: `BlockBasedLearning/WebInterface/server.py` (lines 25-27)

**Classes:**
- PascalCase: `CustomHTTPRequestHandler`

**Constants:**
- UPPER_CASE: `PORT = 8081`, `HOST = "localhost"`

## Code Style

### Formatting

**Arduino C++:**
- 2-space indentation observed in most control structures
- Brackets on same line: `if (condition) {`
- Comments use `//` for single-line, `/* */` rarely used
- File: `Arduino_Slave/Arduino_Slave.ino` shows consistent 2-space indentation

**Dart:**
- 2-space indentation (standard Dart)
- Brackets on same line: `if (condition) {`
- Type annotations explicit: `final int distance`, `String motorStatus`
- File: `MobileControlApp/lib/models/sensor_data.dart` (lines 1-37)

**JavaScript:**
- 4-space indentation
- Brackets on same line: `if (condition) {`
- Template literals used: `` const wsUrl = `ws://${ip}:81`; ``
- File: `BlockBasedLearning/WebInterface/app.js` uses 4-space indentation

**Python:**
- 4-space indentation (PEP 8)
- File: `BlockBasedLearning/WebInterface/server.py` (lines 29-60)

### Linting

**Dart:**
- Uses `flutter_lints` configured in `analysis_options.yaml`
- File: `MobileControlApp/analysis_options.yaml` (line 10): `include: package:flutter_lints/flutter.yaml`
- Run with: `flutter analyze`
- Linting rules enforced: null safety, avoid unused imports, consistent naming

**Arduino C++:**
- No formal linter configured; code follows Arduino IDE conventions
- Manual style enforcement through code review (CLAUDE.md guidelines)

**JavaScript:**
- No formal linter configured (ESLint absent)
- Follows basic best practices: use `const`/`let` not `var`, template literals preferred

**Python:**
- No formal linter configured; follows basic conventions (no flake8/pylint in evidence)

## Import Organization

### Arduino C++

**Order:**
1. System libraries: `#include <Wire.h>`, `#include <Servo.h>`
2. Firmware-specific headers (rarely used)

**Pattern:**
```cpp
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
```
File: `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` (lines 5-8)

### Dart

**Order:**
1. dart: imports
2. package: imports from pubspec.yaml
3. Relative imports (lib/...)

**Pattern:**
```dart
import 'dart:async';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../models/sensor_data.dart';
```
File: `MobileControlApp/lib/services/websocket_service.dart` (lines 1-5)

### JavaScript

**Pattern:**
- All imports at top of file (none observed in this project)
- Global variables declared before use
- Blockly library assumed loaded via HTML `<script>` tag

## Error Handling

### Arduino C++

**Pattern:**
- No try-catch (C++ not used in Arduino typically)
- Validation with `if` statements and early returns
- Status strings track state: `motorStatus = "STOPPED"`, `motorStatus = "FORWARD"`
- Timeout protection with millis() comparisons
- File: `Arduino_Slave/Arduino_Slave.ino` (lines 536-555): Movement timeout handling

**Example:**
```cpp
if (millis() - startTime > timeout) {
  stopMotors();
  Serial.print("movement timeout after ");
  Serial.print(timeout);
  Serial.println("ms!");
  break;
}
```

### Dart

**Pattern:**
- Try-catch blocks with explicit error handling
- Stream-based error propagation via StreamController
- ChangeNotifier updates for UI state
- File: `MobileControlApp/lib/services/websocket_service.dart` (lines 58-101)

**Example:**
```dart
try {
  _channel = WebSocketChannel.connect(Uri.parse(_serverUrl!));
  _channel!.stream.listen(
    (message) { _handleMessage(message.toString()); },
    onError: (error) {
      debugPrint('WebSocket error: $error');
      _onErrorController.add('Connection error: ${error.toString()}');
    }
  );
} catch (e) {
  _onErrorController.add('Failed to connect: ${e.toString()}');
}
```

### JavaScript

**Pattern:**
- Try-catch for JSON parsing and WebSocket operations
- Error messages logged to console and UI
- Fallback behavior for connection failures
- Automatic reconnection with exponential backoff
- File: `BlockBasedLearning/WebInterface/app.js` (lines 147-155)

**Example:**
```javascript
try {
  ws = new WebSocket(wsUrl);
  setupWebSocketHandlers();
} catch (error) {
  addToConsole(`Verbindungsfehler: ${error.message}`, 'error');
  if (autoReconnect) {
    scheduleReconnect();
  }
}
```

## Logging

### Arduino C++

**Framework:** Serial.print() / Serial.println()

**Patterns:**
- Initialization messages: "=== arduino robot car - i2c slave with encoders ==="
- Command echo: `Serial.print("cmd: "); Serial.println(cmd);`
- Status updates: `Serial.print("motors: L="); Serial.print(leftSpeed);`
- File: `Arduino_Slave/Arduino_Slave.ino` (lines 80-145)

**Approach:**
- Frequent diagnostic logging in ISRs and main loop
- Status via Serial at 115200 baud
- No conditional log levels (all enabled)

### Dart

**Framework:** debugPrint() and Stream-based logging

**Patterns:**
- debugPrint() for debug output: `debugPrint('WebSocket error: $error')`
- Stream errors: `_onErrorController.add('Connection error: ...')`
- UI state reflects connection status via `connectionStatus` string
- File: `MobileControlApp/lib/services/websocket_service.dart` (lines 72-74)

### JavaScript

**Framework:** console.error() and in-app logging

**Patterns:**
- Error logging: `console.error('Error closing WebSocket:', e)`
- UI console output: `addToConsole('Schließe bestehende Verbindung...', 'info')`
- Events logged: "✅ Mit Roboter verbunden!" (success), red text for errors
- File: `BlockBasedLearning/WebInterface/app.js` (lines 127, 160, 178)

### Python

**Framework:** sys.stderr for logging

**Patterns:**
- Custom logging in request handler: `sys.stderr.write("%s - %s\n" % (self.address_string(), format % args))`
- File: `BlockBasedLearning/WebInterface/server.py` (line 23)

## Comments

### When to Comment

**Arduino C++:**
- File headers explaining purpose
- Complex calculations (e.g., encoder calibration factors with justification)
- ISR behavior and timing-sensitive sections
- Deprecated legacy commands marked with "Legacy command support for backward compatibility"
- File: `Arduino_Slave/Arduino_Slave.ino` (line 297): `// Legacy command support for backward compatibility`

**Dart:**
- Class documentation above service classes
- Complex stream logic
- Widget state management comments
- File: `MobileControlApp/lib/services/websocket_service.dart` lines 42-50: Stream declarations documented

**JavaScript:**
- Blockly block definitions with tooltips
- Complex reconnection logic
- Error scenarios
- File: `BlockBasedLearning/WebInterface/blockly-robot.js` (lines 19-20): `this.setTooltip("Fährt exakt die angegebene Distanz vorwärts");`

### JSDoc/TSDoc

**Not used in this project** - No formal documentation generation observed.

## Function Design

### Size

**Guideline:** Single responsibility, typically 20-50 lines

**Arduino C++:**
- `moveDistance()` and `rotateDegrees()` are complex (50-80 lines) due to blocking movement loops
- File: `Arduino_Slave/Arduino_Slave.ino` (lines 493-562): `moveDistance()` function

**Dart:**
- Service methods typically 10-30 lines
- Widget build methods vary by complexity
- File: `MobileControlApp/lib/models/sensor_data.dart` (lines 20-30): `fromJson()` factory simple

**JavaScript:**
- Event handlers 10-40 lines
- Blockly block definitions 10-20 lines
- File: `BlockBasedLearning/WebInterface/blockly-robot.js` (lines 5-21): Block definition

### Parameters

**Arduino C++:**
- Typically 2-4 parameters
- Example: `moveDistance(int distanceMM, char direction, int speed)` (3 params)
- File: `Arduino_Slave/Arduino_Slave.ino` (line 493)

**Dart:**
- Named parameters with required/optional: `connect(String ipAddress, int port)`
- Constructor parameter pattern: `CustomJoystick({required this.onMove, required this.onRelease, this.size = 240})`
- File: `MobileControlApp/lib/widgets/custom_joystick.dart` (lines 10-16)

**JavaScript:**
- Positional parameters with defaults: `connectWebSocket(autoReconnect = false)`
- File: `BlockBasedLearning/WebInterface/app.js` (line 107)

### Return Values

**Arduino C++:**
- void for state-modifying functions
- int/long/float for computed values
- File: `Arduino_Slave/Arduino_Slave.ino`: `void moveDistance()`, `void setMotors()`

**Dart:**
- Explicit return types always: `Future<void> connect()`, `Stream<String> get onError`
- File: `MobileControlApp/lib/services/websocket_service.dart` (line 52): `void connect(String ipAddress, int port)`

**JavaScript:**
- return statements for computed values
- Async/await for asynchronous operations (Blockly code generation)
- File: `BlockBasedLearning/WebInterface/blockly-robot.js` (lines 281-290): Generated code with `return` and `await`

## Module Design

### Exports

**Dart:**
- Classes and public functions exported by default (no `library` directives observed)
- Private members use leading underscore `_`
- File: `MobileControlApp/lib/main.dart` exports `RobotCarApp`

**JavaScript:**
- No module.exports observed; all global scope or Blockly namespace
- Functions are globally available to HTML event handlers
- File: `BlockBasedLearning/WebInterface/app.js` all functions global

**Python:**
- Classes and functions exported implicitly
- Entry point: `if __name__ == '__main__':`
- File: `BlockBasedLearning/WebInterface/server.py` (line 29)

### Barrel Files

**Not used in this project** - No index.ts or __init__.py re-exports observed.

## Asynchronous Patterns

### Arduino C++

**Pattern:** Blocking loops with timeouts (no async/await)
- Movement functions block until encoder targets reached
- Timeout protection prevents hangs
- Interrupts continue to work during blocking loops
- File: `Arduino_Slave/Arduino_Slave.ino` (lines 528-555): Blocking `moveDistance()` loop

**Key Example:**
```cpp
while (true) {
  cli();
  long leftCount = leftEncoderCount;
  long rightCount = rightEncoderCount;
  sei();

  if (avgPulses >= (pulsesNeeded - POSITION_TOLERANCE)) {
    stopMotors();
    break;
  }

  if (millis() - startTime > timeout) {
    stopMotors();
    break;
  }

  delayMicroseconds(500);  // allow I2C interrupts
}
```

### Dart

**Pattern:** Future-based async/await with StreamController for events

**Key Example:**
- WebSocket connection returns Future: `_channel = WebSocketChannel.connect(Uri.parse(_serverUrl!))`
- Listeners handle async messages: `_channel!.stream.listen((message) { ... })`
- Error propagation via StreamController: `_onErrorController` broadcasts errors to UI
- File: `MobileControlApp/lib/services/websocket_service.dart` (lines 65-76)

### JavaScript

**Pattern:** Promise-based async/await for Blockly execution

**Key Example:**
- Block code generation creates async functions: `await moveDistance(...); await wait(...);`
- Heartbeat timer: `heartbeatInterval = setInterval(..., 2000)`
- Reconnection with exponential backoff
- File: `BlockBasedLearning/WebInterface/blockly-robot.js` (lines 281-290): Async code generation

## Voltage/Hardware-Specific Patterns

### Motor Control (Arduino)

**Calibration constants:**
- `LEFT_MOTOR_FACTOR = 1.05` (increase if drifting right)
- `RIGHT_MOTOR_FACTOR = 0.90` (decrease if drifting right)
- `TURN_CALIBRATION_FACTOR = 1.12` (12% more rotation for slippage compensation)
- File: `Arduino_Slave/Arduino_Slave.ino` (lines 32-39)

**Encoder specifications:**
- 21 pulses per rotation, 65mm wheel → 9.72mm per pulse
- 145mm wheelbase
- Pin Change Interrupts (PCINT8/PCINT9) on A0/A1
- File: `Arduino_Slave/Arduino_Slave.ino` (lines 26-30)

### Servo Control

**Pattern:**
- Attach/detach to save power
- Deadband of 1 degree (only update if delta > 1)
- Auto-detach after 2 seconds of inactivity
- File: `Arduino_Slave/Arduino_Slave.ino` (lines 332-347)

---

*Convention analysis: 2026-02-05*
