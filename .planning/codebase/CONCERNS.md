# Codebase Concerns

**Analysis Date:** 2026-02-05

## Tech Debt

**Duplicate Firmware Implementations:**
- Issue: Two separate ESP32 firmware implementations with significant code duplication (`ESP32_Master.ino` vs `BlockBasedLearning/ESP32_Code.ino`)
- Files: `ESP32_Master/ESP32_Master.ino` (305 lines), `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` (658 lines)
- Impact: Maintenance burden - bug fixes must be applied to both versions. The Blockly version is substantially newer with reliability improvements (watchdog timer, memory checks, aggressive heartbeat) that don't exist in Master version
- Fix approach: Consolidate to single firmware with compile-time feature toggles or runtime configuration for camera vs blockly mode

**Code Duplication in Web Interface:**
- Issue: Redundant calibration, motor control, and sensor reading logic scattered across multiple functions
- Files: `BlockBasedLearning/WebInterface/app.js` (1361 lines)
- Impact: Bug fixes need to be applied in multiple places (e.g., `moveRobot()` vs `turnDegrees()` both have collision detection and calibration logic)
- Fix approach: Extract movement logic into reusable functions: `_performMovement()`, `_applyMotorCalibration()`, `_handleCollisionProtection()`

**Blocking Movements in Arduino Firmware:**
- Issue: Motor control uses blocking `while(true)` loops in `moveDistance()` and `rotateDegrees()` functions
- Files: `Arduino_Slave/Arduino_Slave.ino` lines 528, 605
- Impact: While I2C interrupts can still be processed (via `delayMicroseconds(500)`), long movements block serial communication and prevent responsive servo adjustments. Max 15-second timeout can leave robot stuck if encoder fails
- Fix approach: Implement non-blocking state machine for movements or set shorter hard timeout (5 seconds max)

**Hardcoded Calibration Values Across Multiple Files:**
- Issue: Motor calibration factors and turn calibration duplicated/inconsistent
- Files: `Arduino_Slave/Arduino_Slave.ino` (lines 34-39), `BlockBasedLearning/WebInterface/app.js` (lines 376-379, 556-562, 904-908)
- Impact: Changing calibration requires updates in 2+ files. Blockly web interface hardcodes different turn timing calculations than Arduino firmware
- Fix approach: Create single calibration configuration file (JSON or EEPROM-stored on Arduino) that all systems read

**Unreliable Connection Status Tracking:**
- Issue: Multiple disconnected connection state management approaches
- Files: `BlockBasedLearning/WebInterface/app.js` (11+ boolean flags: `isReconnecting`, `connectionLost`, `stopRequested`, `clientConnected`, `isExecutingCode`), `ESP32_Master/ESP32_Master.ino` (separate client/arduino/camera flags), `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` (different set of flags)
- Impact: State can become inconsistent (e.g., `clientConnected=true` but `connectionLost=true` simultaneously). Hard to debug connection issues
- Fix approach: Implement single source-of-truth state machine enum with clear transitions

---

## Known Bugs

**Servo Detach Race Condition:**
- Symptoms: Servo may twitch if commands arrive during detach timeout window
- Files: `Arduino_Slave/Arduino_Slave.ino` lines 185-188
- Trigger: Servo command arrives between 2-second inactivity check and detach operation
- Current mitigation: 1-degree deadband (line 332) prevents unnecessary re-attachments
- Status: Minor issue - unlikely in typical usage but could manifest under specific timing

**I2C Data Truncation Risk:**
- Symptoms: Long sensor responses may be truncated silently
- Files: `Arduino_Slave/Arduino_Slave.ino` lines 219-226, `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` lines 497-516
- Trigger: Comma-delimited response format doesn't validate all fields present before parsing. If response is cut short (>31 bytes), parsing silently accepts partial data
- Example: Response `"100,123,456,FORWARD"` vs truncated `"100,123,456,FOR"` - both parse but status is corrupted
- Workaround: Responses validated client-side; missing commas cause parse rejection
- Fix approach: Add response length validation and newline terminator guarantee

**Encoder Movement Timeout Has No Lower Bound on Movement Duration:**
- Symptoms: Very short movements (<100ms) may timeout before encoders register pulse
- Files: `Arduino_Slave/Arduino_Slave.ino` lines 52-54, 497-498
- Trigger: `D 10 F 50` (10mm forward at speed 50) needs only ~1 pulse but timeout is MIN 3000ms + movement time
- Impact: No safety issue but awkward for precise calibration movements
- Fix approach: Add minimum pulse count (e.g., 2 pulses) instead of time-based timeout for encoder movements

**Watchdog Timer Panic on Blockly Firmware:**
- Symptoms: Long operations (sensor scan, complex block sequences) could trigger watchdog restart
- Files: `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` lines 70-76
- Trigger: 15-second watchdog timeout set globally; long blocking scan operations come close to this limit
- Current scan operation can take ~7 seconds (180° in 30° steps × 250ms per step), leaving narrow safety margin
- Mitigation: `esp_task_wdt_reset()` called in main loop (line 118)
- Risk: If user creates infinite loop in Blockly code without any blocking operations, watchdog will trigger
- Fix approach: Wrap code execution in timeout handler; reset WDT before each block execution

---

## Security Concerns

**Hardcoded WiFi Credentials in Firmware:**
- Risk: Plain-text WiFi passwords embedded in published firmware code
- Files:
  - `ESP32_Master/ESP32_Master.ino` lines 10-13: `CAM_PASSWORD = "12345678"`, `FALLBACK_PASSWORD = "12345678"`
  - `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` lines 13-14: `AP_PASSWORD = "12345678"`
  - `ESP32_Camera/ESP32_Camera.ino`: Same pattern
- Impact: Anyone with access to code repository can connect to robot's WiFi network
- Mitigation: Default password is simple/memorable for educational use case; network is local-only with no internet exposure
- Recommendation: Store credentials in EEPROM/NVS at runtime; allow factory reset to set credentials; document that this is development firmware

**No Authentication on WebSocket Commands:**
- Risk: Any client that connects to WebSocket can execute robot commands without authentication
- Files: `ESP32_Master/ESP32_Master.ino` lines 118-184, `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` lines 225-290
- Impact: On local network, any device can connect and control robot (e.g., move it into danger, trigger sensors)
- Mitigation: WiFi network itself is access-controlled; robot car is educational platform without safety-critical operations
- Recommendation: For multi-user classroom: add simple token-based auth or WiFi whitelist by MAC address

**No Input Validation on I2C Commands:**
- Risk: Arduino processes I2C commands with minimal validation of parameters
- Files: `Arduino_Slave/Arduino_Slave.ino` lines 231-356
- Impact: Malformed commands (e.g., `D -5000 F 500`) could cause integer overflow or undefined behavior, though constraints mitigate this
- Mitigation: `constrain()` applied to all motor speeds (lines 245-246, 319-320); distance and angle inputs constrained at web/Blockly layer
- Recommendation: Add explicit range validation with error responses (currently fails silently)

**Collision Detection Cannot Be Disabled:**
- Risk: Web interface collision protection (lines 472-480 in app.js) hardcoded 5cm threshold
- Impact: Cannot perform legitimate operations like pushing objects or driving through narrow gaps
- Files: `BlockBasedLearning/WebInterface/app.js` lines 472-481
- Fix: Add calibration setting for collision threshold or disable-collision-check button

---

## Performance Bottlenecks

**I2C Blocking on Sensor Reads:**
- Problem: Master ESP32 calls `Wire.requestFrom()` synchronously with 25ms timeout per read
- Files: `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` lines 497-516
- Impact: Sensor reads happen every 250ms and can block WebSocket processing for up to 50ms (line 44: `I2C_READ_TIMEOUT`)
- Symptoms: Joystick response might have 50-100ms lag during sensor read
- Improvement path: Implement async I2C with DMA; cache stale sensor data instead of blocking

**Inefficient Servo Detach/Attach Cycles:**
- Problem: Servo detached every 2 seconds if inactive, re-attached on next command
- Files: `Arduino_Slave/Arduino_Slave.ino` lines 185-188, 336-339
- Impact: 15ms delay on every servo move command (line 338: `delay(15)` for servo initialization)
- Improvement: Keep servo attached but send reduced PWM signal or hold at same angle without re-attach

**Web Interface Heartbeat Interval Too Aggressive:**
- Problem: Ping interval every 2 seconds is more than enough for local network
- Files: `BlockBasedLearning/WebInterface/app.js` lines 237-261 (2000ms), line 422 (`MIN_COMMAND_INTERVAL = 10ms` is also very tight)
- Impact: Unnecessary CPU and network overhead, especially on battery-powered devices
- Improvement: Increase to 5-10 second interval for local LAN

**Repeated Distance Sensor Initialization:**
- Problem: `readDistance()` initializes sensor pin state on every read
- Files: `Arduino_Slave/Arduino_Slave.ino` lines 437-444
- Impact: 60ms minimum read interval means sensor updates only every 100-250ms in practice; adds latency
- Improvement: Implement faster polling (10-20ms intervals) with rolling average buffer

---

## Fragile Areas

**Encoder-Based Movement Timing (Arduino):**
- Files: `Arduino_Slave/Arduino_Slave.ino` lines 493-562 (moveDistance), 564-639 (rotateDegrees)
- Why fragile:
  - Depends on exact wheel circumference (65mm diameter) and encoder pulse count (21/rotation)
  - Calibration factors (LEFT_MOTOR_FACTOR=1.05, RIGHT_MOTOR_FACTOR=0.90) are hand-tuned for specific hardware
  - Wheel wear, battery voltage sag, and surface friction affect accuracy
  - Turn calibration uses arc-length geometry that's sensitive to wheelbase measurement (145mm)
- Safe modification:
  - Add encoder error logging (count vs expected) for every movement
  - Implement adaptive calibration that adjusts factors based on last 10 movements
  - Create calibration routine that measures actual vs commanded distance/angle
- Test coverage: No automated testing; calibration is manual trial-and-error

**Sensor Data Parsing (ESP32 Master):**
- Files: `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` lines 519-541
- Why fragile:
  - Comma-delimited format with no error codes or validation
  - If Arduino response format changes, parsing breaks silently with stale data
  - No checksum or length prefix to detect corruption
  - Arduino may return partial responses if buffer overflowed
- Safe modification:
  - Add length prefix: `<length>:<data>` format
  - Implement strict validation of all 4 fields before accepting
  - Return error response if any field is missing/invalid
- Test coverage: No test cases for malformed responses

**Blockly Code Generation (Web Interface):**
- Files: `BlockBasedLearning/WebInterface/blockly-robot.js` (479 lines)
- Why fragile:
  - Custom code generator must handle async timing for blocking operations
  - Servo scan functions use fixed delays (250ms per angle) that may not work if servo is slow
  - Turn timing uses lookup table (lines 556-562 in app.js) that's speed-dependent but missing intermediate speeds
  - Collision detection during movement uses polling (100ms interval, line 508) that could miss fast obstacles
- Safe modification:
  - Add feedback from servo position sensor before proceeding
  - Use encoder feedback to detect movement completion rather than duration
  - Make all timing configurable per-robot calibration
- Test coverage: Only manual Blockly testing; no automated test suite

**WiFi Reconnection Logic (Blockly ESP32):**
- Files: `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` lines 135-170, 629-646
- Why fragile:
  - Server-side ping sent every 2 seconds but client-side may be hung (JavaScript blocking)
  - Missed pong counter (lines 147-150) can trigger disconnect during long operations
  - Watchdog timer (15s) could trigger while waiting for client response, causing unexplained resets
  - No graceful degradation - hardware becomes unresponsive if client/ESP32 loses sync
- Safe modification:
  - Extend pong timeout during code execution (currently 10s, could be 30s)
  - Add diagnostic output showing internal state when disconnect happens
  - Implement heartbeat at multiple levels (WebSocket protocol, JSON app-level, I2C command response)
- Test coverage: Tested manually with real networks; no automated timeout/disconnect scenarios

---

## Scaling Limits

**WebSocket Message Queue Growth:**
- Current capacity: `commandQueue` is unbounded array (app.js line 21)
- Limit: With very slow robot or network lag, commands accumulate. 1000 queued commands consuming ~50KB memory
- Scaling path: Implement bounded queue with max 100 items; drop oldest commands when full instead of growing unbounded

**I2C Transaction Size:**
- Current capacity: Response limited to 31 bytes (Arduino_Slave.ino line 224)
- Limit: Sensor parsing expects exact format `distance,leftEnc,rightEnc,status\n` - future fields won't fit
- Scaling path: Implement 2-stage protocol: request with field list, response with JSON or protobuf

**ESP32 Memory Usage:**
- Current: Free heap monitoring in BlockBasedLearning/ESP32_Code.ino (lines 587-603)
- Limit: Restart triggered if heap drops below 15KB (line 46)
- Current usage: Approximately 100-120KB under normal operation; margin is tight
- Scaling path: Profile memory usage with multiple concurrent WebSocket clients; implement connection limits

**Blockly Workspace Size:**
- Current: No limit on program complexity in localStorage
- Limit: localStorage typically 5-10MB per domain but Blockly workspace XML grows quadratically
- Impact: Very complex programs (100+ blocks) could cause browser lag
- Scaling: Implement project size warning at 50KB, implement undo history limit

---

## Dependencies at Risk

**WebSocketsServer Library (Arduino):**
- Risk: External library by Markus Sattler, not actively maintained (last update 2021)
- Impact: Security vulnerabilities in library not patched; compatibility issues with newer Arduino-esp32 versions
- Current usage: `WebSocketsServer webSocket` - core to entire system
- Migration plan: Evaluate `AsyncWebServer` library or native ESP-IDF WebSocket implementation; both more actively maintained

**ArduinoJson Library (Arduino):**
- Risk: Version pinning required; library major versions have breaking changes
- Impact: Firmware compilation may break with library updates
- Recommendation: Pin specific version (e.g., 6.x) in platformio.ini or Arduino IDE library manager

**Flutter `web_socket_channel` Package:**
- Risk: Deprecated in favor of native WebSocket implementation (dart:io)
- Impact: May not receive updates for Flutter 3.x+
- Files: `MobileControlApp/lib/services/websocket_service.dart` line 4
- Migration: Replace with `web_socket` package or dart:io WebSocket

---

## Missing Critical Features

**No Emergency Stop Button:**
- Problem: Code execution can only be stopped from browser (app.js `stopCode()`)
- Blocks: Cannot stop robot safely if:
  - Browser crashes
  - Network drops mid-execution
  - Operator loses connection
- Fix: Add hardware button on ESP32 that immediately sets `M 0 0` (motor stop)

**No Battery Monitoring:**
- Problem: `SensorData` includes `batteryPercent` field but Arduino never provides it (always 0)
- Blocks: Cannot detect low battery or brownout conditions
- Files: `Arduino_Slave/Arduino_Slave.ino` - no ADC reading for battery
- Fix: Add ADC read on battery input pin; send in sensor response

**No Servo Position Feedback:**
- Problem: Servo angle commands are sent but actual servo position never verified
- Blocks: Cannot reliably wait for servo to settle before distance read
- Impact: Scan operations use fixed 250ms delay instead of confirming servo movement complete
- Files: `BlockBasedLearning/WebInterface/app.js` lines 730-755
- Fix: Add feedback servo to main servo motor (potentiometer analog read)

**No Collision Avoidance in Blockly Code:**
- Problem: Collision detection in web interface (app.js lines 472-480) is hardcoded 5cm, cannot be configured
- Blocks: Cannot create obstacle-avoiding programs in Blockly without manual if-obstacle blocks
- Fix: Add distance-threshold configuration in Blockly settings; expose as parameter to movement blocks

---

## Test Coverage Gaps

**No Automated Tests for Encoder Movement:**
- What's not tested:
  - Various movement distances (10mm, 100mm, 500mm, 1000mm)
  - Various speeds (50-255 PWM)
  - Surface friction variations (carpet vs hardwood)
  - Worn wheels (degraded accuracy)
  - Battery voltage sag (affects PWM actual speed)
- Files: `Arduino_Slave/Arduino_Slave.ino` lines 493-639
- Risk: Encoder calibration factors may not work on different robots; silent accuracy degradation
- Priority: **HIGH** - affects core functionality

**No Automated Tests for WebSocket Reconnection:**
- What's not tested:
  - Sudden network disconnection during code execution
  - Reconnection with queued commands
  - Multiple clients attempting to connect simultaneously
  - Rapid connect/disconnect cycles
  - Client timeout while robot is moving
- Files: `BlockBasedLearning/WebInterface/app.js` lines 107-229, `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` lines 135-170
- Risk: Undetected race conditions leading to hung connections or duplicate commands
- Priority: **HIGH** - affects reliability

**No Automated Tests for I2C Command Parsing:**
- What's not tested:
  - Malformed command strings
  - Out-of-range parameter values
  - Missing required fields
  - Command buffer overflow (>31 bytes)
  - Concurrent I2C requests
- Files: `Arduino_Slave/Arduino_Slave.ino` lines 231-356
- Risk: Undefined behavior or silent command drops
- Priority: **MEDIUM**

**No Automated Tests for Sensor Data Parsing:**
- What's not tested:
  - Corrupted/partial responses from Arduino
  - Missing fields in CSV response
  - Unicode/invalid characters in status string
  - Extreme distance values (0cm, >500cm)
- Files: `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` lines 518-542
- Risk: Stale sensor data accepted, causing incorrect collision detection or calibration
- Priority: **MEDIUM**

**No E2E Tests for Complete Movement Cycle:**
- What's not tested:
  - `moveDistance()` -> encoder count -> verify actual distance traveled
  - `rotateDegrees()` -> encoder counts -> verify actual angle rotated
  - Movement with obstacle -> collision detection -> automatic stop
  - Long-running Blockly programs (>5 minutes)
- Files: All firmware + web interface
- Risk: Silent accuracy degradation, unreliable operation
- Priority: **HIGH**

---

## Minor Issues & Recommendations

**Hardcoded Constants Should Be Configurable:**
- Issue: Values like `WHEEL_DIAMETER`, `PULSES_PER_ROTATION`, `WHEELBASE` are Arduino-only constants
- Files: `Arduino_Slave/Arduino_Slave.ino` lines 26-30
- Impact: If robot hardware is modified, firmware must be recompiled
- Recommendation: Store in Arduino EEPROM; provide calibration CLI or Blockly blocks to adjust

**Inconsistent Logging Across Firmware:**
- Issue: `ESP32_Master` uses simple `Serial.println()` while `BlockBasedLearning/ESP32_Code` has structured logging with tags (`[WS]`, `[MEM]`, `[WiFi]`)
- Impact: Makes debugging harder when switching between firmware versions
- Recommendation: Standardize logging format across all firmware

**Memory Leak Risk in JavaScript Command Queue:**
- Issue: `commandQueue` and `pendingCommands` never automatically clear old entries
- Files: `BlockBasedLearning/WebInterface/app.js` lines 21-22
- Impact: After hours of operation, queue objects grow unbounded
- Recommendation: Implement automatic cleanup of commands >5 minutes old

**No Rate Limiting on Web Interface Commands:**
- Issue: `sendCommandWithRateLimit()` (line 440) checks 10ms minimum but `sendCommand()` (line 270) has no limit
- Impact: User could send 1000 commands/second directly to WebSocket
- Recommendation: Always route through rate-limited function; add server-side rate limiter

---

*Concerns audit: 2026-02-05*
