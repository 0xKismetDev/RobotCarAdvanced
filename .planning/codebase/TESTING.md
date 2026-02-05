# Testing Patterns

**Analysis Date:** 2026-02-05

## Test Framework

### Dart/Flutter

**Test Framework:**
- Flutter test framework (built-in)
- Package: `flutter_test` (dev_dependency)
- File: `MobileControlApp/pubspec.yaml` (line 24-25)

**Run Commands:**
```bash
cd MobileControlApp
flutter test                  # Run all tests
flutter test --coverage       # Generate coverage report
flutter test -v              # Verbose output
```

**Current Test Status:**
- One placeholder test file exists: `MobileControlApp/test/widget_test.dart`
- File contains boilerplate "Counter increments smoke test" (lines 14-29)
- **CRITICAL:** This is a stub test that does not test actual app functionality

### Arduino C++

**Test Framework:** None

**Testing approach:** Hardware integration only
- Code runs directly on Arduino/ESP32
- No unit testing framework
- Serial monitor used for validation
- Manual testing via physical hardware

### JavaScript/Blockly

**Test Framework:** None

**Testing approach:** Browser manual testing
- No test runner configured (Jest, Mocha absent)
- Testing done through browser dev tools
- Manual verification of WebSocket communication

### Python

**Test Framework:** None

**Testing approach:** Manual server startup and browser connection
- Server.py tested by running and connecting from browser
- No automated tests

## Test File Organization

### Location

**Dart:**
- `MobileControlApp/test/` directory
- Pattern: `test/widget_test.dart` for main test file
- File: `MobileControlApp/test/widget_test.dart` exists as single test

**Arduino C++:**
- No test files; all testing on hardware

**JavaScript:**
- No test files; manual browser testing only

### Naming

**Dart:**
- File naming: `*_test.dart`
- Test function naming: `testWidgets('...')` for widget tests

**Arduino/JavaScript/Python:**
- Not applicable (no test framework)

## Test Structure

### Dart Widget Test Example

**Current pattern from `widget_test.dart` (lines 13-29):**
```dart
void main() {
  testWidgets('Counter increments smoke test', (WidgetTester tester) async {
    // Build our app and trigger a frame.
    await tester.pumpWidget(const MyApp());

    // Verify that our counter starts at 0.
    expect(find.text('0'), findsOneWidget);
    expect(find.text('1'), findsNothing);

    // Tap the '+' icon and trigger a frame.
    await tester.tap(find.byIcon(Icons.add));
    await tester.pump();

    // Verify that our counter has incremented.
    expect(find.text('0'), findsNothing);
    expect(find.text('1'), findsOneWidget);
  });
}
```

**Patterns observed:**
- Setup: `tester.pumpWidget()` loads widget
- Action: `tester.tap()` interacts with UI
- Assertion: `expect(find.XXX, matcher)` verifies state
- Async: `async`/`await` for asynchronous operations

### Setup/Teardown

**Dart pattern:**
- No explicit setUp/tearDown observed in current code
- Widget tests self-contained with pumpWidget() setup
- **Note:** Service tests would need to mock WebSocket and timer streams

### Assertion Pattern

**Dart:**
- `expect(actual, matcher)` syntax
- Common matchers: `findsOneWidget`, `findsNothing`
- Examples from flutter_test: `find.text()`, `find.byIcon()`

## Mocking

### Dart

**Framework:** No mocking library imported

**Required for proper testing:**
- mockito (for mocking WebSocketChannel)
- fake_async (for timer testing)
- Not currently in pubspec.yaml

**What would need mocking:**
- WebSocketChannel connection and stream
- Timer for heartbeat functionality
- Shared preferences for settings
- File: `MobileControlApp/lib/services/websocket_service.dart` (lines 52-80)

**What NOT to mock:**
- ChangeNotifier state updates (use real notifiers)
- Material Design widgets (test actual rendering)
- Dart language features

### Arduino C++

**Approach:** No mocking needed; direct hardware testing
- Test I2C communication via physical connection
- Serial monitor shows command echo and responses

### JavaScript

**No mocking framework** - Manual testing via browser console

## Fixtures and Factories

### Test Data

**Dart pattern (needed but not implemented):**
```dart
// Factory for creating test SensorData
final testSensorData = SensorData(
  distance: 50,
  servoPosition: 90,
  batteryPercent: 75,
  motorStatus: 'FORWARD',
  timestamp: 123456,
  arduinoConnected: true,
);
```

**Location:** Would go in `MobileControlApp/test/fixtures/` or `test/mocks/`

**Current SensorData class:**
- File: `MobileControlApp/lib/models/sensor_data.dart` (lines 1-37)
- Has `fromJson()` factory for parsing WebSocket messages
- Could be used for test data creation

### Arduino Test Data

Not applicable - hardware tests only.

## Coverage

### Reporting

**Current:** Not enforced

**Potential command:**
```bash
flutter test --coverage
# Generates lcov.info in coverage/
# View with: genhtml coverage/lcov.info -o coverage/html
```

**No coverage thresholds configured in codebase.**

### Coverage Gaps - Critical

1. **WebSocketService connection logic:** Lines 52-101 in `websocket_service.dart`
   - Connection, reconnection, error handling untested
   - Heartbeat mechanism untested
   - Stream subscriptions not verified

2. **SensorData parsing:** `sensor_data.dart` lines 20-30
   - `fromJson()` factory not tested
   - Edge cases (missing fields, invalid values) not covered

3. **Motor control logic:** `Arduino_Slave/Arduino_Slave.ino` lines 358-424
   - `setMotors()` calibration not verified
   - Direction switching safety not validated

4. **Blockly code generation:** `blockly-robot.js` lines 270-350+
   - Generated async code not validated
   - Command queue execution untested

5. **WebSocket reconnection:** `app.js` lines 107-156
   - Exponential backoff logic untested
   - Connection state transitions not verified

## Test Types

### Unit Tests

**Status:** None implemented

**Candidates for unit testing:**
- Dart models: `SensorData.fromJson()`
- Dart utilities: Distance calculations, threshold checks
- JavaScript utilities: IP address parsing, WebSocket URL construction
- Python request handlers: Header injection

**How to implement:**
```dart
// Example unit test structure (not currently in codebase)
void main() {
  group('SensorData', () {
    test('fromJson creates correct instance', () {
      final json = {
        'distance': 50,
        'servo_position': 90,
        'motor_status': 'FORWARD',
      };
      final sensor = SensorData.fromJson(json);
      expect(sensor.distance, equals(50));
      expect(sensor.servoPosition, equals(90));
    });
  });
}
```

### Integration Tests

**Status:** None implemented

**Candidates:**
- WebSocket connection → Arduino I2C → Motor response (full stack)
- Flutter app → WebSocket → ESP32 → Arduino
- Blockly browser → ESP32 WebSocket → Arduino

**Structure (not implemented):**
```dart
void main() {
  setUpAll(() async {
    // Start mock WebSocket server
    // Start Arduino simulator
  });

  testWidgets('Motors respond to movement commands', (tester) async {
    // Connect to mock server
    // Send movement command
    // Verify encoder response
  });
}
```

### E2E Tests

**Status:** Not applicable

**Manual E2E testing performed:**
- Physical robot car with remote control app
- Blockly browser interface with live ESP32
- No automated E2E framework

## Common Patterns

### Async Testing - Dart

**Pattern (needed but not implemented):**
```dart
test('WebSocket connects and receives data', () async {
  final service = WebSocketService();

  // Listen for connection
  final connected = service.onConnected.first;

  // Initiate connection
  service.connect('192.168.4.1', 81);

  // Wait for connection event
  await connected;
  expect(service.isConnected, isTrue);
});
```

**Current test file:**
- File: `MobileControlApp/test/widget_test.dart` uses `async` with `WidgetTester`
- Pattern: `testWidgets('name', (WidgetTester tester) async { ... })`

### Error Testing - Dart

**Pattern (not implemented but needed):**
```dart
test('Connection error propagates to UI', () async {
  final service = WebSocketService();

  final errorStream = service.onError;
  final errorCompleter = Completer<String>();

  errorStream.listen((error) {
    errorCompleter.complete(error);
  });

  // Trigger connection failure
  service.connect('invalid.ip.address', 81);

  final error = await errorCompleter.future;
  expect(error, contains('Failed to connect'));
});
```

### Hardware Testing - Arduino

**Current approach (manual):**
1. Flash code to Arduino/ESP32 via Arduino IDE
2. Open Serial Monitor at 115200 baud
3. Observe initialization messages
4. Send I2C commands from ESP32
5. Read Serial responses and verify behavior

**Example command sequence observed in `Arduino_Slave.ino`:**
```
=== arduino robot car - i2c slave with encoders ===
i2c address: 0x08 (sda=a4, scl=a5)
encoder pins: left=A0, right=A1, 21 pulses/rotation
ready! waiting for i2c commands...
i2c rx (5 bytes): M 150 150
cmd: M 150 150
motors: L=150 R=150
```

## Known Testing Limitations

### Critical Gaps

1. **No Flutter widget tests** - The single test file is a stub
   - Cannot verify UI responsiveness
   - Cannot validate real-time sensor data updates
   - Cannot test error UI states

2. **No Arduino unit tests** - All tests manual on hardware
   - Cannot catch motor control regressions
   - Cannot verify encoder ISR logic
   - Calibration changes require physical testing

3. **No Blockly validation** - Code generation not verified
   - Cannot ensure generated async code is correct
   - Cannot test block parameter validation
   - WebSocket message format not validated

4. **No mock servers** - Cannot test offline behavior
   - Connection loss scenarios untested
   - Reconnection logic not validated
   - Timeout handling not verified

### Manual Testing Workarounds (Current)

**For developers modifying code:**
1. Run Flutter app and manually test UI
2. Flash Arduino and watch Serial Monitor
3. Test Blockly in browser connected to real ESP32
4. Use browser DevTools Network tab to inspect WebSocket frames
5. Physical testing on actual robot car

## Test Execution in CI/CD

**Status:** No CI/CD pipeline configured

**If implemented, would run:**
```bash
# Dart tests
cd MobileControlApp
flutter test
flutter test --coverage

# JavaScript linting (if added)
npm install && npm run lint

# Python linting (if added)
pip install pylint && pylint BlockBasedLearning/WebInterface/server.py
```

## Recommended Testing Strategy

### Immediate Priority

1. **Convert placeholder test to real test:**
   - Test WebSocketService connection flow
   - Mock WebSocketChannel
   - File: `MobileControlApp/test/widget_test.dart`

2. **Add SensorData tests:**
   - Test JSON parsing with various inputs
   - Test null/missing field handling
   - Location: Create `MobileControlApp/test/models/sensor_data_test.dart`

3. **Add Arduino simulation tests:**
   - Create mock I2C interface
   - Test motor control logic without hardware
   - Location: `Arduino_Slave/tests/` (new directory)

### Medium Priority

1. Set up mock WebSocket server for integration tests
2. Add Blockly code generation validation
3. Create Python server tests

### Long-term

1. Implement GitHub Actions CI/CD
2. Set up code coverage reporting (target: 80%+)
3. Add E2E tests with physical robot simulation

---

*Testing analysis: 2026-02-05*
