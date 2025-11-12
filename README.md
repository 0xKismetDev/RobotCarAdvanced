# RobotCarAdvanced

Advanced robotics projects for the AZ Delivery Robot Car kit with ESP32 WiFi control.

## Repository Structure

```
├── BlockBasedLearning/     # Block-based visual programming interface
├── ESP32_Master/          # Main ESP32 WebSocket server code
├── Arduino_Slave/         # Arduino motor controller (I2C slave)
├── ESP32_Camera/          # ESP32-CAM streaming extension
└── MobileControlApp/      # Flutter mobile control application
```

## Hardware Wiring

### Core System: ESP32 + Arduino Communication (I2C)

The robot uses an ESP32 as the main controller (WiFi/WebSocket server) and Arduino UNO for motor control.

```
ESP32                Arduino UNO (V5 Extension Board)
-----                -------------------------------
GPIO21 (SDA)    →    A4 (SDA) - I2C Data Line
GPIO22 (SCL)    →    A5 (SCL) - I2C Clock Line
GND             →    GND
VIN/3.3V        →    3.3V or 5V Power Rail
```

### Arduino Pin Assignments
```
Pin  | Function              | Component
-----|----------------------|------------------
2    | IN1 (Motor A Dir)    | L298N Motor Driver
4    | IN2 (Motor A Dir)    | L298N Motor Driver
5    | ENA (Motor A PWM)    | L298N Motor Driver
6    | ENB (Motor B PWM)    | L298N Motor Driver
7    | IN3 (Motor B Dir)    | L298N Motor Driver
8    | IN4 (Motor B Dir)    | L298N Motor Driver
9    | Ultrasonic Trigger   | HC-SR04 Sensor
10   | Ultrasonic Echo      | HC-SR04 Sensor
11   | IR Receiver Signal   | IR Receiver Module
A0   | Left Encoder Input   | Optical Wheel Encoder (PCINT8)
A1   | Right Encoder Input  | Optical Wheel Encoder (PCINT9)
A2   | Servo Motor Control  | SG90 Servo
A4   | SDA (I2C Data)       | ESP32 Communication
A5   | SCL (I2C Clock)      | ESP32 Communication
```

### Encoder Wiring Configuration

The robot uses optical spoke wheel encoders for precise distance and rotation measurements.

**Encoder Specifications:**
- Type: Optical light-based sensors with spoke wheels
- Resolution: 21 pulses per complete wheel rotation
- Distance per pulse: ~9.72mm (calculated from 65mm wheel diameter)
- Wheelbase: 145mm center-to-center (for rotation calculations)

**Wiring:**
```
Left Encoder:
- Signal → Arduino A0 (PCINT8)
- VCC → 5V
- GND → GND

Right Encoder:
- Signal → Arduino A1 (PCINT9)
- VCC → 5V
- GND → GND
```

**Internal Pull-ups:** The Arduino code enables internal pull-up resistors on A0 and A1, so no external resistors are needed.

### ESP32-CAM Extension (Optional)
For camera streaming, connect an ESP32-CAM module:
- Power from car's power supply (5V/GND)
- No data connection needed (operates independently via WiFi)
- Mount on front of car for best viewing angle

## Setup Instructions

### 1. Block-Based Programming

#### Requirements:
- WebSocketsServer library
- ArduinoJson library
- Python 3.6+

#### Setup:
1. Upload `Arduino_Slave/Arduino_Slave.ino` to Arduino UNO
2. Upload `BlockBasedLearning/ESP32_Code/ESP32_Code.ino` to ESP32
3. Start web interface:
   ```bash
   cd BlockBasedLearning/WebInterface
   python3 server.py
   ```
4. Open browser to `http://localhost:8080`
5. Connect to WiFi: `RobotCar-Blockly` (password: `12345678`)

### 2. ESP32 WebSocket Control

#### Setup:
1. Upload `ESP32_Master/ESP32_Master.ino` to ESP32
2. Upload `Arduino_Slave/Arduino_Slave.ino` to Arduino
3. Connect via WebSocket to `ws://192.168.4.1:81`

Note: ESP32_Master first tries to connect to ESP32-CAM network. If not found, it creates its own AP (`RobotCar-AP`)

### 2.1. ESP32-CAM Streaming (Optional)

#### Setup:
1. Upload `ESP32_Camera/ESP32_Camera.ino` to ESP32-CAM
2. Configure WiFi credentials in code
3. Access stream at `http://[ESP32-CAM-IP]/stream`

### 2.2. Mobile Control App

#### Requirements:
- Flutter SDK 3.0+

#### Setup:
```bash
cd MobileControlApp
flutter pub get
flutter run
```

## Required Libraries

### Arduino IDE Libraries:
- **WebSocketsServer** by Markus Sattler (for ESP32)
- **ArduinoJson** by Benoit Blanchon (for ESP32)
- **Wire** (built-in, for I2C)
- **Servo** (built-in, for servo control)

### Python:
- Python 3.6+ (for Block Interface web server)

### Flutter:
- Flutter SDK 3.0+
- See `MobileControlApp/pubspec.yaml` for dependencies

## WiFi Configuration

### Block-Based Programming
- SSID: `RobotCar-Blockly`
- Password: `12345678`
- IP: `192.168.4.1`

### ESP32 WebSocket Control
- SSID: `RobotCar-AP`
- Password: `12345678`
- IP: `192.168.4.1`

### ESP32-CAM Stream
- SSID: `ESP32-CAM-STREAM`
- Password: `12345678`
- IP: `192.168.4.1`

## Communication Protocol

### WebSocket Commands (ESP32 ↔ Web Interface)

JSON over WebSocket (port 81):

```json
// Encoder-Based Precise Movement (NEW)
{"type": "command", "action": "moveDistance", "distance": 200, "direction": "forward", "speed": 150}
{"type": "command", "action": "moveDistance", "distance": 100, "direction": "backward", "speed": 150}
{"type": "command", "action": "rotateDegrees", "degrees": 90, "direction": "left", "speed": 150}
{"type": "command", "action": "rotateDegrees", "degrees": 45, "direction": "right", "speed": 150}
{"type": "command", "action": "resetEncoders"}
{"type": "command", "action": "getEncoders"}

// Direct Motor Control
{"type": "command", "action": "differential", "leftSpeed": 150, "rightSpeed": 150}

// Sensors
{"type": "command", "action": "read_distance"}
{"type": "command", "action": "servo", "angle": 90}
{"type": "command", "action": "scan"}

// System
{"type": "ping"}
```

### I2C Commands (ESP32 → Arduino)

String-based commands sent over I2C bus:

```
// Encoder-Based Movement (NEW Protocol)
D <distance_mm> <direction> <speed>     // Distance movement
  Example: "D 200 F 150"  (move 200mm forward at speed 150)
  Example: "D 100 B 150"  (move 100mm backward at speed 150)

R <degrees> <direction> <speed>         // Rotation movement
  Example: "R 90 L 150"   (rotate 90° left at speed 150)
  Example: "R 45 R 150"   (rotate 45° right at speed 150)

E                                       // Reset encoder counters to zero
Q                                       // Query encoder values

// Direct Motor Control
M <leftSpeed> <rightSpeed>              // Set motor speeds (-255 to +255)
  Example: "M 150 150"    (both motors forward at 150)
  Example: "M -100 100"   (turn left: left backward, right forward)

// Servo & Sensors
S <angle>                               // Set servo angle (0-180)
  Example: "S 90"

SCAN                                    // Perform 180° servo scan
READ_DISTANCE                          // Read ultrasonic sensor once

// Legacy Commands (backward compatible)
MOVE <distance_mm> <speed>             // Old format (sign indicates direction)
TURN <degrees> <speed>                 // Old format (sign indicates direction)
RESET_ENCODERS                         // Old format for encoder reset
```

### I2C Data Response (Arduino → ESP32)

CSV format string:

```
distance,leftEncoder,rightEncoder,status\n
```

Example: `"25,42,43,MOVING\n"`

- **distance**: Ultrasonic sensor reading in cm
- **leftEncoder**: Left wheel pulse count
- **rightEncoder**: Right wheel pulse count
- **status**: Motor status (STOPPED, FORWARD, BACKWARD, LEFT, RIGHT, MOVING)

## Encoder-Based Blockly Blocks

The Block-Based programming interface includes blocks for precise encoder-based movement:

### Movement Blocks

**🚗 Fahre [distance] mm vorwärts** (Move Forward)
- Moves the robot exactly the specified distance forward
- Distance: 10-1000mm
- Speed options: slow (100), medium (150), fast (200)
- Example: Move 200mm forward at medium speed

**🚗 Fahre [distance] mm rückwärts** (Move Backward)
- Moves the robot exactly the specified distance backward
- Same parameters as forward movement

**🔄 Drehe [degrees] Grad nach [left/right]** (Rotate Degrees)
- Rotates the robot precisely by the specified angle
- Degrees: 1-360°
- Direction: left or right
- Fixed speed: 150
- Example: Rotate 90° right

**🛑 STOPP** (Stop)
- Immediately stops all motors

### Encoder Utility Blocks

**🔄 Encoder zurücksetzen** (Reset Encoders)
- Resets both encoder counters to zero
- Useful at the start of a program or for resetting position tracking

**📊 Encoder [left/right]** (Get Encoder Value)
- Returns the current pulse count for the selected wheel
- Can be used in conditions or for display
- Example: Check if left encoder > 50

### Example Programs

**Square Pattern (200mm sides):**
```
Repeat 4 times:
  Move 200mm forward (medium speed)
  Wait 0.5 seconds
  Rotate 90° right
  Wait 0.5 seconds
```

**Figure-8 Pattern:**
```
Repeat 2 times:
  // First circle
  Repeat 8 times:
    Move 100mm forward
    Rotate 45° left

  // Second circle (opposite direction)
  Repeat 8 times:
    Move 100mm forward
    Rotate 45° right
```

**Distance Tracking:**
```
Reset encoders
Move 500mm forward
Print "Left encoder: " + (Encoder left)
Print "Right encoder: " + (Encoder right)
```

## Encoder Calibration

### Motor Speed Calibration

The Arduino code includes calibration factors to compensate for differences in motor speeds:

```cpp
const float LEFT_MOTOR_FACTOR = 1.05;   // Increase if drifting right
const float RIGHT_MOTOR_FACTOR = 0.90;  // Decrease if drifting right
```

**To calibrate:**
1. Place robot on a flat surface with at least 2 meters of clear space
2. Run a program to move forward 1000mm (1 meter)
3. Measure actual distance traveled
4. If robot drifts right: Increase `LEFT_MOTOR_FACTOR` or decrease `RIGHT_MOTOR_FACTOR`
5. If robot drifts left: Decrease `LEFT_MOTOR_FACTOR` or increase `RIGHT_MOTOR_FACTOR`
6. Adjust in small increments (0.05) and retest

### Distance Calibration

If the robot consistently travels more or less than commanded:

1. The `MM_PER_PULSE` constant may need adjustment:
   ```cpp
   const float MM_PER_PULSE = WHEEL_CIRCUMFERENCE / PULSES_PER_ROTATION;
   ```

2. Verify wheel diameter (should be 65mm)
3. Count encoder pulses for one complete wheel rotation (should be 21)
4. Calculate: `MM_PER_PULSE = (π × 65mm) / 21 ≈ 9.72mm`

### Rotation Calibration

If rotations are inaccurate:

1. Verify the `WHEELBASE` constant (distance between wheel centers):
   ```cpp
   const float WHEELBASE = 145.0;  // mm
   ```

2. Measure actual distance between the center of left and right wheels
3. Update the constant if different from 145mm
4. Test 90°, 180°, and 360° rotations and adjust if needed

## Troubleshooting

### Encoders Not Counting

**Symptoms:**
- Encoder values stay at 0
- Dashboard shows "L:0 (0mm) R:0 (0mm)" even during movement

**Solutions:**
1. Check encoder wiring:
   - Left encoder signal → Arduino A0
   - Right encoder signal → Arduino A1
   - Both encoders have 5V and GND connected
2. Verify encoder spoke wheels are attached to motor shafts
3. Check for loose connections or damaged wires
4. Test encoders with Serial Monitor (Arduino code prints encoder values)

### Robot Moves but Stops Immediately

**Symptoms:**
- Robot starts moving but stops after 1-2 pulses
- Movement is very short (< 1cm)

**Solutions:**
1. Check `POSITION_TOLERANCE` setting (should be 2 pulses)
2. Verify motors have sufficient power (batteries charged)
3. Increase speed parameter if too low (minimum 100 recommended)
4. Check for mechanical issues (wheels binding, high friction)

### Inaccurate Distances

**Symptoms:**
- Robot travels more or less than commanded distance
- Consistent error (e.g., always 10% too far)

**Solutions:**
1. Recalibrate `MM_PER_PULSE` constant
2. Verify wheel diameter is actually 65mm
3. Check that encoder spoke wheels are securely attached
4. Test on flat, smooth surface (carpet can affect accuracy)

### Robot Drifts During Straight Movement

**Symptoms:**
- Robot veers left or right instead of moving straight
- One encoder counts faster than the other

**Solutions:**
1. Adjust motor calibration factors (`LEFT_MOTOR_FACTOR`, `RIGHT_MOTOR_FACTOR`)
2. Check that both wheels are the same size
3. Ensure wheels are properly inflated/seated
4. Verify motors are receiving equal power (check battery voltage)

### Inaccurate Rotations

**Symptoms:**
- 90° rotation results in 80° or 100° turn
- Rotation accuracy degrades at higher speeds

**Solutions:**
1. Verify `WHEELBASE` measurement (145mm center-to-center)
2. Ensure wheels have equal traction (not slipping)
3. Test on smooth, flat surface (avoid carpet)
4. Reduce rotation speed for better accuracy
5. Check that wheels are parallel and not toed-in/out

### I2C Communication Errors

**Symptoms:**
- Arduino serial monitor shows "I2C error: X"
- ESP32 shows "Warning: Arduino not connected!"
- Encoders work but commands not received

**Solutions:**
1. Check I2C wiring:
   - ESP32 GPIO21 (SDA) → Arduino A4
   - ESP32 GPIO22 (SCL) → Arduino A5
   - Common GND between ESP32 and Arduino
2. Verify I2C pull-up resistors (4.7kΩ or 10kΩ on SDA and SCL)
3. Check I2C address (Arduino should be 0x08)
4. Reduce I2C clock speed if experiencing errors:
   ```cpp
   Wire.setClock(50000);  // Try 50kHz instead of 100kHz
   ```

### Encoder Display Not Updating

**Symptoms:**
- Dashboard shows encoders stuck at same value
- WebSocket connected but no sensor updates

**Solutions:**
1. Check WebSocket connection status (should show "Verbunden")
2. Verify ESP32 is receiving data from Arduino via I2C
3. Check browser console for JavaScript errors (F12 Developer Tools)
4. Refresh the web page and reconnect

## Technical Details

### How Encoder-Based Movement Works

1. **Pin Change Interrupts (ISR):**
   - Arduino uses Pin Change Interrupts on Port C (PCINT8/PCINT9)
   - Interrupt fires on every state change of encoder pins
   - ISR counts rising edges (LOW → HIGH transitions)
   - Very fast execution (~1-2 microseconds per interrupt)

2. **Distance Calculation:**
   ```
   pulses_needed = distance_mm / MM_PER_PULSE
   MM_PER_PULSE = (π × 65mm) / 21 ≈ 9.72mm

   Example: 200mm forward
   pulses_needed = 200 / 9.72 ≈ 21 pulses
   ```

3. **Rotation Calculation:**
   ```
   arc_length = (π × wheelbase × degrees) / 360°
   pulses_needed = arc_length / MM_PER_PULSE

   Example: 90° turn with 145mm wheelbase
   arc_length = (π × 145 × 90) / 360 ≈ 114mm
   pulses_needed = 114 / 9.72 ≈ 12 pulses per wheel
   ```

4. **Movement Control Loop:**
   - Main loop polls encoder counts
   - Compares to target with tolerance (±2 pulses)
   - Stops motors when target reached or 5-second timeout expires
   - Non-blocking on ESP32 side (fire-and-forget commands)

### Performance Characteristics

- **Distance Accuracy:** ±10mm for movements > 100mm
- **Rotation Accuracy:** ±5° for rotations at medium speed
- **Maximum Speed:** 255 PWM (actual speed depends on battery voltage)
- **Recommended Speed:** 100-200 for best accuracy
- **Movement Timeout:** 5 seconds (safety feature)
- **I2C Update Rate:** 500ms between sensor polls
- **WebSocket Update Rate:** 1000ms for sensor broadcasts

### Safety Features

1. **Movement Timeout:** Motors automatically stop after 5 seconds
2. **Servo Auto-Detach:** Servo detaches after 2 seconds of inactivity (prevents jitter)
3. **Client Timeout:** ESP32 stops motors if no client activity for 15 seconds
4. **Speed Constraints:** All speeds constrained to -255 to +255 range
5. **Distance Limits:** Distance commands limited to ±1000mm (WebSocket level)
