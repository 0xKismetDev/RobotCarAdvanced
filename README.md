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
A2   | Servo Motor Control  | SG90 Servo
A4   | SDA (I2C Data)       | ESP32 Communication
A5   | SCL (I2C Clock)      | ESP32 Communication
```

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

JSON over WebSocket (port 81):

```json
// Movement
{"command": "move", "direction": "forward|backward", "speed": 0-255}
{"command": "turn", "direction": "left|right", "speed": 0-255}
{"command": "stop"}

// Sensors
{"command": "getDistance"}
{"command": "servo", "angle": 0-180}

// System
{"command": "status"}
{"command": "reset"}
```
