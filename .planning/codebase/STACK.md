# Technology Stack

**Analysis Date:** 2026-02-05

## Languages

**Primary:**
- C++ (Arduino dialect) - Motor control, encoder interrupts, I2C slave on Arduino UNO
- C++ (ESP-IDF variant) - WiFi/WebSocket server, I2C master, firmware on ESP32
- Dart 3.5.4+ - Flutter mobile app for iOS/Android control
- JavaScript - Block-based visual programming interface and Blockly integration
- Python 3.6+ - Local web server for Blockly interface

**Secondary:**
- HTML5 - Web interface markup for Blockly editor
- CSS3 - Styling for web interface

## Runtime

**Environment:**
- Arduino IDE 2.0+ - Firmware development and upload
- ESP32 (32-bit dual-core Xtensa processor, 240MHz) - Main wireless controller
- Arduino UNO (ATmega328P, 16MHz) - Motor and sensor control
- Python 3.6+ - Block interface server runtime
- Flutter SDK 3.5.4+ - Mobile app compilation and execution

**Package Manager:**
- Dart Pub - Flutter dependency management
  - Lockfile: `MobileControlApp/pubspec.lock` (present)
- Python pip - Python dependencies (implicit for http.server, socketserver - stdlib only)
- Arduino Library Manager - C++ library distribution

## Frameworks

**Core:**
- Flutter 3.5.4+ - Cross-platform mobile UI framework (iOS/Android)
- WebSocketsServer by Markus Sattler - ESP32 WebSocket server implementation
- ArduinoJson by Benoit Blanchon - JSON parsing on ESP32 for protocol messages
- Blockly (Google) - Block-based visual programming library for web interface
- Wire (Arduino built-in) - I2C communication library
- Servo (Arduino built-in) - Servo motor control

**Testing:**
- flutter_test (built-in) - Flutter widget and unit testing
- Basic manual testing via Serial Monitor (Arduino)

**Build/Dev:**
- Arduino IDE CLI - Firmware compilation and uploading
- Flutter build system - APK/IPA generation
- Python http.server - Built-in HTTP server (SimpleHTTPRequestHandler)

## Key Dependencies

**Critical - Firmware:**
- WebSocketsServer (ESP32) - Enables WebSocket protocol for real-time command delivery
- ArduinoJson (ESP32) - JSON message parsing from clients
- Wire (Arduino) - I2C I2C slave communication with ESP32 master

**Critical - Mobile App:**
- web_socket_channel 3.0.1 - WebSocket client for Flutter
- provider 6.1.2 - State management across screens
- shared_preferences 2.3.2 - Persistent storage of connection settings
- connectivity_plus 6.1.0 - WiFi network detection and status
- vibration 2.0.1 - Haptic feedback on control actions

**UI/UX - Mobile App:**
- flutter_joystick 0.2.2 - On-screen joystick widget for motor control
- percent_indicator 4.2.3 - Progress/battery level visualization
- google_fonts 6.2.1 - Typography customization
- flutter_svg 2.0.10+1 - SVG asset rendering
- cupertino_icons 1.0.8 - iOS-style iconography

**Infrastructure:**
- esp_wifi.h (ESP32 SDK) - WiFi power management
- esp_task_wdt.h (ESP32 SDK) - Watchdog timer for system stability
- esp_camera.h (ESP32 SDK, optional) - Camera module support for streaming

## Configuration

**Environment:**
- WiFi credentials hardcoded in firmware (AP SSID/password: `RobotCar-Blockly`, `RobotCar-AP`, `ESP32-CAM-STREAM` with password `12345678`)
- I2C address hardcoded: `0x08` for Arduino slave
- I2C clock: 400kHz (configurable in `Wire.setClock()`)
- WebSocket port: 81 (hardcoded across all ESP32 firmware variants)

**Build:**
- `.prettierrc` - Code formatter configuration (not found in repo)
- `analysis_options.yaml` (`MobileControlApp/`) - Dart linting rules
- `pubspec.yaml` (`MobileControlApp/`) - Dart package manifest with dependency versions
- Arduino IDE project files: `.ino` files are auto-compiled without separate config

**Hardware Pin Mapping (Arduino UNO):**
- Motors: ENA=5, IN1=2, IN2=4, ENB=6, IN3=7, IN4=8 (L298N motor driver)
- Encoders: A0 (PCINT8), A1 (PCINT9) - Pin change interrupts
- Ultrasonic: Trigger=9, Echo=10 (HC-SR04)
- Servo: A2
- I2C: SDA=A4, SCL=A5

**Hardware Pin Mapping (ESP32):**
- I2C SDA: GPIO21
- I2C SCL: GPIO22
- LED status: GPIO2
- Flash control: GPIO25 (camera flash, optional)

## Platform Requirements

**Development:**
- macOS (Apple Silicon or Intel) with Xcode Command Line Tools for iOS builds
- Linux or macOS for Python web server (BlockBasedLearning/WebInterface/)
- Arduino IDE 2.0+
- ESP32 Board Package (installed via Board Manager)
- Arduino boards installed (ATmega328P support)
- USB-to-UART adapter (CP2102 or similar) for firmware uploads

**Production:**
- Deployment target (Embedded):
  - ESP32 module (development board or built-in)
  - Arduino UNO microcontroller
  - L298N dual motor driver module
  - HC-SR04 ultrasonic sensor
  - SG90 servo motor
  - Optical wheel encoders (21 pulses/rotation)
  - USB power supply (5V for Arduino/motors, can power ESP32 from same)

**Deployment Target (Mobile):**
- iOS 11.0+ for Flutter app
- Android 7.0+ (API level 24+) for Flutter app
- WiFi connectivity required (802.11 b/g/n)

**Deployment Target (Web Interface):**
- Any modern browser supporting WebSockets (Chrome, Firefox, Safari, Edge)
- Python 3.6+ runtime on local machine
- Network connectivity to ESP32 (same WiFi network)

## Compilation & Flashing

**Arduino Firmware:**
```cpp
// Required libraries via Arduino Library Manager:
// - WebSocketsServer by Markus Sattler (v2.x)
// - ArduinoJson by Benoit Blanchon (v6.x or v7.x)
```

**ESP32 Firmware:**
Requires Arduino IDE with ESP32 board package installed. Compilation uses:
- Arduino Core for ESP32 (managed via Boards Manager)
- IDF v4.4 or v5.0 (underlying platform)

**Flutter App:**
```bash
flutter pub get          # Fetch dependencies
flutter build apk        # Android APK
flutter build ios        # iOS binary (requires macOS with Xcode)
```

**Web Interface:**
No compilation needed. Served directly by Python http.server module.

---

*Stack analysis: 2026-02-05*
