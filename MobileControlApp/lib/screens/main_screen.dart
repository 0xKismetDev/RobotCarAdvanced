import 'dart:async';
import 'dart:io';
import 'dart:math';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:percent_indicator/percent_indicator.dart';
import 'package:vibration/vibration.dart';
import '../services/websocket_service.dart';
import '../services/settings_service.dart';
import '../widgets/custom_joystick.dart';
import '../widgets/camera_view.dart';
import '../utils/theme.dart';
import 'settings_screen.dart';

class MainScreen extends StatefulWidget {
  const MainScreen({Key? key}) : super(key: key);

  @override
  State<MainScreen> createState() => _MainScreenState();
}

class _MainScreenState extends State<MainScreen> {
  late WebSocketService _wsService;
  late SettingsService _settings;

  int _currentLeftSpeed = 0;
  int _currentRightSpeed = 0;
  int _servoAngle = 90;
  bool _isTankTurning = false;
  Timer? _tankTurnTimer;
  bool _flashEnabled = false;
  Timer? _commandWatchdog;
  DateTime _lastCommandTime = DateTime.now();

  // motor control parameters
  static const int _motorDeadZone = 80;
  static const int _motorStartThreshold = 100;
  static const double _motorCompensationFactor = 0.6;

  String _direction = 'STOP';
  final TextEditingController _ipController = TextEditingController();

  @override
  void initState() {
    super.initState();
    _wsService = context.read<WebSocketService>();
    _settings = context.read<SettingsService>();

    _ipController.text = _settings.esp32Ip;

    _startCommandWatchdog();

    if (_settings.autoConnect) {
      WidgetsBinding.instance.addPostFrameCallback((_) {
        _connect();
      });
    }
  }

  void _startCommandWatchdog() {
    _commandWatchdog?.cancel();
    _commandWatchdog = Timer.periodic(const Duration(seconds: 1), (timer) {
      if (DateTime.now().difference(_lastCommandTime).inSeconds > 2 &&
          (_currentLeftSpeed != 0 || _currentRightSpeed != 0)) {
        debugPrint('Watchdog: Stopping stuck motors');
        _sendMotorCommand(0, 0);
      }
    });
  }

  @override
  void dispose() {
    _tankTurnTimer?.cancel();
    _commandWatchdog?.cancel();
    _ipController.dispose();
    // Ensure motors stop when leaving screen
    _wsService.sendDifferentialCommand(0, 0);
    super.dispose();
  }

  void _connect() {
    final ip = _ipController.text.trim();
    if (ip.isNotEmpty) {
      _settings.setEsp32Ip(ip);
      _wsService.connect(ip, _settings.esp32Port);
    }
  }

  void _disconnect() {
    _wsService.disconnect();
  }

  void _handleJoystickMove(double x, double y, double strength) {
    if (_isTankTurning) return;

    // If joystick is nearly centered, stop immediately
    if (strength < 0.1) {
      _sendMotorCommand(0, 0);
      return;
    }

    // Apply dead zone for joystick
    if (x.abs() < 0.08) x = 0;
    if (y.abs() < 0.08) y = 0;

    // Apply exponential curves for smoother control
    final curvedX = x * x * x; // Cubic for turning
    final curvedY = y * y.abs(); // Quadratic for forward/back

    // Progressive turning - less aggressive at low speeds
    final speedFactor = y.abs();
    final turnMultiplier = 0.25 + (speedFactor * 0.35); // Range: 0.25 to 0.6

    // Calculate base motor speeds
    final forward = curvedY * _settings.maxSpeed;
    final turn = curvedX * _settings.maxSpeed * turnMultiplier;

    // Mix for differential drive
    var leftMotor = (forward + turn).round();
    var rightMotor = (forward - turn).round();

    // Apply motor dead zone compensation
    leftMotor = _compensateMotorDeadZone(leftMotor);
    rightMotor = _compensateMotorDeadZone(rightMotor);

    // Constrain to valid range
    leftMotor = leftMotor.clamp(-_settings.maxSpeed, _settings.maxSpeed);
    rightMotor = rightMotor.clamp(-_settings.maxSpeed, _settings.maxSpeed);

    _sendMotorCommand(leftMotor, rightMotor);
  }

  int _compensateMotorDeadZone(int motorSpeed) {
    if (motorSpeed == 0) return 0;

    // Map the usable motor range to compensate for dead zone
    // Input: 0 to maxSpeed
    // Output: 0 or (motorStartThreshold to maxSpeed)

    final absSpeed = motorSpeed.abs();
    final sign = motorSpeed.sign;

    // If speed is too low, return 0 (complete stop)
    if (absSpeed < 20) return 0;

    // Map the speed from (20 to maxSpeed) to (motorStartThreshold to maxSpeed)
    // This ensures motors actually move when commanded
    final usableRange = _settings.maxSpeed - _motorStartThreshold;
    final inputRange = _settings.maxSpeed - 20;

    final mappedSpeed = ((absSpeed - 20) * usableRange / inputRange + _motorStartThreshold).round();

    return sign * mappedSpeed.clamp(_motorStartThreshold, _settings.maxSpeed);
  }

  void _handleJoystickRelease() {
    // Immediate stop - no smoothing on release
    _sendMotorCommand(0, 0);
  }

  void _sendMotorCommand(int left, int right) {
    if (_wsService.isConnected) {
      _lastCommandTime = DateTime.now();

      // Ensure complete stop when commanded to stop
      if (left == 0 && right == 0) {
        // Cancel watchdog timer on stop
        _commandWatchdog?.cancel();

        setState(() {
          _currentLeftSpeed = 0;
          _currentRightSpeed = 0;
          _direction = 'STOP';
        });
        // Send stop command multiple times to ensure it's received
        _wsService.sendDifferentialCommand(0, 0);
        Future.delayed(const Duration(milliseconds: 50), () {
          if (_currentLeftSpeed == 0 && _currentRightSpeed == 0) {
            _wsService.sendDifferentialCommand(0, 0);
          }
        });
      } else {
        // Start or restart watchdog timer for non-stop commands
        _startCommandWatchdog();

        setState(() {
          _currentLeftSpeed = left;
          _currentRightSpeed = right;
          _updateDirection();
        });
        _wsService.sendDifferentialCommand(left, right);
      }
    }
  }

  void _updateDirection() {
    if (_currentLeftSpeed == 0 && _currentRightSpeed == 0) {
      _direction = 'STOP';
    } else if (_currentLeftSpeed > 50 && _currentRightSpeed > 50) {
      _direction = 'FWD';
    } else if (_currentLeftSpeed < -50 && _currentRightSpeed < -50) {
      _direction = 'REV';
    } else if ((_currentLeftSpeed - _currentRightSpeed).abs() > 100) {
      _direction = _currentLeftSpeed > _currentRightSpeed ? 'RIGHT' : 'LEFT';
    } else {
      _direction = 'MOVE';
    }
  }

  void _performTankTurn(int degrees) async {
    if (!_wsService.isConnected || _isTankTurning) return;

    setState(() {
      _isTankTurning = true;
    });

    // Vibrate for feedback
    if (await Vibration.hasVibrator() ?? false) {
      Vibration.vibrate(duration: 50);
    }

    // Calculate turn duration (approximate)
    final duration = Duration(milliseconds: (degrees * 1000) ~/ 360);

    // Start tank turn
    _sendMotorCommand(255, -255);

    // Stop after duration
    _tankTurnTimer = Timer(duration, () {
      _sendMotorCommand(0, 0);
      setState(() {
        _isTankTurning = false;
      });
    });
  }

  void _updateServoAngle(int angle) {
    setState(() {
      _servoAngle = angle.clamp(0, 180);
    });
    _wsService.sendServoCommand(_servoAngle);
  }


  void _emergencyStop() async {
    // Force immediate stop
    _sendMotorCommand(0, 0);
    // Send multiple stop commands to ensure reception
    for (int i = 0; i < 3; i++) {
      await Future.delayed(const Duration(milliseconds: 50));
      _wsService.sendDifferentialCommand(0, 0);
    }

    // Vibrate for feedback
    if (await Vibration.hasVibrator() ?? false) {
      Vibration.vibrate(duration: 200);
    }

    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(
        content: Text('EMERGENCY STOP!'),
        backgroundColor: Colors.red,
        duration: Duration(seconds: 1),
      ),
    );
  }

  void _toggleFlash() {
    // Toggle flash state immediately for responsive UI
    setState(() {
      _flashEnabled = !_flashEnabled;
    });

    // Send flash command through WebSocket to ESP32 master
    // The master will forward it to the ESP32-CAM
    _wsService.sendFlashCommand(_flashEnabled);

    debugPrint('Flash toggled to: ${_flashEnabled ? "ON" : "OFF"}');
  }

  void _toggleSpeed() {
    final newSpeed = _settings.maxSpeed == 255 ? 150 : 255;
    _settings.setMaxSpeed(newSpeed);

    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text('Speed: $newSpeed'),
        duration: const Duration(seconds: 1),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Theme.of(context).scaffoldBackgroundColor,
      appBar: AppBar(
        title: Row(
          children: [
            const Icon(Icons.smart_toy, size: 20),
            const SizedBox(width: 8),
            const Text('Robot Car', style: TextStyle(fontSize: 16)),
            const SizedBox(width: 16),
            // Connection status
            Consumer<WebSocketService>(
              builder: (context, ws, _) {
                return Row(
                  children: [
                    Icon(
                      Icons.circle,
                      size: 10,
                      color: ws.isConnected ? Colors.green : Colors.red,
                    ),
                    const SizedBox(width: 4),
                    Text(
                      ws.isConnected ? 'Connected' : 'Disconnected',
                      style: const TextStyle(fontSize: 12),
                    ),
                  ],
                );
              },
            ),
          ],
        ),
        actions: [
          // Distance indicator
          Consumer<WebSocketService>(
            builder: (context, ws, _) {
              final distance = ws.sensorData?.distance ?? -1;
              return Padding(
                padding: const EdgeInsets.symmetric(horizontal: 8),
                child: Row(
                  children: [
                    Icon(
                      Icons.sensors,
                      size: 18,
                      color: distance >= 0 ? AppTheme.getDistanceColor(distance) : Colors.grey,
                    ),
                    const SizedBox(width: 4),
                    Text(
                      distance >= 0 ? '${distance}cm' : '--',
                      style: TextStyle(
                        fontSize: 12,
                        color: distance >= 0 ? AppTheme.getDistanceColor(distance) : Colors.grey,
                      ),
                    ),
                  ],
                ),
              );
            },
          ),
          // Battery indicator
          Consumer<WebSocketService>(
            builder: (context, ws, _) {
              final battery = ws.sensorData?.batteryPercent ?? 0;
              return Padding(
                padding: const EdgeInsets.symmetric(horizontal: 8),
                child: Row(
                  children: [
                    Icon(
                      Icons.battery_full,
                      size: 18,
                      color: AppTheme.getBatteryColor(battery),
                    ),
                    const SizedBox(width: 4),
                    Text(
                      '$battery%',
                      style: TextStyle(
                        fontSize: 12,
                        color: AppTheme.getBatteryColor(battery),
                      ),
                    ),
                  ],
                ),
              );
            },
          ),
          // Settings button
          IconButton(
            icon: const Icon(Icons.settings, size: 20),
            onPressed: () {
              Navigator.push(
                context,
                MaterialPageRoute(
                  builder: (context) => const SettingsScreen(),
                ),
              );
            },
          ),
        ],
      ),
      body: _buildLandscapeLayout(),
    );
  }


  Widget _buildLandscapeLayout() {
    return Padding(
      padding: const EdgeInsets.all(4),
      child: Row(
        children: [
          // Left side - Camera (takes most space)
          Expanded(
            flex: 5,
            child: _buildCameraCard(),
          ),
          const SizedBox(width: 4),
          // Middle - Joystick
          SizedBox(
            width: 240,
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                _buildJoystickWithDirection(),
              ],
            ),
          ),
          const SizedBox(width: 4),
          // Right side - Compact controls
          SizedBox(
            width: 180,
            child: SingleChildScrollView(
              child: Column(
                children: [
                  _buildCompactConnectionStatus(),
                  const SizedBox(height: 4),
                  _buildCompactControls(),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildConnectionCard() {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                const Icon(Icons.wifi),
                const SizedBox(width: 8),
                Text(
                  'Connection',
                  style: Theme.of(context).textTheme.titleLarge,
                ),
                const Spacer(),
                Consumer<WebSocketService>(
                  builder: (context, ws, _) {
                    return Container(
                      width: 8,
                      height: 8,
                      decoration: BoxDecoration(
                        shape: BoxShape.circle,
                        color: ws.isConnected ? Colors.green : Colors.red,
                      ),
                    );
                  },
                ),
              ],
            ),
            const SizedBox(height: 16),
            Row(
              children: [
                Expanded(
                  child: TextField(
                    controller: _ipController,
                    decoration: const InputDecoration(
                      labelText: 'Robot IP Address',
                      hintText: '192.168.4.1',
                    ),
                  ),
                ),
                const SizedBox(width: 16),
                Consumer<WebSocketService>(
                  builder: (context, ws, _) {
                    return ElevatedButton(
                      onPressed: ws.isConnected ? _disconnect : _connect,
                      child: Text(ws.isConnected ? 'Disconnect' : 'Connect'),
                    );
                  },
                ),
              ],
            ),
            const SizedBox(height: 8),
            Consumer<WebSocketService>(
              builder: (context, ws, _) {
                return Text(
                  ws.connectionStatus,
                  style: Theme.of(context).textTheme.bodySmall,
                );
              },
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildSensorCard() {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                const Icon(Icons.sensors),
                const SizedBox(width: 8),
                Text(
                  'Sensor Data',
                  style: Theme.of(context).textTheme.titleLarge,
                ),
              ],
            ),
            const SizedBox(height: 16),
            Consumer<WebSocketService>(
              builder: (context, ws, _) {
                final data = ws.sensorData;
                return Row(
                  mainAxisAlignment: MainAxisAlignment.spaceAround,
                  children: [
                    _buildSensorItem(
                      'Distance',
                      data != null ? '${data.distance} cm' : '-- cm',
                      data != null
                          ? AppTheme.getDistanceColor(data.distance)
                          : null,
                    ),
                    _buildSensorItem(
                      'Servo',
                      '$_servoAngle°',
                    ),
                    _buildSensorItem(
                      'Motor',
                      data?.motorStatus ?? 'STOPPED',
                    ),
                    _buildSensorItem(
                      'Battery',
                      data != null ? '${data.batteryPercent}%' : '--%',
                      data != null
                          ? AppTheme.getBatteryColor(data.batteryPercent)
                          : null,
                    ),
                  ],
                );
              },
            ),
            const SizedBox(height: 16),
            Consumer<WebSocketService>(
              builder: (context, ws, _) {
                final battery = ws.sensorData?.batteryPercent ?? 0;
                return LinearPercentIndicator(
                  lineHeight: 8.0,
                  percent: battery / 100.0,
                  backgroundColor: Theme.of(context).colorScheme.surfaceContainer,
                  progressColor: AppTheme.getBatteryColor(battery),
                  barRadius: const Radius.circular(4),
                );
              },
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildSensorItem(String label, String value, [Color? color]) {
    return Column(
      children: [
        Text(
          label,
          style: Theme.of(context).textTheme.bodySmall,
        ),
        const SizedBox(height: 4),
        Text(
          value,
          style: Theme.of(context).textTheme.titleMedium?.copyWith(
                color: color,
                fontWeight: FontWeight.bold,
              ),
        ),
      ],
    );
  }

  Widget _buildControlsCard() {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                const Icon(Icons.gamepad),
                const SizedBox(width: 8),
                Text(
                  'Controls',
                  style: Theme.of(context).textTheme.titleLarge,
                ),
              ],
            ),
            const SizedBox(height: 16),
            // Servo control
            Text(
              'Camera Servo',
              style: Theme.of(context).textTheme.titleSmall,
            ),
            const SizedBox(height: 8),
            Row(
              children: [
                const Icon(Icons.camera_alt, size: 20),
                Expanded(
                  child: Slider(
                    value: _servoAngle.toDouble(),
                    min: 0,
                    max: 180,
                    divisions: 36,
                    label: '$_servoAngle°',
                    onChanged: (value) {
                      _updateServoAngle(value.round());
                    },
                    onChangeEnd: (value) {
                      if (_settings.servoSnapBack) {
                        Future.delayed(const Duration(milliseconds: 500), () {
                          _updateServoAngle(90);
                        });
                      }
                    },
                  ),
                ),
                SizedBox(
                  width: 50,
                  child: Text(
                    '$_servoAngle°',
                    textAlign: TextAlign.center,
                    style: Theme.of(context).textTheme.bodyMedium,
                  ),
                ),
              ],
            ),
            const SizedBox(height: 16),
            // Tank turn buttons
            Text(
              'Tank Turn',
              style: Theme.of(context).textTheme.titleSmall,
            ),
            const SizedBox(height: 8),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: [
                OutlinedButton(
                  onPressed: _wsService.isConnected
                      ? () => _performTankTurn(90)
                      : null,
                  child: const Text('90°'),
                ),
                OutlinedButton(
                  onPressed: _wsService.isConnected
                      ? () => _performTankTurn(180)
                      : null,
                  child: const Text('180°'),
                ),
                OutlinedButton(
                  onPressed: _wsService.isConnected
                      ? () => _performTankTurn(360)
                      : null,
                  child: const Text('360°'),
                ),
              ],
            ),
            const SizedBox(height: 16),
            // Action buttons
            Row(
              children: [
                Expanded(
                  child: ElevatedButton.icon(
                    onPressed: _wsService.isConnected
                        ? () => _wsService.sendScanCommand()
                        : null,
                    icon: const Icon(Icons.radar),
                    label: const Text('Scan'),
                  ),
                ),
                const SizedBox(width: 8),
                Expanded(
                  child: ElevatedButton.icon(
                    onPressed: _wsService.isConnected ? _toggleSpeed : null,
                    icon: const Icon(Icons.speed),
                    label: Consumer<SettingsService>(
                      builder: (context, settings, _) {
                        return Text('${settings.maxSpeed}');
                      },
                    ),
                  ),
                ),
                const SizedBox(width: 8),
                Expanded(
                  child: ElevatedButton.icon(
                    onPressed: _emergencyStop,
                    icon: const Icon(Icons.stop),
                    label: const Text('STOP'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.red,
                      foregroundColor: Colors.white,
                    ),
                  ),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildJoystickWithDirection() {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(8),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Text(
              _direction,
              style: TextStyle(
                fontSize: 14,
                color: _getDirectionColor(),
                fontWeight: FontWeight.bold,
              ),
            ),
            const SizedBox(height: 4),
            Consumer<SettingsService>(
              builder: (context, settings, _) {
                return CustomJoystick(
                  size: 200,
                  deadzone: settings.joystickDeadzone,
                  onMove: _handleJoystickMove,
                  onRelease: _handleJoystickRelease,
                );
              },
            ),
            const SizedBox(height: 4),
            Text(
              'L: $_currentLeftSpeed | R: $_currentRightSpeed',
              style: const TextStyle(fontSize: 10),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildJoystickCard() {
    return _buildJoystickWithDirection();
  }

  Color _getDirectionColor() {
    switch (_direction) {
      case 'STOP':
        return Colors.red;
      case 'FWD':
        return Colors.green;
      case 'REV':
        return Colors.orange;
      default:
        return Theme.of(context).colorScheme.primary;
    }
  }

  Widget _buildCameraCard() {
    return Consumer<WebSocketService>(
      builder: (context, ws, _) {
        return CameraView(
          cameraIp: ws.cameraIp,
          isConnected: ws.isConnected,
        );
      },
    );
  }

  Widget _buildCompactConnectionStatus() {
    return Consumer<WebSocketService>(
      builder: (context, ws, _) {
        return Card(
          child: Padding(
            padding: const EdgeInsets.all(8),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                SizedBox(
                  height: 28,
                  child: TextField(
                    controller: _ipController,
                    decoration: const InputDecoration(
                      hintText: 'IP Address',
                      contentPadding: EdgeInsets.symmetric(horizontal: 8, vertical: 2),
                      isDense: true,
                      border: OutlineInputBorder(),
                    ),
                    style: const TextStyle(fontSize: 11),
                  ),
                ),
                const SizedBox(height: 4),
                SizedBox(
                  height: 28,
                  width: double.infinity,
                  child: ElevatedButton(
                    onPressed: ws.isConnected ? _disconnect : _connect,
                    style: ElevatedButton.styleFrom(
                      padding: EdgeInsets.zero,
                    ),
                    child: Text(
                      ws.isConnected ? 'Disconnect' : 'Connect',
                      style: const TextStyle(fontSize: 11),
                    ),
                  ),
                ),
              ],
            ),
          ),
        );
      },
    );
  }

  Widget _buildCompactSensorDisplay() {
    return const SizedBox.shrink(); // Removed since indicators are now in app bar
  }

  Widget _buildCompactControls() {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(8),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            // Speed control
            Row(
              children: [
                const Icon(Icons.speed, size: 14),
                const SizedBox(width: 4),
                Expanded(
                  child: Consumer<SettingsService>(
                    builder: (context, settings, _) {
                      return SliderTheme(
                        data: SliderTheme.of(context).copyWith(
                          trackHeight: 2,
                          thumbShape: const RoundSliderThumbShape(enabledThumbRadius: 6),
                          overlayShape: const RoundSliderOverlayShape(overlayRadius: 12),
                        ),
                        child: Slider(
                          value: settings.maxSpeed.toDouble(),
                          min: 100,
                          max: 255,
                          onChanged: (value) {
                            settings.setMaxSpeed(value.round());
                          },
                        ),
                      );
                    },
                  ),
                ),
                Consumer<SettingsService>(
                  builder: (context, settings, _) {
                    return SizedBox(
                      width: 25,
                      child: Text(
                        '${settings.maxSpeed}',
                        style: const TextStyle(fontSize: 10),
                      ),
                    );
                  },
                ),
              ],
            ),
            // Servo control (horizontal scan)
            Row(
              children: [
                const Icon(Icons.rotate_right, size: 14),
                const SizedBox(width: 4),
                Expanded(
                  child: SliderTheme(
                    data: SliderTheme.of(context).copyWith(
                      trackHeight: 2,
                      thumbShape: const RoundSliderThumbShape(enabledThumbRadius: 6),
                      overlayShape: const RoundSliderOverlayShape(overlayRadius: 12),
                    ),
                    child: Slider(
                      value: _servoAngle.toDouble(),
                      min: 0,
                      max: 180,
                      onChanged: (value) {
                        _updateServoAngle(value.round());
                      },
                      onChangeEnd: (value) {
                        // Snap back to center if enabled in settings
                        if (_settings.servoSnapBack) {
                          Future.delayed(const Duration(milliseconds: 500), () {
                            _updateServoAngle(90);
                          });
                        }
                      },
                    ),
                  ),
                ),
                SizedBox(
                  width: 25,
                  child: Text(
                    '$_servoAngle°',
                    style: const TextStyle(fontSize: 10),
                  ),
                ),
              ],
            ),
            const SizedBox(height: 4),
            // First row: Flash and Scan
            Row(
              children: [
                Expanded(
                  child: SizedBox(
                    height: 28,
                    child: ElevatedButton.icon(
                      onPressed: _toggleFlash,
                      icon: Icon(
                        _flashEnabled ? Icons.flash_on : Icons.flash_off,
                        size: 14,
                      ),
                      label: Text(
                        _flashEnabled ? 'ON' : 'OFF',
                        style: const TextStyle(fontSize: 10),
                      ),
                      style: ElevatedButton.styleFrom(
                        padding: EdgeInsets.zero,
                        backgroundColor: _flashEnabled ? Colors.amber : Theme.of(context).colorScheme.surface,
                        foregroundColor: _flashEnabled ? Colors.black : Theme.of(context).colorScheme.onSurface,
                      ),
                    ),
                  ),
                ),
                const SizedBox(width: 4),
                Expanded(
                  child: SizedBox(
                    height: 28,
                    child: ElevatedButton(
                      onPressed: () => _wsService.sendScanCommand(),
                      style: ElevatedButton.styleFrom(
                        padding: EdgeInsets.zero,
                        backgroundColor: Theme.of(context).colorScheme.secondary,
                      ),
                      child: const Text(
                        'SCAN',
                        style: TextStyle(fontSize: 11),
                      ),
                    ),
                  ),
                ),
              ],
            ),
            const SizedBox(height: 4),
            // Second row: Emergency Stop (full width)
            SizedBox(
              height: 32,
              width: double.infinity,
              child: ElevatedButton(
                onPressed: _emergencyStop,
                style: ElevatedButton.styleFrom(
                  padding: EdgeInsets.zero,
                  backgroundColor: Colors.red,
                ),
                child: const Text(
                  'EMERGENCY STOP',
                  style: TextStyle(fontSize: 12, fontWeight: FontWeight.bold),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}