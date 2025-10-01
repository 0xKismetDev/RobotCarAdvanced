import 'dart:async';
import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'package:web_socket_channel/web_socket_channel.dart';
import '../models/sensor_data.dart';

class WebSocketService extends ChangeNotifier {
  WebSocketChannel? _channel;
  Timer? _pingTimer;
  Timer? _reconnectTimer;

  String? _serverUrl;
  bool _isConnecting = false;
  bool _shouldReconnect = true;
  int _reconnectAttempts = 0;

  static const int _maxReconnectAttempts = 5;
  static const Duration _pingInterval = Duration(seconds: 5);
  static const Duration _reconnectDelay = Duration(seconds: 3);
  static const Duration _commandThrottle = Duration(milliseconds: 30);

  DateTime _lastCommandTime = DateTime.now();

  bool _isConnected = false;
  bool get isConnected => _isConnected;

  SensorData? _sensorData;
  SensorData? get sensorData => _sensorData;

  String _connectionStatus = 'Disconnected';
  String get connectionStatus => _connectionStatus;

  String _cameraIp = '192.168.4.1';
  String get cameraIp => _cameraIp;

  int _smoothedBattery = 0;
  int _lastStableBattery = 0;
  bool _motorsActive = false;
  DateTime _lastMotorStopTime = DateTime.now();
  static const Duration _batteryStabilizationDelay = Duration(seconds: 3);

  final _onConnectedController = StreamController<void>.broadcast();
  final _onDisconnectedController = StreamController<void>.broadcast();
  final _onErrorController = StreamController<String>.broadcast();
  final _onSensorDataController = StreamController<SensorData>.broadcast();

  Stream<void> get onConnected => _onConnectedController.stream;
  Stream<void> get onDisconnected => _onDisconnectedController.stream;
  Stream<String> get onError => _onErrorController.stream;
  Stream<SensorData> get onSensorData => _onSensorDataController.stream;

  void connect(String ipAddress, int port) {
    if (_isConnecting || _isConnected) {
      debugPrint('Already connected or connecting');
      return;
    }

    try {
      _serverUrl = 'ws://$ipAddress:$port';
      _isConnecting = true;
      _reconnectAttempts = 0;

      _updateConnectionStatus('Connecting...');

      _channel = WebSocketChannel.connect(Uri.parse(_serverUrl!));

      _channel!.stream.listen(
        (message) {
          _handleMessage(message.toString());
        },
        onError: (error) {
          debugPrint('WebSocket error: $error');
          _isConnecting = false;
          _onErrorController.add('Connection error: ${error.toString()}');
          _updateConnectionStatus('Error');
          _disconnect();
        },
        onDone: () {
          debugPrint('WebSocket closed');
          _isConnecting = false;
          _disconnect();

          if (_shouldReconnect && _reconnectAttempts < _maxReconnectAttempts) {
            _scheduleReconnect();
          }
        },
      );

      // Assume connected after successful connection
      // In production, wait for server handshake
      Future.delayed(const Duration(milliseconds: 500), () {
        if (_channel != null) {
          _onConnected();
        }
      });

    } catch (e) {
      debugPrint('Failed to connect: $e');
      _isConnecting = false;
      _onErrorController.add('Failed to connect: ${e.toString()}');
      _updateConnectionStatus('Error');
    }
  }

  void disconnect() {
    _shouldReconnect = false;
    _stopPing();
    _reconnectTimer?.cancel();

    _channel?.sink.close();
    _channel = null;

    _isConnected = false;
    _isConnecting = false;
    _updateConnectionStatus('Disconnected');
    _onDisconnectedController.add(null);
    notifyListeners();
  }

  void _onConnected() {
    debugPrint('WebSocket connected');
    _isConnecting = false;
    _isConnected = true;
    _reconnectAttempts = 0;

    _updateConnectionStatus('Connected');
    _onConnectedController.add(null);

    _startPing();
    _sendStatusRequest();
    notifyListeners();
  }

  void _disconnect() {
    _stopPing();
    _channel = null;
    _isConnected = false;
    _updateConnectionStatus('Disconnected');
    _onDisconnectedController.add(null);
    notifyListeners();
  }

  void _handleMessage(String message) {
    try {
      final json = jsonDecode(message);
      final type = json['type'];

      switch (type) {
        case 'sensor_data':
          _handleSensorData(json);
          break;
        case 'command_ack':
          // Command acknowledged
          break;
        case 'error':
          final errorMsg = json['message'];
          _onErrorController.add('Server error: $errorMsg');
          break;
        case 'pong':
          // Ping response - connection healthy
          break;
        default:
          debugPrint('Unknown message type: $type');
      }
    } catch (e) {
      debugPrint('Failed to parse message: $e');
    }
  }

  void _handleSensorData(Map<String, dynamic> json) {
    try {
      final newData = SensorData.fromJson(json);

      // Battery smoothing logic
      final rawBattery = newData.batteryPercent;

      // Check if motors are active based on motor status
      final motorStatus = newData.motorStatus.toUpperCase();
      final currentMotorsActive = motorStatus != 'STOPPED' && motorStatus != 'IDLE';

      if (currentMotorsActive != _motorsActive) {
        _motorsActive = currentMotorsActive;
        if (!_motorsActive) {
          _lastMotorStopTime = DateTime.now();
        }
      }

      // Update battery reading based on motor state
      if (!_motorsActive &&
          DateTime.now().difference(_lastMotorStopTime) > _batteryStabilizationDelay) {
        // Motors have been off for enough time, update stable battery
        _lastStableBattery = rawBattery;
        _smoothedBattery = rawBattery;
      } else if (!_motorsActive) {
        // Motors just stopped, gradually adjust to new reading
        _smoothedBattery = ((_smoothedBattery * 3 + rawBattery) / 4).round();
      }
      // When motors are active, keep the last stable battery reading

      // Create modified sensor data with smoothed battery
      _sensorData = SensorData(
        distance: newData.distance,
        servoPosition: newData.servoPosition,
        batteryPercent: _motorsActive ? _lastStableBattery : _smoothedBattery,
        motorStatus: newData.motorStatus,
        arduinoConnected: newData.arduinoConnected,
        timestamp: newData.timestamp,
      );

      _onSensorDataController.add(_sensorData!);

      // Update camera IP if provided
      if (json['camera_ip'] != null) {
        _cameraIp = json['camera_ip'];
      }

      notifyListeners();
    } catch (e) {
      debugPrint('Error parsing sensor data: $e');
    }
  }

  void sendMovementCommand(String direction, int speed) {
    if (!_throttleCommand()) return;

    final command = {
      'type': 'command',
      'action': 'move',
      'direction': direction,
      'speed': speed,
      'timestamp': DateTime.now().millisecondsSinceEpoch,
    };

    _sendMessage(jsonEncode(command));
  }

  void sendDifferentialCommand(int leftSpeed, int rightSpeed) {
    // Don't throttle stop commands - they need to be immediate
    final isStopCommand = leftSpeed == 0 && rightSpeed == 0;

    if (!isStopCommand && !_throttleCommand()) return;

    final command = {
      'type': 'command',
      'action': 'differential',
      'leftSpeed': leftSpeed,
      'rightSpeed': rightSpeed,
      'timestamp': DateTime.now().millisecondsSinceEpoch,
    };

    // Track motor state for battery smoothing
    final isMoving = leftSpeed != 0 || rightSpeed != 0;
    if (isMoving != _motorsActive) {
      _motorsActive = isMoving;
      if (!_motorsActive) {
        _lastMotorStopTime = DateTime.now();
      }
    }

    debugPrint('Sending differential: L=$leftSpeed, R=$rightSpeed');
    _sendMessage(jsonEncode(command));
  }

  void sendServoCommand(int angle) {
    final command = {
      'type': 'command',
      'action': 'servo',
      'angle': angle.clamp(0, 180),
      'timestamp': DateTime.now().millisecondsSinceEpoch,
    };

    _sendMessage(jsonEncode(command));
  }


  void sendFlashCommand(bool enabled) {
    final command = {
      'type': 'command',
      'action': 'flash',
      'state': enabled,
      'timestamp': DateTime.now().millisecondsSinceEpoch,
    };

    _sendMessage(jsonEncode(command));
  }

  void sendScanCommand() {
    final command = {
      'type': 'command',
      'action': 'scan',
      'timestamp': DateTime.now().millisecondsSinceEpoch,
    };

    _sendMessage(jsonEncode(command));
  }

  void _sendStatusRequest() {
    final request = {
      'type': 'status_request',
      'timestamp': DateTime.now().millisecondsSinceEpoch,
    };

    _sendMessage(jsonEncode(request));
  }

  void _sendPing() {
    final ping = {
      'type': 'ping',
      'timestamp': DateTime.now().millisecondsSinceEpoch,
    };

    _sendMessage(jsonEncode(ping));
  }

  void _sendMessage(String message) {
    if (_channel != null && _isConnected) {
      _channel!.sink.add(message);
      debugPrint('Sent: $message');
    } else {
      debugPrint('Cannot send message - not connected');
    }
  }

  bool _throttleCommand() {
    final now = DateTime.now();
    if (now.difference(_lastCommandTime) < _commandThrottle) {
      return false;
    }
    _lastCommandTime = now;
    return true;
  }

  void _startPing() {
    _stopPing();
    _pingTimer = Timer.periodic(_pingInterval, (_) {
      _sendPing();
    });
  }

  void _stopPing() {
    _pingTimer?.cancel();
    _pingTimer = null;
  }

  void _scheduleReconnect() {
    _reconnectAttempts++;
    _updateConnectionStatus(
      'Reconnecting... ($_reconnectAttempts/$_maxReconnectAttempts)'
    );

    _reconnectTimer = Timer(_reconnectDelay, () {
      if (_shouldReconnect && _serverUrl != null) {
        final uri = Uri.parse(_serverUrl!);
        connect(uri.host, uri.port);
      }
    });
  }

  void _updateConnectionStatus(String status) {
    _connectionStatus = status;
    notifyListeners();
  }

  @override
  void dispose() {
    disconnect();
    _onConnectedController.close();
    _onDisconnectedController.close();
    _onErrorController.close();
    _onSensorDataController.close();
    super.dispose();
  }
}