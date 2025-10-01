import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';

class SettingsService extends ChangeNotifier {
  static const String _keyEsp32Ip = 'esp32_ip';
  static const String _keyEsp32Port = 'esp32_port';
  static const String _keyAutoConnect = 'auto_connect';
  static const String _keyMaxSpeed = 'max_speed';
  static const String _keyJoystickDeadzone = 'joystick_deadzone';
  static const String _keyServoStep = 'servo_step';
  static const String _keyServoSnapBack = 'servo_snap_back';

  // Default values
  static const String defaultIp = '192.168.4.1';
  static const int defaultPort = 81;
  static const int defaultMaxSpeed = 255;
  static const double defaultDeadzone = 0.15;
  static const int defaultServoStep = 5;
  static const bool defaultAutoConnect = false;
  static const bool defaultServoSnapBack = true;

  late SharedPreferences _prefs;
  bool _isInitialized = false;

  // Settings values
  String _esp32Ip = defaultIp;
  int _esp32Port = defaultPort;
  bool _autoConnect = defaultAutoConnect;
  int _maxSpeed = defaultMaxSpeed;
  double _joystickDeadzone = defaultDeadzone;
  int _servoStep = defaultServoStep;
  bool _servoSnapBack = defaultServoSnapBack;

  // Getters
  String get esp32Ip => _esp32Ip;
  int get esp32Port => _esp32Port;
  bool get autoConnect => _autoConnect;
  int get maxSpeed => _maxSpeed;
  double get joystickDeadzone => _joystickDeadzone;
  int get servoStep => _servoStep;
  bool get servoSnapBack => _servoSnapBack;
  bool get isInitialized => _isInitialized;

  Future<void> init() async {
    _prefs = await SharedPreferences.getInstance();
    await loadSettings();
    _isInitialized = true;
    notifyListeners();
  }

  Future<void> loadSettings() async {
    _esp32Ip = _prefs.getString(_keyEsp32Ip) ?? defaultIp;
    _esp32Port = _prefs.getInt(_keyEsp32Port) ?? defaultPort;
    _autoConnect = _prefs.getBool(_keyAutoConnect) ?? defaultAutoConnect;
    _maxSpeed = _prefs.getInt(_keyMaxSpeed) ?? defaultMaxSpeed;
    _joystickDeadzone = _prefs.getDouble(_keyJoystickDeadzone) ?? defaultDeadzone;
    _servoStep = _prefs.getInt(_keyServoStep) ?? defaultServoStep;
    _servoSnapBack = _prefs.getBool(_keyServoSnapBack) ?? defaultServoSnapBack;
    notifyListeners();
  }

  Future<void> setEsp32Ip(String ip) async {
    _esp32Ip = ip;
    await _prefs.setString(_keyEsp32Ip, ip);
    notifyListeners();
  }

  Future<void> setEsp32Port(int port) async {
    _esp32Port = port;
    await _prefs.setInt(_keyEsp32Port, port);
    notifyListeners();
  }

  Future<void> setAutoConnect(bool value) async {
    _autoConnect = value;
    await _prefs.setBool(_keyAutoConnect, value);
    notifyListeners();
  }

  Future<void> setMaxSpeed(int speed) async {
    _maxSpeed = speed;
    await _prefs.setInt(_keyMaxSpeed, speed);
    notifyListeners();
  }

  Future<void> setJoystickDeadzone(double deadzone) async {
    _joystickDeadzone = deadzone;
    await _prefs.setDouble(_keyJoystickDeadzone, deadzone);
    notifyListeners();
  }

  Future<void> setServoStep(int step) async {
    _servoStep = step;
    await _prefs.setInt(_keyServoStep, step);
    notifyListeners();
  }

  Future<void> setServoSnapBack(bool value) async {
    _servoSnapBack = value;
    await _prefs.setBool(_keyServoSnapBack, value);
    notifyListeners();
  }

  Future<void> resetToDefaults() async {
    _esp32Ip = defaultIp;
    _esp32Port = defaultPort;
    _autoConnect = defaultAutoConnect;
    _maxSpeed = defaultMaxSpeed;
    _joystickDeadzone = defaultDeadzone;
    _servoStep = defaultServoStep;
    _servoSnapBack = defaultServoSnapBack;

    await _prefs.setString(_keyEsp32Ip, defaultIp);
    await _prefs.setInt(_keyEsp32Port, defaultPort);
    await _prefs.setBool(_keyAutoConnect, defaultAutoConnect);
    await _prefs.setInt(_keyMaxSpeed, defaultMaxSpeed);
    await _prefs.setDouble(_keyJoystickDeadzone, defaultDeadzone);
    await _prefs.setInt(_keyServoStep, defaultServoStep);
    await _prefs.setBool(_keyServoSnapBack, defaultServoSnapBack);

    notifyListeners();
  }
}