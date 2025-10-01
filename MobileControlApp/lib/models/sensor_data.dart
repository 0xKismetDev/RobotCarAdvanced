class SensorData {
  final int distance;
  final int servoPosition;
  final int batteryPercent;
  final String motorStatus;
  final int timestamp;
  final bool arduinoConnected;
  final String? lastCommand;

  SensorData({
    required this.distance,
    required this.servoPosition,
    required this.batteryPercent,
    required this.motorStatus,
    required this.timestamp,
    required this.arduinoConnected,
    this.lastCommand,
  });

  factory SensorData.fromJson(Map<String, dynamic> json) {
    return SensorData(
      distance: json['distance'] ?? -1,
      servoPosition: json['servo_position'] ?? 90,
      batteryPercent: json['battery_percent'] ?? 0,
      motorStatus: json['motor_status'] ?? 'STOPPED',
      timestamp: json['timestamp'] ?? 0,
      arduinoConnected: json['arduino_connected'] ?? false,
      lastCommand: json['last_command'],
    );
  }

  @override
  String toString() {
    return 'SensorData(distance: $distance, servoPosition: $servoPosition, '
        'battery: $batteryPercent%, motorStatus: $motorStatus, '
        'arduinoConnected: $arduinoConnected)';
  }
}