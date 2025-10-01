import 'dart:math';
import 'package:flutter/material.dart';

class CustomJoystick extends StatefulWidget {
  final Function(double x, double y, double strength) onMove;
  final VoidCallback onRelease;
  final double size;
  final double deadzone;

  const CustomJoystick({
    Key? key,
    required this.onMove,
    required this.onRelease,
    this.size = 240,
    this.deadzone = 0.15,
  }) : super(key: key);

  @override
  State<CustomJoystick> createState() => _CustomJoystickState();
}

class _CustomJoystickState extends State<CustomJoystick> {
  Offset _knobPosition = Offset.zero;
  bool _isDragging = false;
  double _strength = 0.0;

  double get _radius => widget.size / 2;
  double get _knobRadius => _radius / 3.5;

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final isDark = theme.brightness == Brightness.dark;

    return SizedBox(
      width: widget.size,
      height: widget.size,
      child: GestureDetector(
        onPanStart: _onPanStart,
        onPanUpdate: _onPanUpdate,
        onPanEnd: _onPanEnd,
        child: CustomPaint(
          painter: _JoystickPainter(
            knobPosition: _knobPosition,
            radius: _radius,
            knobRadius: _knobRadius,
            strength: _strength,
            deadzone: widget.deadzone,
            isDragging: _isDragging,
            isDark: isDark,
            accentColor: theme.colorScheme.primary,
          ),
        ),
      ),
    );
  }

  void _onPanStart(DragStartDetails details) {
    final localPos = details.localPosition;
    final center = Offset(_radius, _radius);
    final offset = localPos - center;

    if (offset.distance <= _radius) {
      setState(() {
        _isDragging = true;
        _updateKnobPosition(localPos);
      });
    }
  }

  void _onPanUpdate(DragUpdateDetails details) {
    if (_isDragging) {
      _updateKnobPosition(details.localPosition);
    }
  }

  void _onPanEnd(DragEndDetails details) {
    if (_isDragging) {
      setState(() {
        _isDragging = false;
        _knobPosition = Offset.zero;
        _strength = 0.0;
      });
      widget.onRelease();
    }
  }

  void _updateKnobPosition(Offset localPosition) {
    final center = Offset(_radius, _radius);
    final offset = localPosition - center;
    var distance = offset.distance;

    Offset newPosition;
    if (distance <= _radius) {
      newPosition = offset;
    } else {
      newPosition = offset / distance * _radius;
      distance = _radius;
    }

    final strength = distance / _radius;

    setState(() {
      _knobPosition = newPosition;
      _strength = strength;
    });

    if (strength > widget.deadzone) {
      final x = newPosition.dx / _radius;
      final y = -newPosition.dy / _radius; // Invert Y for standard coordinates
      widget.onMove(x, y, strength);
    }
  }
}

class _JoystickPainter extends CustomPainter {
  final Offset knobPosition;
  final double radius;
  final double knobRadius;
  final double strength;
  final double deadzone;
  final bool isDragging;
  final bool isDark;
  final Color accentColor;

  _JoystickPainter({
    required this.knobPosition,
    required this.radius,
    required this.knobRadius,
    required this.strength,
    required this.deadzone,
    required this.isDragging,
    required this.isDark,
    required this.accentColor,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(radius, radius);

    // Base circle
    final basePaint = Paint()
      ..color = isDark ? Colors.grey[900]! : Colors.grey[200]!
      ..style = PaintingStyle.fill;
    canvas.drawCircle(center, radius, basePaint);

    // Base border
    final borderPaint = Paint()
      ..color = isDark ? Colors.grey[700]! : Colors.grey[400]!
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2;
    canvas.drawCircle(center, radius, borderPaint);

    // Deadzone indicator
    final deadzonePaint = Paint()
      ..color = (isDark ? Colors.grey[800]! : Colors.grey[300]!).withOpacity(0.5)
      ..style = PaintingStyle.fill;
    canvas.drawCircle(center, radius * deadzone, deadzonePaint);

    // Center cross
    final crossPaint = Paint()
      ..color = accentColor.withOpacity(0.3)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2;
    const crossSize = 25.0;
    canvas.drawLine(
      Offset(center.dx - crossSize, center.dy),
      Offset(center.dx + crossSize, center.dy),
      crossPaint,
    );
    canvas.drawLine(
      Offset(center.dx, center.dy - crossSize),
      Offset(center.dx, center.dy + crossSize),
      crossPaint,
    );

    // Strength indicator ring
    if (strength > deadzone) {
      final strengthPaint = Paint()
        ..color = accentColor.withOpacity(strength)
        ..style = PaintingStyle.stroke
        ..strokeWidth = 6;
      final strengthRadius = knobRadius + 15 + (strength * 10);
      canvas.drawCircle(center + knobPosition, strengthRadius, strengthPaint);
    }

    // Knob shadow
    final shadowPaint = Paint()
      ..color = Colors.black.withOpacity(0.2)
      ..style = PaintingStyle.fill;
    canvas.drawCircle(
      center + knobPosition + const Offset(4, 4),
      knobRadius,
      shadowPaint,
    );

    // Knob
    final knobPaint = Paint()
      ..shader = RadialGradient(
        colors: [
          accentColor.withOpacity(0.8),
          accentColor,
        ],
        stops: const [0.0, 1.0],
      ).createShader(
        Rect.fromCircle(center: center + knobPosition, radius: knobRadius),
      );
    canvas.drawCircle(center + knobPosition, knobRadius, knobPaint);

    // Knob highlight
    if (isDragging) {
      final highlightPaint = Paint()
        ..color = Colors.white.withOpacity(0.3)
        ..style = PaintingStyle.fill;
      canvas.drawCircle(
        center + knobPosition - Offset(knobRadius / 3, knobRadius / 3),
        knobRadius / 3,
        highlightPaint,
      );
    }

    // Direction indicators
    _drawDirectionIndicators(canvas, center);
  }

  void _drawDirectionIndicators(Canvas canvas, Offset center) {
    final textPainter = TextPainter(
      textAlign: TextAlign.center,
      textDirection: TextDirection.ltr,
    );

    final indicatorRadius = radius + 30;

    // Simple text labels for main directions only
    final directions = [
      {'angle': -pi/2, 'text': 'FORWARD'},
      {'angle': pi/2, 'text': 'BACKWARD'},
      {'angle': 0, 'text': 'RIGHT'},
      {'angle': pi, 'text': 'LEFT'},
    ];

    for (final dir in directions) {
      final angle = dir['angle'] as double;
      final text = dir['text'] as String;
      final x = center.dx + cos(angle) * indicatorRadius;
      final y = center.dy + sin(angle) * indicatorRadius;

      // Check if this direction is active
      final isActive = _isMainDirectionActive(angle);

      textPainter.text = TextSpan(
        text: text,
        style: TextStyle(
          fontSize: isActive ? 14 : 12,
          fontWeight: isActive ? FontWeight.bold : FontWeight.normal,
          color: isActive
              ? accentColor
              : (isDark ? Colors.grey[600] : Colors.grey[400]),
        ),
      );

      textPainter.layout();
      textPainter.paint(
        canvas,
        Offset(x - textPainter.width / 2, y - textPainter.height / 2),
      );
    }
  }

  bool _isMainDirectionActive(double targetAngle) {
    if (strength <= deadzone) return false;

    final angle = atan2(knobPosition.dy, knobPosition.dx);

    // Calculate angular difference
    var diff = (angle - targetAngle).abs();
    if (diff > pi) diff = 2 * pi - diff;

    // Active if within 45 degrees (pi/4 radians)
    return diff < pi / 4;
  }

  @override
  bool shouldRepaint(covariant _JoystickPainter oldDelegate) {
    return oldDelegate.knobPosition != knobPosition ||
        oldDelegate.strength != strength ||
        oldDelegate.isDragging != isDragging;
  }
}