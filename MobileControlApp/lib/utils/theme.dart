import 'package:flutter/material.dart';
import 'package:google_fonts/google_fonts.dart';

class AppTheme {
  static ThemeData darkTheme = ThemeData(
    brightness: Brightness.dark,
    useMaterial3: true,
    colorScheme: const ColorScheme.dark(
      primary: Color(0xFF00E676),
      primaryContainer: Color(0xFF00C853),
      secondary: Color(0xFF64FFDA),
      secondaryContainer: Color(0xFF00BFA5),
      surface: Color(0xFF1A1A1A),
      surfaceContainer: Color(0xFF242424),
      surfaceContainerHigh: Color(0xFF2E2E2E),
      surfaceContainerHighest: Color(0xFF383838),
      onSurface: Color(0xFFE0E0E0),
      onSurfaceVariant: Color(0xFFA0A0A0),
      error: Color(0xFFFF5252),
      onError: Color(0xFFFFFFFF),
      errorContainer: Color(0xFFD32F2F),
      onErrorContainer: Color(0xFFFFFFFF),
      outline: Color(0xFF424242),
      outlineVariant: Color(0xFF616161),
    ),
    scaffoldBackgroundColor: const Color(0xFF0A0A0A),
    appBarTheme: AppBarTheme(
      backgroundColor: const Color(0xFF1A1A1A),
      elevation: 0,
      titleTextStyle: GoogleFonts.roboto(
        fontSize: 20,
        fontWeight: FontWeight.w500,
        color: const Color(0xFFE0E0E0),
      ),
      iconTheme: const IconThemeData(color: Color(0xFFE0E0E0)),
    ),
    cardTheme: CardThemeData(
      color: const Color(0xFF1A1A1A),
      elevation: 2,
      shape: RoundedRectangleBorder(
        borderRadius: BorderRadius.circular(12),
      ),
    ),
    elevatedButtonTheme: ElevatedButtonThemeData(
      style: ElevatedButton.styleFrom(
        backgroundColor: const Color(0xFF00E676),
        foregroundColor: const Color(0xFF000000),
        elevation: 2,
        padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 12),
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(8),
        ),
      ),
    ),
    outlinedButtonTheme: OutlinedButtonThemeData(
      style: OutlinedButton.styleFrom(
        foregroundColor: const Color(0xFF00E676),
        side: const BorderSide(color: Color(0xFF00E676)),
        padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 12),
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(8),
        ),
      ),
    ),
    sliderTheme: const SliderThemeData(
      activeTrackColor: Color(0xFF00E676),
      inactiveTrackColor: Color(0xFF424242),
      thumbColor: Color(0xFF00E676),
      overlayColor: Color(0x2900E676),
      valueIndicatorColor: Color(0xFF00E676),
    ),
    textTheme: GoogleFonts.robotoTextTheme(
      const TextTheme(
        displayLarge: TextStyle(color: Color(0xFFE0E0E0)),
        displayMedium: TextStyle(color: Color(0xFFE0E0E0)),
        displaySmall: TextStyle(color: Color(0xFFE0E0E0)),
        headlineLarge: TextStyle(color: Color(0xFFE0E0E0)),
        headlineMedium: TextStyle(color: Color(0xFFE0E0E0)),
        headlineSmall: TextStyle(color: Color(0xFFE0E0E0)),
        titleLarge: TextStyle(color: Color(0xFFE0E0E0)),
        titleMedium: TextStyle(color: Color(0xFFE0E0E0)),
        titleSmall: TextStyle(color: Color(0xFFE0E0E0)),
        bodyLarge: TextStyle(color: Color(0xFFE0E0E0)),
        bodyMedium: TextStyle(color: Color(0xFFE0E0E0)),
        bodySmall: TextStyle(color: Color(0xFFA0A0A0)),
        labelLarge: TextStyle(color: Color(0xFFE0E0E0)),
        labelMedium: TextStyle(color: Color(0xFFE0E0E0)),
        labelSmall: TextStyle(color: Color(0xFFA0A0A0)),
      ),
    ),
    inputDecorationTheme: InputDecorationTheme(
      filled: true,
      fillColor: const Color(0xFF242424),
      border: OutlineInputBorder(
        borderRadius: BorderRadius.circular(8),
        borderSide: const BorderSide(color: Color(0xFF424242)),
      ),
      enabledBorder: OutlineInputBorder(
        borderRadius: BorderRadius.circular(8),
        borderSide: const BorderSide(color: Color(0xFF424242)),
      ),
      focusedBorder: OutlineInputBorder(
        borderRadius: BorderRadius.circular(8),
        borderSide: const BorderSide(color: Color(0xFF00E676), width: 2),
      ),
      labelStyle: const TextStyle(color: Color(0xFFA0A0A0)),
      hintStyle: const TextStyle(color: Color(0xFF707070)),
    ),
    switchTheme: SwitchThemeData(
      thumbColor: WidgetStateProperty.resolveWith((states) {
        if (states.contains(WidgetState.selected)) {
          return const Color(0xFF00E676);
        }
        return const Color(0xFF616161);
      }),
      trackColor: WidgetStateProperty.resolveWith((states) {
        if (states.contains(WidgetState.selected)) {
          return const Color(0xFF00E676).withOpacity(0.5);
        }
        return const Color(0xFF424242);
      }),
    ),
    progressIndicatorTheme: const ProgressIndicatorThemeData(
      color: Color(0xFF00E676),
      linearTrackColor: Color(0xFF424242),
    ),
  );

  // Color utilities
  static Color getBatteryColor(int percent) {
    if (percent >= 60) return const Color(0xFF4CAF50); // Green
    if (percent >= 30) return const Color(0xFFFF9800); // Orange
    return const Color(0xFFF44336); // Red
  }

  static Color getDistanceColor(int distance) {
    if (distance < 20) return const Color(0xFFF44336); // Red - danger
    if (distance < 50) return const Color(0xFFFF9800); // Orange - warning
    return const Color(0xFF4CAF50); // Green - safe
  }
}