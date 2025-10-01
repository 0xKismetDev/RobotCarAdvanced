import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import '../services/settings_service.dart';

class SettingsScreen extends StatefulWidget {
  const SettingsScreen({Key? key}) : super(key: key);

  @override
  State<SettingsScreen> createState() => _SettingsScreenState();
}

class _SettingsScreenState extends State<SettingsScreen> {
  late TextEditingController _ipController;
  late TextEditingController _portController;

  @override
  void initState() {
    super.initState();
    final settings = context.read<SettingsService>();
    _ipController = TextEditingController(text: settings.esp32Ip);
    _portController = TextEditingController(text: settings.esp32Port.toString());
  }

  @override
  void dispose() {
    _ipController.dispose();
    _portController.dispose();
    super.dispose();
  }

  void _saveSettings() {
    final settings = context.read<SettingsService>();

    final ip = _ipController.text.trim();
    final port = int.tryParse(_portController.text.trim()) ?? SettingsService.defaultPort;

    settings.setEsp32Ip(ip);
    settings.setEsp32Port(port);

    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(
        content: Text('Settings saved'),
        duration: Duration(seconds: 1),
      ),
    );

    Navigator.pop(context);
  }

  void _resetToDefaults() async {
    final settings = context.read<SettingsService>();
    await settings.resetToDefaults();

    setState(() {
      _ipController.text = settings.esp32Ip;
      _portController.text = settings.esp32Port.toString();
    });

    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(
        content: Text('Reset to defaults'),
        duration: Duration(seconds: 1),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Settings'),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back),
          onPressed: () {
            Navigator.pop(context);
          },
        ),
        actions: [
          TextButton(
            onPressed: _saveSettings,
            child: const Text('SAVE'),
          ),
        ],
      ),
      body: Consumer<SettingsService>(
        builder: (context, settings, _) {
          return SingleChildScrollView(
            padding: const EdgeInsets.all(16),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                // Connection Settings
                _buildSectionHeader('Connection Settings'),
                Card(
                  child: Padding(
                    padding: const EdgeInsets.all(16),
                    child: Column(
                      children: [
                        TextField(
                          controller: _ipController,
                          decoration: const InputDecoration(
                            labelText: 'ESP32 IP Address',
                            hintText: '192.168.4.1',
                            prefixIcon: Icon(Icons.wifi),
                          ),
                          keyboardType: TextInputType.url,
                        ),
                        const SizedBox(height: 16),
                        TextField(
                          controller: _portController,
                          decoration: const InputDecoration(
                            labelText: 'WebSocket Port',
                            hintText: '81',
                            prefixIcon: Icon(Icons.network_check),
                          ),
                          keyboardType: TextInputType.number,
                          inputFormatters: [
                            FilteringTextInputFormatter.digitsOnly,
                          ],
                        ),
                        const SizedBox(height: 16),
                        SwitchListTile(
                          title: const Text('Auto Connect'),
                          subtitle: const Text('Connect automatically on app start'),
                          value: settings.autoConnect,
                          onChanged: (value) {
                            settings.setAutoConnect(value);
                          },
                        ),
                      ],
                    ),
                  ),
                ),
                const SizedBox(height: 24),

                // Control Settings
                _buildSectionHeader('Control Settings'),
                Card(
                  child: Padding(
                    padding: const EdgeInsets.all(16),
                    child: Column(
                      children: [
                        // Max Speed
                        ListTile(
                          title: const Text('Max Speed'),
                          subtitle: Text('${settings.maxSpeed}'),
                          trailing: SizedBox(
                            width: 200,
                            child: Slider(
                              value: settings.maxSpeed.toDouble(),
                              min: 50,
                              max: 255,
                              divisions: 41,
                              label: settings.maxSpeed.toString(),
                              onChanged: (value) {
                                settings.setMaxSpeed(value.round());
                              },
                            ),
                          ),
                        ),
                        const Divider(),

                        // Joystick Deadzone
                        ListTile(
                          title: const Text('Joystick Deadzone'),
                          subtitle: Text('${(settings.joystickDeadzone * 100).toStringAsFixed(0)}%'),
                          trailing: SizedBox(
                            width: 200,
                            child: Slider(
                              value: settings.joystickDeadzone,
                              min: 0.0,
                              max: 0.5,
                              divisions: 50,
                              label: '${(settings.joystickDeadzone * 100).toStringAsFixed(0)}%',
                              onChanged: (value) {
                                settings.setJoystickDeadzone(value);
                              },
                            ),
                          ),
                        ),
                        const Divider(),

                        // Servo Step
                        ListTile(
                          title: const Text('Servo Step Size'),
                          subtitle: Text('${settings.servoStep}°'),
                          trailing: SizedBox(
                            width: 200,
                            child: Slider(
                              value: settings.servoStep.toDouble(),
                              min: 1,
                              max: 30,
                              divisions: 29,
                              label: '${settings.servoStep}°',
                              onChanged: (value) {
                                settings.setServoStep(value.round());
                              },
                            ),
                          ),
                        ),
                        const Divider(),

                        // Servo Snap Back
                        SwitchListTile(
                          title: const Text('Servo Snap Back'),
                          subtitle: const Text('Return servo to center after release'),
                          value: settings.servoSnapBack,
                          onChanged: (value) {
                            settings.setServoSnapBack(value);
                          },
                        ),
                      ],
                    ),
                  ),
                ),
                const SizedBox(height: 24),

                // Info Section
                _buildSectionHeader('About'),
                Card(
                  child: Padding(
                    padding: const EdgeInsets.all(16),
                    child: Column(
                      children: [
                        ListTile(
                          leading: const Icon(Icons.info_outline),
                          title: const Text('Version'),
                          subtitle: const Text('1.0.0'),
                        ),
                        ListTile(
                          leading: const Icon(Icons.code),
                          title: const Text('Built with Flutter'),
                          subtitle: const Text('Modern cross-platform framework'),
                        ),
                        ListTile(
                          leading: const Icon(Icons.memory),
                          title: const Text('ESP32 Firmware'),
                          subtitle: const Text('WebSocket server on port 81'),
                        ),
                      ],
                    ),
                  ),
                ),
                const SizedBox(height: 24),

                // Action Buttons
                Center(
                  child: Wrap(
                    spacing: 16,
                    children: [
                      OutlinedButton.icon(
                        onPressed: _resetToDefaults,
                        icon: const Icon(Icons.restore),
                        label: const Text('Reset to Defaults'),
                      ),
                      ElevatedButton.icon(
                        onPressed: _saveSettings,
                        icon: const Icon(Icons.save),
                        label: const Text('Save Settings'),
                      ),
                    ],
                  ),
                ),
                const SizedBox(height: 32),
              ],
            ),
          );
        },
      ),
    );
  }

  Widget _buildSectionHeader(String title) {
    return Padding(
      padding: const EdgeInsets.only(left: 4, bottom: 8),
      child: Text(
        title,
        style: Theme.of(context).textTheme.titleMedium?.copyWith(
              color: Theme.of(context).colorScheme.primary,
              fontWeight: FontWeight.bold,
            ),
      ),
    );
  }
}