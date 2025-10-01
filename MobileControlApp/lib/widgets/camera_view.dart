import 'package:flutter/material.dart';
import 'dart:typed_data';
import 'dart:async';
import 'dart:io';

class CameraView extends StatefulWidget {
  final String cameraIp;
  final bool isConnected;

  const CameraView({
    Key? key,
    required this.cameraIp,
    required this.isConnected,
  }) : super(key: key);

  @override
  State<CameraView> createState() => _CameraViewState();
}

class _CameraViewState extends State<CameraView> {
  HttpClient? _httpClient;
  StreamSubscription? _streamSubscription;
  Uint8List? _imageData;
  bool _isStreaming = false;
  String _errorMessage = '';

  @override
  void initState() {
    super.initState();
    if (widget.isConnected) {
      _startStream();
    }
  }

  @override
  void didUpdateWidget(CameraView oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.isConnected != oldWidget.isConnected ||
        widget.cameraIp != oldWidget.cameraIp) {
      if (widget.isConnected) {
        _startStream();
      } else {
        _stopStream();
      }
    }
  }

  @override
  void dispose() {
    _stopStream();
    super.dispose();
  }

  Future<void> _startStream() async {
    if (_isStreaming) return;

    try {
      setState(() {
        _isStreaming = true;
        _errorMessage = '';
      });

      _httpClient = HttpClient();
      _httpClient!.connectionTimeout = const Duration(seconds: 5);

      final request = await _httpClient!.getUrl(
        Uri.parse('http://${widget.cameraIp}/stream'),
      );

      final response = await request.close();

      if (response.statusCode == 200) {
        final List<int> buffer = [];
        bool isReadingJpeg = false;

        _streamSubscription = response.listen(
          (List<int> data) {
            buffer.addAll(data);

            // Look for JPEG boundaries
            while (buffer.length > 2) {
              if (!isReadingJpeg) {
                // Look for JPEG start marker (0xFF 0xD8)
                int startIndex = -1;
                for (int i = 0; i < buffer.length - 1; i++) {
                  if (buffer[i] == 0xFF && buffer[i + 1] == 0xD8) {
                    startIndex = i;
                    break;
                  }
                }

                if (startIndex >= 0) {
                  // Remove everything before the JPEG start
                  buffer.removeRange(0, startIndex);
                  isReadingJpeg = true;
                } else {
                  // No JPEG start found, keep only last few bytes
                  if (buffer.length > 1024) {
                    buffer.removeRange(0, buffer.length - 1024);
                  }
                  break;
                }
              }

              if (isReadingJpeg) {
                // Look for JPEG end marker (0xFF 0xD9)
                int endIndex = -1;
                for (int i = 2; i < buffer.length - 1; i++) {
                  if (buffer[i] == 0xFF && buffer[i + 1] == 0xD9) {
                    endIndex = i + 2;
                    break;
                  }
                }

                if (endIndex >= 0) {
                  // Extract the complete JPEG image
                  final imageBytes = Uint8List.fromList(
                    buffer.sublist(0, endIndex),
                  );

                  // Update the displayed image
                  if (mounted) {
                    setState(() {
                      _imageData = imageBytes;
                    });
                  }

                  // Remove the processed image from buffer
                  buffer.removeRange(0, endIndex);
                  isReadingJpeg = false;
                } else {
                  // No end marker found yet, wait for more data
                  break;
                }
              }
            }
          },
          onError: (error) {
            if (mounted) {
              setState(() {
                _errorMessage = 'Stream error: $error';
                _isStreaming = false;
              });
            }
          },
          onDone: () {
            if (mounted) {
              setState(() {
                _isStreaming = false;
              });
            }
          },
          cancelOnError: true,
        );
      } else {
        setState(() {
          _errorMessage = 'Failed to connect: ${response.statusCode}';
          _isStreaming = false;
        });
      }
    } catch (e) {
      if (mounted) {
        setState(() {
          _errorMessage = 'Connection error: $e';
          _isStreaming = false;
        });
      }
    }
  }

  void _stopStream() {
    _streamSubscription?.cancel();
    _streamSubscription = null;
    _httpClient?.close();
    _httpClient = null;

    if (mounted) {
      setState(() {
        _isStreaming = false;
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return Card(
      elevation: 4,
      child: ClipRRect(
        borderRadius: BorderRadius.circular(8),
        child: Container(
          color: Colors.black,
          child: Stack(
            fit: StackFit.expand,
            children: [
              if (_imageData != null)
                Image.memory(
                  _imageData!,
                  fit: BoxFit.cover,
                  gaplessPlayback: true,
                )
              else
                Center(
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      Icon(
                        widget.isConnected
                            ? Icons.videocam
                            : Icons.videocam_off,
                        size: 48,
                        color: Colors.white54,
                      ),
                      const SizedBox(height: 16),
                      Text(
                        widget.isConnected
                            ? (_isStreaming
                                ? 'Loading camera...'
                                : 'Connecting to camera...')
                            : 'Camera not connected',
                        style: const TextStyle(
                          color: Colors.white54,
                          fontSize: 14,
                        ),
                      ),
                      if (_errorMessage.isNotEmpty) ...[
                        const SizedBox(height: 8),
                        Text(
                          _errorMessage,
                          style: const TextStyle(
                            color: Colors.redAccent,
                            fontSize: 12,
                          ),
                          textAlign: TextAlign.center,
                        ),
                      ],
                    ],
                  ),
                ),

              // Streaming indicator
              if (_isStreaming)
                Positioned(
                  top: 8,
                  right: 8,
                  child: Container(
                    padding: const EdgeInsets.symmetric(
                      horizontal: 8,
                      vertical: 4,
                    ),
                    decoration: BoxDecoration(
                      color: Colors.red,
                      borderRadius: BorderRadius.circular(4),
                    ),
                    child: Row(
                      mainAxisSize: MainAxisSize.min,
                      children: const [
                        Icon(
                          Icons.circle,
                          size: 8,
                          color: Colors.white,
                        ),
                        SizedBox(width: 4),
                        Text(
                          'LIVE',
                          style: TextStyle(
                            color: Colors.white,
                            fontSize: 12,
                            fontWeight: FontWeight.bold,
                          ),
                        ),
                      ],
                    ),
                  ),
                ),

              // Retry button
              if (!_isStreaming && widget.isConnected && _errorMessage.isNotEmpty)
                Positioned(
                  bottom: 16,
                  left: 0,
                  right: 0,
                  child: Center(
                    child: ElevatedButton.icon(
                      onPressed: _startStream,
                      icon: const Icon(Icons.refresh),
                      label: const Text('Retry'),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Theme.of(context).primaryColor,
                      ),
                    ),
                  ),
                ),
            ],
          ),
        ),
      ),
    );
  }
}