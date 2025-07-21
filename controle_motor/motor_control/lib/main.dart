import 'dart:async';
import 'dart:math';

import 'package:flutter/material.dart';

void main() {
  runApp(const RotatingJointApp());
}

class RotatingJointApp extends StatelessWidget {
  const RotatingJointApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Controle de Junta Rotativa',
      theme: ThemeData(
        useMaterial3: true,
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.teal),
      ),
      home: const JointControlPage(),
    );
  }
}

class JointControlPage extends StatefulWidget {
  const JointControlPage({super.key});

  @override
  State<JointControlPage> createState() => _JointControlPageState();
}

class _JointControlPageState extends State<JointControlPage> {
  double _angle = 0.0;      // graus [0..180]
  double _velocity = 0.34;   // rad/s
  String _status = '';

  final List<double> _sequence = [30, 120, 75, 100, 0];

  void _sendToEsp(double angle, double velocity) {
    // TODO: implementar comunicação com ESP32 (HTTP, WebSocket etc.)
    setState(() {
      _status = 'Enviado: ângulo ${angle.toStringAsFixed(1)}°, velocidade ${velocity.toStringAsFixed(2)} rad/s';
    });
  }

  Future<void> _runImplementation04() async {
    for (var target in _sequence) {
      // Ajusta velocidade conforme trecho
      double speed = (target == 100 && _angle != 0)
          ? 0.17
          : 0.34;
      setState(() {
        _velocity = speed;
        _angle = target;
      });
      _sendToEsp(_angle, _velocity);
      await Future.delayed(const Duration(seconds: 2));
    }
    setState(() {
      _status = 'Implementação 04 concluída';
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Controle de Junta Rotativa'),
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            // Protractor display
            Expanded(
              child: CustomPaint(
                painter: _ProtractorPainter(angle: _angle),
                child: Center(
                  child: Transform.rotate(
                    angle: (pi * (_angle - 90) / 180),
                    child: Container(
                      width: 4,
                      height: 100,
                      color: Theme.of(context).colorScheme.primary,
                    ),
                  ),
                ),
              ),
            ),
            const SizedBox(height: 16),
            // Angle slider + input
            Row(
              children: [
                Expanded(
                  child: Slider(
                    min: 0,
                    max: 180,
                    divisions: 180,
                    value: _angle,
                    label: '${_angle.round()}°',
                    onChanged: (v) {
                      setState(() => _angle = v);
                      _sendToEsp(_angle, _velocity);
                    },
                  ),
                ),
                SizedBox(
                  width: 60,
                  child: TextField(
                    keyboardType: TextInputType.number,
                    decoration: const InputDecoration(
                      labelText: 'Ângulo',
                      suffixText: '°',
                    ),
                    onSubmitted: (txt) {
                      double? v = double.tryParse(txt);
                      if (v == null) return;
                      if (v < 0) v = 0;
                      if (v > 180) v = 180;
                      setState(() => _angle = v!);
                      _sendToEsp(_angle, _velocity);
                    },
                  ),
                ),
              ],
            ),
            const SizedBox(height: 8),
            // Velocity slider + input
            Row(
              children: [
                Expanded(
                  child: Slider(
                    min: 0.0,
                    max: 1.0,
                    divisions: 100,
                    value: _velocity,
                    label: '${_velocity.toStringAsFixed(2)} rad/s',
                    onChanged: (v) {
                      setState(() => _velocity = v);
                      _sendToEsp(_angle, _velocity);
                    },
                  ),
                ),
                SizedBox(
                  width: 80,
                  child: TextField(
                    keyboardType: TextInputType.number,
                    decoration: const InputDecoration(
                      labelText: 'Vel. (rad/s)',
                    ),
                    onSubmitted: (txt) {
                      double? v = double.tryParse(txt);
                      if (v == null) return;
                      setState(() => _velocity = v);
                      _sendToEsp(_angle, _velocity);
                    },
                  ),
                ),
              ],
            ),
            const SizedBox(height: 16),
            // Status display
            Text(
              _status,
              textAlign: TextAlign.center,
              style: TextStyle(
                fontStyle: FontStyle.italic,
                color: Theme.of(context).colorScheme.secondary,
              ),
            ),
            const SizedBox(height: 16),
            // Botão Implementação 04
            ElevatedButton(
              onPressed: _runImplementation04,
              child: const Text('Implementação 04'),
            ),
          ],
        ),
      ),
    );
  }
}

class _ProtractorPainter extends CustomPainter {
  final double angle;
  _ProtractorPainter({required this.angle});

  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = Colors.grey.shade300
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2;
    final center = Offset(size.width/2, size.height);
    final radius = size.width/2;
    // Semicírculo
    canvas.drawArc(
      Rect.fromCircle(center: center, radius: radius),
      pi,
      pi,
      false,
      paint,
    );
    // Marcas a cada 10°
    for (int i = 0; i<=18; i++) {
      double a = pi + (pi/18)*i;
      double x1 = center.dx + radius * cos(a);
      double y1 = center.dy + radius * sin(a);
      double len = i % 3 == 0 ? 15 : 8;
      double x2 = center.dx + (radius - len) * cos(a);
      double y2 = center.dy + (radius - len) * sin(a);
      canvas.drawLine(Offset(x1,y1), Offset(x2,y2), paint);
      // Texto
      if (i % 3 == 0) {
        String text = (i*10).toString();
        final tp = TextPainter(
          text: TextSpan(
            text: text,
            style: TextStyle(color: Colors.black, fontSize: 12),
          ),
          textAlign: TextAlign.center,
          textDirection: TextDirection.ltr,
        );
        tp.layout();
        double tx = center.dx + (radius - 30) * cos(a) - tp.width/2;
        double ty = center.dy + (radius - 30) * sin(a) - tp.height/2;
        tp.paint(canvas, Offset(tx, ty));
      }
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}
