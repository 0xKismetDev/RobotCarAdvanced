# BlockBasedLearning Robot Car — Reliability Fix

## What This Is

A block-based programming interface for an educational robot car, built for children to learn programming fundamentals by dragging visual blocks and watching the car execute their programs. The system spans a Blockly web interface, an ESP32 WiFi controller, and an Arduino motor/sensor slave connected via I2C. This project focuses on making the BlockBasedLearning subsystem work reliably — accurate movements, responsive command chains, and robust communication.

## Core Value

When a child builds a program with blocks and hits "Run", the car must do exactly what the blocks say — no drift, no mysterious pauses, no broken command chains. If the blocks say "drive forward 30cm, turn right 90 degrees, drive forward 30cm", that's what happens.

## Requirements

### Validated

- ✓ Blockly web interface loads and connects to ESP32 via WebSocket — existing
- ✓ Custom Blockly blocks defined for robot movement (forward, backward, turn, etc.) — existing
- ✓ ESP32 creates WiFi AP (`RobotCar-Blockly`) and runs WebSocket server on port 81 — existing
- ✓ ESP32 relays commands to Arduino slave over I2C (address 0x08) — existing
- ✓ Arduino processes motor commands (M, D, R, S, E, Q) — existing
- ✓ Encoder-based movement with PCINT interrupts on A0/A1 (21 pulses/rotation) — existing
- ✓ Ultrasonic distance sensor reading and servo control — existing
- ✓ Servo auto-detach to prevent twitching — existing
- ✓ WebSocket heartbeat and reconnection handling — existing
- ✓ Motor safety stop on client disconnect — existing

### Active

- [ ] Car drives in a straight line without drifting (encoder-corrected motor balancing)
- [ ] Commands execute without noticeable delay between them (eliminate post-turn latency, currently up to 5s)
- [ ] Blocking movement functions (moveDistance, rotateDegrees) don't stall the command pipeline
- [ ] Communication pipeline (Blockly → ESP32 → Arduino → response → next command) is robust and responsive
- [ ] Movement accuracy: commanded distances and angles match actual movements within reasonable tolerance
- [ ] Command chain execution works smoothly for typical kid programs (drive-turn-drive patterns)

### Out of Scope

- Flutter mobile app improvements — separate project, not part of this work
- ESP32_Master firmware — only BlockBasedLearning/ESP32_Code matters here
- ESP32 Camera functionality — not relevant to block-based learning
- Hardware changes (new sensors, different motors) — software fixes only
- Multi-user/classroom features — single user at a time is fine
- Battery monitoring — nice to have but not blocking reliability
- WiFi credential security — acceptable for educational local use

## Context

- **Architecture:** ESP32 master creates WiFi AP, serves WebSocket. Arduino slave handles motors/encoders via I2C. Blockly web interface generates JS that sends WebSocket commands.
- **Known drift issue:** LEFT_MOTOR_FACTOR=1.05, RIGHT_MOTOR_FACTOR=0.90 are hand-tuned calibration values. No real-time encoder feedback correction during straight-line driving — motors run open-loop with pre-calibrated factors.
- **Known latency issue:** `moveDistance()` and `rotateDegrees()` are blocking `while(true)` loops on Arduino. Larger movements/turns block longer. The ESP32 polls for completion but the blocking nature likely causes the command chain delay — the Arduino can't acknowledge completion quickly, and the ESP32/Blockly side waits.
- **Duplicate firmware:** BlockBasedLearning/ESP32_Code.ino (658 lines) is a separate, newer firmware from ESP32_Master.ino (305 lines). Only the Blockly version matters for this project.
- **No automated tests:** All testing is manual — flash firmware, run Blockly programs, observe car behavior.
- **Target audience:** Children learning programming. The system must be forgiving and predictable. A "drive in a square" program should produce a recognizable square.

## Constraints

- **Tech stack**: Arduino C++ on UNO (8-bit, 32KB flash, 2KB RAM), ESP32 Arduino framework, vanilla JS web interface — no changes to languages or frameworks
- **Hardware**: Existing AZ Delivery robot car kit — 65mm wheels, 145mm wheelbase, L298N motor driver, 21-pulse encoders, HC-SR04 ultrasonic, SG90 servo
- **I2C protocol**: Must remain backward-compatible with existing command format (M, D, R, S, E, Q commands) — other control interfaces may use it
- **Arduino RAM**: Only 2KB available — no heavy data structures or large buffers
- **Upload method**: Arduino IDE — no PlatformIO or custom build systems

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| Focus only on BlockBasedLearning subsystem | User's priority; Flutter app works separately | — Pending |
| Software-only fixes (no hardware mods) | Minimize complexity; work with existing kit | — Pending |
| Keep I2C command format compatible | Other interfaces (Flutter app) use same Arduino slave | — Pending |

---
*Last updated: 2026-02-05 after initialization*
