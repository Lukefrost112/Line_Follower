# ESP32 Autonomous Line Follower Robot

An ESP32-based autonomous line following robot using a QTR-8RC infrared sensor array, PD control, automatic corner detection with in-place pivoting, and real-time Bluetooth tuning.

---

## Features

- **PD Control** with low-pass filtering for smooth, stable line following
- **Deadband** zone around center to eliminate twitching on straight sections
- **Automatic Corner Detection** — triggers when 5+ sensors simultaneously see black
- **In-place Pivot** at corners — outer wheel drives, inner wheel stops, no motor reversal
- **State Machine Architecture** — DISARMED → FOLLOW → PIVOT → BUMP → FOLLOW
- **Real-time Bluetooth Tuning** — adjust all parameters live without reflashing
- **No Serial dependency** — all debug output over Bluetooth, runs untethered

---

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | ESP32 Dev Board |
| Sensor Array | Pololu QTR-8RC (8 channel IR) |
| Motors | Dual DC motors with motor driver |
| Power | LiPo battery |

### Pin Assignments

| Function | GPIO Pins |
|----------|-----------|
| IR Sensors | 25, 26, 27, 14, 4, 16, 17, 5 |
| Left Motor (M2) | 22 (A), 23 (B) |
| Right Motor (M1) | 18 (A), 19 (B) |

---

## Software Dependencies

- [Arduino ESP32 Core](https://github.com/espressif/arduino-esp32)
- [QTRSensors Library](https://github.com/pololu/qtr-sensors-arduino) by Pololu
- `BluetoothSerial` (included in ESP32 Arduino core)

Install via Arduino Library Manager or PlatformIO.

---

## Getting Started

1. Clone this repo and open `LineFollower.ino` in Arduino IDE
2. Install the dependencies listed above
3. Select your ESP32 board and correct COM port
4. Flash the code
5. On power-up the robot immediately begins **calibration** for 4 seconds — move it slowly back and forth across the line during this time
6. Connect to `ESP32_PD` over Bluetooth from your phone or PC
7. Send `A` to arm and start following

---

## State Machine

```
DISARMED ──(A)──► FOLLOW ──(5+ sensors black)──► PIVOT
                    ▲                                │
                    │                                ▼
                  BUMP ◄────────(pivot done)──────── │
```

| State | Behaviour |
|-------|-----------|
| `DISARMED` | Motors stopped, waiting for arm command |
| `FOLLOW` | PD control active, reading sensor position |
| `PIVOT` | Outer wheel forward, inner stopped — arcs into corner |
| `BUMP` | Short forward nudge to relocate line after pivot |

---

## PD Control

The sensor array returns a position value from `0` to `7000`, where `3500` is perfectly centered.

```
error = position - 3500
steer = (Kp × error) + (Kd × (error - lastError))
filtered_steer = α × steer + (1 - α) × filtered_steer
```

- **Kp** corrects based on how far off-center the robot is
- **Kd** corrects based on how fast the error is changing — prevents overshoot and oscillation
- **Low-pass filter (α)** smooths out sensor noise spikes
- **Deadband** — errors within ±`DB` of center are treated as zero, eliminating twitching on straights

---

## Bluetooth Tuning Commands

Connect to `ESP32_PD` and send commands as plain text lines (e.g. `P0.020`, `B60`).

| Command | Parameter | Default | Range |
|---------|-----------|---------|-------|
| `P<val>` | Proportional gain Kp | `0.018` | 0.0 – 1.0 |
| `D<val>` | Derivative gain Kd | `1.500` | 0.0 – 5.0 |
| `B<val>` | Base speed | `55` | 0 – 255 |
| `DB<val>` | Deadband (error units) | `150` | 0 – 500 |
| `AL<val>` | Filter alpha | `0.40` | 0.0 – 1.0 |
| `PT<val>` | Pivot duration (ms) | `350` | 0 – 2000 |
| `PW<val>` | Pivot motor speed | `110` | 0 – 255 |
| `PB<val>` | Post-pivot bump (ms) | `150` | 0 – 2000 |
| `FW<val>` | Bump forward speed | `65` | 0 – 255 |
| `CC<val>` | Corner sensor count | `5` | 1 – 8 |
| `CT<val>` | Corner black threshold | `700` | 0 – 1000 |
| `A` | Arm robot | — | — |
| `S` | Stop / disarm | — | — |
| `V` | Print all current values | — | — |

---

## Tuning Guide

Follow this order for best results:

1. **Base Speed** — start low (`B50`) and only increase once control is stable
2. **Kp** — set `D0`, raise Kp until it oscillates on a straight, then back off ~30%
3. **Kd** — raise until straight-line oscillation stops. Typically 50–80× your Kp value
4. **Deadband** — if still twitching when centered, raise `DB` in steps of 25 (e.g. `DB175`, `DB200`)
5. **Filter alpha** — lower `AL` (e.g. `AL0.25`) for smoother straights; raise (e.g. `AL0.50`) if curves feel sluggish
6. **Pivot time** — tune `PT` in steps of 25ms until corners reliably hit ~90°. Usually 300–450ms
7. **Pivot speed** — raise `PW` for tighter arcs, lower if it overshoots the line after pivoting

---

## Project Structure

```
├── LineFollower.ino       # Main source file
├── README.md              # This file
└── docs/                  # GitHub Pages site
    └── index.html
```

---

## License

MIT License — free to use, modify, and distribute.

---

## Demo

> 📸 *Add photo here*
> 🎥 *Add video link here*
