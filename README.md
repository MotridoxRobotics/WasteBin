# WasteBin (Westo)

A Smart Waste Bin with built-in waste compression mechanism.

**ESP32** + **2x HC-SR04** + **2x Stepper Motors (A4988)** + **1x Servo** + **Flutter App**

## Features
- Auto-detect person → open lid via servo
- Dual stepper compression plate with stall detection
- Ultrasonic waste level measurement
- Wi-Fi AP + STA with web dashboard
- REST API for Flutter companion app ([Westo](https://github.com/IrfanFathan/westo))

## Wiring

### HC-SR04 Front (Person Detection)
| HC-SR04 | ESP32 |
|---------|-------|
| VCC | 5V |
| GND | GND |
| TRIG | GPIO 23 |
| ECHO | GPIO 22 |

### HC-SR04 Top (Plate/Waste Level)
| HC-SR04 | ESP32 |
|---------|-------|
| VCC | 5V |
| GND | GND |
| TRIG | GPIO 19 |
| ECHO | GPIO 18 |

### A4988 Driver 1 → Stepper Motor 1
| A4988 | Connect To |
|-------|-----------|
| STEP | GPIO 25 |
| DIR | GPIO 26 |
| SLEEP | GPIO 27 |
| RESET | **Jumper to SLEEP** ⚠️ |
| MS1 | GPIO 16 (shared) |
| MS2 | GPIO 17 (shared) |
| MS3 | GPIO 5 (shared) |
| VDD | 3.3V |
| GND | Common GND |
| VMOT | 8-35V motor supply |
| 1A/1B/2A/2B | Stepper coil wires |

### A4988 Driver 2 → Stepper Motor 2
| A4988 | Connect To |
|-------|-----------|
| STEP | GPIO 32 |
| DIR | GPIO 33 |
| SLEEP | GPIO 14 |
| RESET | **Jumper to SLEEP** ⚠️ |
| MS1 | GPIO 16 (shared) |
| MS2 | GPIO 17 (shared) |
| MS3 | GPIO 5 (shared) |
| VDD | 3.3V |
| GND | Common GND |
| VMOT | 8-35V motor supply |
| 1A/1B/2A/2B | Stepper coil wires |

### SG90 Servo (Lid)
| Servo | ESP32 |
|-------|-------|
| Signal | GPIO 4 |
| VCC | 5V |
| GND | GND |

### ⚠️ Important
- **RESET↔SLEEP jumper is mandatory** on each A4988 — solder or wire a bridge between these two pins on the board. RESET has no internal pull-up; leaving it floating causes erratic behavior.
- **MS1/MS2/MS3 are shared** between both drivers — wire each MS pin from the ESP32 to BOTH A4988 boards. All set HIGH = 1/16 microstepping (best torque and smoothness).
- **100µF capacitor** across VMOT–GND on each A4988 is recommended.
- **All GNDs must be connected** (ESP32 + motor supply + A4988).

## Calibration

After wiring, update these `#define` values in `WasteBin.ino`:
- `PLATE_TOP_CM` — sensor-to-plate distance when plate is at the top
- `PLATE_BOTTOM_CM` — sensor-to-plate distance when plate is at the bottom
- `DIR_DOWN` / `DIR_UP` — swap if plate moves the wrong direction
- `STEPPER2_INVERT` — set `true` if motor 2 is physically mirrored

## API

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Web dashboard |
| `/status` | GET | JSON: waste level, sensor data, compressor state |
| `/compress` | POST | Trigger compression cycle |
| `/update?level=XX` | GET | Manual waste level override |
| `/device/info` | GET | Firmware version, MAC, IP |

## Version History

| Version | Changes |
|---------|---------|
| v2.2.0 | 1/16 microstepping: shared MS1/MS2/MS3 pins, speed adjustment |
| v2.1.0 | Stall debounce, proximity debounce, boot safety, timeout fix |
| v2.0.0 | Full hardware: steppers, ultrasonics, servo, state machine |
| v1.1.0 | WiFi AP, dashboard, LED blink on compress trigger |

## License

MIT
