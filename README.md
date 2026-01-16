# 3-Axis AI Tracking Gimbal

A high-performance active stabilizing gimbal using **Computer Vision (YOLOv8)** on a **Raspberry Pi 5** for object tracking, and an **ESP32** for real-time motor control and IMU stabilization.

## Features
- **Object Tracking:** Uses YOLOv8 (Nano) to track people/objects at high framerates.
- **Hybrid Motor Control:** 
  - **Pan:** NEMA 17 Stepper (via TMC2209) for 360° continuous rotation.
  - **Tilt:** MG90S metal micro servo motor 
  - **Roll:** ES08MDII digital metal micro servo motor
- **Auto-Leveling:** MPU6500 IMU keeps the camera horizon level (Roll axis) independently of tracking.
- **Communication:** UART Serial link between Pi 5 (Vision) and ESP32 (Motion).

## 🛠 Hardware List
- **Compute:** Raspberry Pi 5 (8GB) + Camera Module 3
- **Controller:** ESP32 Dev Module
- **Motors:** 
  - Pan: NEMA 17 Stepper
  - Tilt: MG90s Metal Gear Servo
  - Roll: ES08MDII Metal Gear Servo
- **Drivers & Sensors:** 
  - TMC2209 Stepper Driver (UART/DIR mode)
  - MPU6500 6-Axis Gyro/Accelerometer
- **Power:** External 12V (Stepper) and 5V (Pi/Servos) sources

## Wiring Overview
| Component | Pin | Connected To |
|-----------|-----|--------------|
| **Pi 5**  | UART TX (GPIO14)| ESP32 RX (GPIO16/User Def) |
| **Pi 5**  | UART RX (GPIO15)| ESP32 TX (GPIO17/User Def) |
| **Stepper**| STEP | ESP32 GPIO 25 |
| **Stepper**| DIR | ESP32 GPIO 26 |
| **Servo** | Tilt Signal | ESP32 GPIO 17 |
| **Servo** | Roll Signal | ESP32 GPIO 16 |
| **IMU**   | SDA/SCL | ESP32 GPIO 21/22 |

## 💻 Installation

### 1. Raspberry Pi (Vision)
```bash
cd rpi_vision
pip install -r requirements.txt
python main.py
