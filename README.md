# ESP32 High-Speed Self-Balancing Robot

## Description
A high-speed self-balancing robot using ESP32, demonstrating real-time control and stabilization algorithms for two-wheeled robots.

## Hardware Setup
- **Microcontroller:** ESP32
- **Motors:** 2 DC motors with encoders
- **IMU Sensor:** MPU6050
- **Power Supply:** Li-ion battery pack
- **Wheels:** 2 standard wheels for balance
- **Other:** Chassis, connectors, and supporting electronics

## Features
- Real-time balancing using sensor feedback from the MPU6050
- High-speed operation with stable control
- Modular and clean code structure for easy customization

## Code Overview
- Written in **C/C++** for the **ESP32** platform
- Implements PID-based control for balancing
- Uses IMU data for angle and rate calculation
- Modular functions for sensor reading, motor control, and stabilization

## How to Use
1. Connect the hardware components according to the setup above.
2. Flash the `main.c` code to your ESP32 using Arduino IDE or PlatformIO.
3. Power on the robot and observe self-balancing in action.
4. Adjust PID constants in the code for optimal performance.

## Acknowledgements
This project is inspired by Wouter Klop's high-speed balancing robot design.

## License
This project is open-source under the MIT License.


