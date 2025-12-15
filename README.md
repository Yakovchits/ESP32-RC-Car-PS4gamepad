PS4-controlled RC Car

A high-performance RC drift car controlled by a PS4 gamepad via Bluetooth, featuring real-time stability control with a gyroscope (BNO055), dual-motor torque vectoring (front/rear ESC), battery voltage monitoring with LED feedback gamepad, and persistent steering trim memory. Built on ESP32 and Bluepad32, it combines responsive handling with advanced telemetry.

Key features:
🎮 PS4 Bluetooth control – Low-latency gamepad input
🌀 Active yaw stabilization – PID-controlled drift assist
⚡ Dual ESC with torque vectoring
🔋 Smart battery monitor – RGB LED alerts & vibration warnings
🎯 Persistent steering trim – Saved to EEPROM, adjustable via L1/R1

Perfect for: RC enthusiasts, drift competitions, and embedded systems projects.

Code optimized for minimal latency – no blocking delays, all non-blocking logic.

📋 Pin Connection Table
Component	ESP32 Pin	Purpose	Notes
Steering Servo	GPIO 18	Steering control	0-180°, center 90°
Extra Servo 1	GPIO 19	Auxiliary function	Configure via buttons
Extra Servo 2	GPIO 21	Auxiliary function	Configure via buttons
Rear ESC	GPIO 17	Rear motor	PWM 1000-2000 µs
Front ESC	GPIO 16	Front motor	PWM 1000-2000 µs
BNO055 SDA	GPIO 21	I2C data	4.7kΩ pull-up to 3.3V
BNO055 SCL	GPIO 22	I2C clock	4.7kΩ pull-up to 3.3V
Battery 1 divider	GPIO 34	Battery 1 measurement	100k/10k divider
Battery 2 divider	GPIO 35	Battery 2 measurement	100k/10k divider

Steering Trim Calibration (Live)
Button	Action	LED Feedback
PS4 L1	-2° trim (left)	Controller flashes
PS4 R1	+2° trim (right)	Controller flashes
PS4 Options	Reset trim to 0	Double flash

<img width="8330" height="2385" alt="deepseek_mermaid_20251215_61f9f1" src="https://github.com/user-attachments/assets/10a0debf-8206-4cdf-a7d9-2b17b42e6a2c" />
