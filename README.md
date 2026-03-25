# ESP32 Rescue Robot Car

This project is a low-cost, Wi-Fi-controlled Unmanned Ground Vehicle (UGV) built with an ESP32, designed for hazardous environment inspection. 

Unlike traditional Bluetooth robots, this vehicle operates as a standalone Wi-Fi Access Point (SoftAP). You can control it and view live sensor data from any standard web browser—no app installation required!

## Key Features

* **Standalone Web Dashboard:** HTML/JS interface stored in the ESP32 PROGMEM.
* **Real-Time Telemetry:** AJAX updates MQ-2 gas levels and HC-SR04 distance continuously without page refreshes.
* **Non-Blocking Architecture:** Built on a Finite State Machine (FSM) using `millis()`, ensuring smooth multitasking for motor control, sensor polling, and web serving.
* **Autonomous Modes:** Features Obstacle Avoidance using an SG90 servo, plus Wall-Following and Gas Tracking.
* **Live 2D Mapping:** Client-side rendering plots the robot's trajectory and hazard zones.

## Bill of Materials (Components List)

* **Microcontroller:** ESP32 Development Board
* **Motor Driver:** L298N Dual H-Bridge
* **Chassis & Actuators:** Standard 4WD Robot Chassis Kit (includes 4x Yellow TT DC Gear Motors and wheels)
* **Radar Actuator:** SG90 Micro Servo
* **Sensors:** MQ-2 Analog Gas Sensor, HC-SR04 Ultrasonic Sensor
* **Power Supply:** 2x 18650 Lithium-ion batteries with a battery holder

## Circuit & Wiring Diagram

### 1. Power Supply
* 18650 Battery Pack (+) ➔ L298N (12V Input)
* 18650 Battery Pack (-) ➔ L298N (GND)
* L298N (5V Output) ➔ ESP32 (VIN / 5V)
* L298N (GND) ➔ ESP32 (GND)

### 2. Motor Control (L298N to ESP32)
* ENA ➔ GPIO 14 (Left Motor PWM)
* IN1 ➔ GPIO 27 (Left Direction 1)
* IN2 ➔ GPIO 26 (Left Direction 2)
* IN3 ➔ GPIO 25 (Right Direction 1)
* IN4 ➔ GPIO 33 (Right Direction 2)
* ENB ➔ GPIO 32 (Right Motor PWM)

### 3. Sensors & Actuators
* HC-SR04 (Ultrasonic): TRIG ➔ GPIO 4 | ECHO ➔ GPIO 5
* MQ-2 (Gas Sensor): A0 ➔ GPIO 36 (VP)
* SG90 (Radar Servo): Signal ➔ GPIO 13

## How to Operate

1. Turn on the hardware power switch. Wait 3-5 seconds for the ESP32 to initialize the sensors and broadcast its Wi-Fi.
2. Connect your smartphone or laptop to the robot's Wi-Fi network:
   * **SSID:** Robot_Tham_Do
   * **Password:** thebeacroxtheic161093
3. Open a web browser (Chrome, Safari, Edge) and navigate to the default IP address: `http://192.168.4.1`
4. The Control Dashboard will load immediately. Select a mode or use the D-Pad to drive!
