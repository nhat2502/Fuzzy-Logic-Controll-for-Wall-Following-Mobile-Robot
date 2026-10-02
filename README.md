# Wall-Following Mobile Robot with Fuzzy Control 

This repository contains the source code, communication scripts, and monitoring software for a differential-drive mobile robot. The system utilizes a Fuzzy Logic Controller to maintain a fixed distance from a wall while moving.

## 📌 Key Features
*   **Fuzzy Logic Control:** Implements MAX-MIN inference and weighted average defuzzification to adjust the angular velocity of the wheels based on distance and orientation errors.
*   **Sensor Noise Filtering:** Utilizes a Kalman filter on ultrasonic sensor readings to ensure stable and reliable distance estimation.
*   **Closed-loop Speed Control:** Uses an inner PID control loop for precise DC motor speed regulation via encoder feedback.
*   **Wireless Telemetry:** Streams real-time data from the robot to a PC dashboard using an ESP32 as a UART-to-Bluetooth bridge.

## 🚀 Repository Structure

Based on the uploaded files, the repository is organized as follows:

*   📂 **`Fuzzy_Controller_Arduino/`** 
    Contains the core control logic running on the Arduino Uno R3. This includes the Fuzzy Logic algorithm, Kalman filter implementation, encoder reading interrupts, and PID motor control.
*   📂 **`UARTtoBluetooth_ESP32/`**
    Contains the code for the ESP32 module. It acts as a wireless bridge, reading UART serial data from the Arduino and transmitting it over Bluetooth to the PC.
*   📦 **`BTL_DKTM_Monitor.rar`**
    A compressed archive containing the C# WinForms GUI application. Extract this file and run the application to visualize the robot's real-time Process Value (PV - actual distance) and Setpoint (SP - target distance) on a dynamic chart.
*   📄 **`Fuzzy Control_Wall Following Robot_Report.pdf`**
    Comprehensive project report detailing the kinematic model, fuzzy controller design, hardware configuration, and experimental results (in Vietnamese).
## 🛠 Hardware Components
*   **Main Controller:** Arduino Uno R3
*   **Communication:** ESP32 Module
*   **Sensors:** 2x MKE-S01 Ultrasonic Sensors (Front & Rear)
*   **Actuators:** 2x JGB37-520 12V DC Motors with Encoders
*   **Motor Driver:** L298N
*   **Power:** 3x 3.7V Li-ion batteries
