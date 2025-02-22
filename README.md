# SM18512PS LED Strip Controller with DMX512 Protocol

## Overview
This project provides a C++ implementation for controlling LED strips using the SM18512PS IC via the DMX512 protocol. The LED strip consists of four wires:
- **12-24V Input** (Power Supply)
- **A & B Wires** (RS485 Communication)
- **Negative (GND)**

The communication is established using a **USB to RS485** adapter, and the software ensures proper control of the LED effects through serial communication.

## Features
- DMX512 Protocol for LED control
- Supports **RGBW** color variation
- Predefined effects: **Solid, Breathing, Flashing**
- Implemented in **C++**
- Integrated with **ROS 2** for use in **Autonomous Mobile Robots (AMR) or Automated Guided Vehicles (AGV)**

## Technical Details
- **Baud Rate:**
  - **250000** for normal operation
  - **90000** for initialization
- **Time Breaks:**
  - **100 microseconds** for 250000 baud
  - **12 microseconds** for data transmission

*Note:* These values may vary depending on different LED strips and ICs.

## LED Control Behavior
Due to the architecture of the **SM18512PS IC**, **individual LED control is not possible**. Instead, we control **groups of LEDs**:
- Each **IC controls 6 LEDs**.
- We define **1 Group = 6 LEDs**.
- This means **control is limited to groups of 6 LEDs** rather than individual LEDs.

## ROS 2 Integration
This project includes a **ROS 2 package** that allows seamless integration with AMR and AGV systems. The ROS 2 node listens for LED mode commands and publishes the LED status.

### Package Usage
1. **Clone the Repository:**
   ```sh
   https://github.com/Kapilmech2003/led_strip_cpp_code.git
   cd led_strip_cpp_code
   ```
2. **Build the Package:**
   ```sh
   colcon build --packages-select led_strip_cpp_code
   ```
3. **Run the Node:**
   ```sh
   ros2 run led_strip_cpp_code ros_led
   ```

## Contact
For any issues or help needed, feel free to reach out:
📧 **kapilkapi2003@gmail.com**

