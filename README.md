# Self-Driving Car Project

## Overview
This project is a prototype for an autonomous self-driving car that uses a combination of Arduino firmware and Python scripts. The Arduino code (in the `.ino` file) handles real-time sensor inputs and motor control, while the Python code processes higher-level data such as computer vision and decision-making through a serial connection.  
![Demo Image](image.jpg)

With recent updates, the project now integrates ROS2 to enable advanced control schemes. It supports both manual (joystick-controlled) and fully autonomous modes, where the system can operate in two fully autonomous configurations:
- **Standalone Autonomous Mode:** The Raspberry Pi 5 serves as the master, handling all sensor processing, SLAM, and computer vision without external interference.
- **Remote Autonomous Mode:** An external master computer (laptop/desktop) processes the sensor data and sends control commands via ROS2, while the Raspberry Pi acts as a gateway interfacing with the Arduino.

## Features
- **Autonomous Navigation:** Real-time sensor fusion, SLAM for mapping, and obstacle detection/path planning.
- **Dual Software Architecture:** 
  - **Low-Level Control:** Arduino firmware for direct hardware interfacing (motor control, sensor data).
  - **High-Level Processing:** Python scripts and ROS2 nodes for computer vision, decision-making, and remote control.
- **ROS2 Integration:** Modular nodes enable distributed control—teleoperation via a joystick or fully autonomous modes.
- **Modular Design:** Easily expandable with clear separation between hardware control and advanced software algorithms.
- **Real-Time Feedback:** Console outputs, ROS2 topics, and logging for system monitoring and debugging.

## Hardware Requirements
- **Processing Units:**
  - **Raspberry Pi 5:** Acts as the ROS2 gateway and can serve as the master in standalone autonomous mode.
  - **Laptop/Desktop PC:** Optionally serves as the ROS2 master in remote autonomous or teleoperated modes.
- **Arduino Boards:**
  - **Arduino Uno R4 WiFi Board:** Primary board for interfacing with sensors and controlling actuators.
  - **Arduino Uno (with WiFi Module):** For legacy support and additional connectivity.
- **Motor & Sensor Components:**
  - **Motor Driver:** L298N (or equivalent) for controlling DC motors.
  - **DC Motors & Wheels:** For propulsion and movement.
  - **Ultrasonic Sensors:** For distance measurement and obstacle detection.
  - **Camera Module:** For visual processing, road detection, and computer vision tasks.
  - **Servo Motor:** For steering control (if applicable).
- **Additional Components:**
  - **Power Supply:** Battery pack with appropriate voltage/current ratings.
  - **MicroSD Card:** For Raspberry Pi storage.
  - **Raspberry Pi Case with Fan:** For cooling and protection.
  - **Additional Wiring:** Jumper wires, breadboard, resistors, and connectors.

## Software Requirements
- **Arduino IDE:** To compile and upload the `.ino` files to your Arduino boards.
- **Python 3.x:** For running high-level control scripts and ROS2 nodes.
- **ROS2 (Foxy, Galactic, or newer):** For distributed communication and node management.
- **Python Libraries:**
  - `pyserial` (for serial communication)
  - `opencv-python` (for computer vision tasks)
  - `numpy` (for numerical operations)
  - *(Additional libraries as needed for ROS2 nodes, SLAM, etc.)*

## Installation

### Arduino Firmware
1. **Open the Arduino Code:**
   - For the legacy Arduino Uno with WiFi module, open `firmware/self_driving_car.ino` in the Arduino IDE.
   - For the Arduino Uno R4 WiFi board, open `firmware/self_driving_car_r4.ino`.
2. **Connect Your Arduino:**
   - Plug in your Arduino board via USB or configure the WiFi connection as specified.
3. **Select Board and Port:**
   - In the Arduino IDE, choose the correct board model and COM port.
4. **Upload the Code:**
   - Click the upload button to flash the firmware to your Arduino.

### Python Software & ROS2 Setup
1. **Clone or Download the Repository:**
   ```bash
   git clone https://github.com/your_username/self-driving-car.git
   cd self-driving-car
   ```
2. **Navigate to the Project Directory:**  
   Open a terminal in the project folder.
3. **Set Up a Virtual Environment (Optional):**
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows use: venv\Scripts\activate
   ```
4. **Install Required Python Libraries:**
   ```bash
   pip install -r software/requirements.txt
   ```
5. **ROS2 Workspace Setup:**
   - Place your ROS2 nodes (found under `ros2_nodes/`) in your ROS2 workspace (e.g., `ros2_ws/src/`).
   - Build the workspace with:
     ```bash
     colcon build
     ```
   - Source the workspace:
     ```bash
     source ros2_ws/install/setup.bash
     ```

## Usage

### Operation Modes
- **Start-Up:** Once the Arduino firmware is running and the Python/ROS2 nodes are launched, the car initializes its sensors and begins processing data.
- **Teleoperated Mode (Joystick Control):**
  - Run the teleoperation ROS2 node on your master PC:
    ```bash
    ros2 launch your_ros_package teleop.launch.py
    ```
  - The node sends velocity commands over ROS2 to the Raspberry Pi, which relays them to the Arduino.
- **Fully Autonomous Mode:**
  - **Standalone Autonomous Mode:**  
    The Raspberry Pi acts as the master, processing sensor data (using SLAM and OpenCV) and directly controlling the car:
    ```bash
    ros2 launch your_ros_package autonomous.launch.py mode:=standalone
    ```
  - **Remote Autonomous Mode:**  
    An external master computer processes sensor data and sends control commands. The Raspberry Pi acts as an intermediary:
    ```bash
    ros2 launch your_ros_package autonomous.launch.py mode:=remote
    ```

### Monitoring
- Use ROS2 command line tools (e.g., `ros2 topic echo`, `ros2 topic list`) to monitor the data flow.
- Check the console output and log files for real-time logs, error messages, and debugging information.

## Project Structure
```
self-driving-car/
├── hardware/
│   ├── circuit_diagram.pdf     # Wiring diagrams and hardware setup instructions
│   └── parts_list.md           # List of hardware components (updated for new boards)
├── firmware/
│   ├── self_driving_car.ino    # Arduino firmware code for legacy board with WiFi module
│   └── self_driving_car_r4.ino # Updated firmware for Arduino Uno R4 WiFi Board
├── ros2_nodes/
│   ├── car_control_node.py     # ROS2 node for Arduino communication and car control
│   ├── teleop_joystick.py      # ROS2 node for joystick-based teleoperation
│   └── autonomous_mode.py      # ROS2 node integrating SLAM, OpenCV, and additional algorithms
├── software/
│   ├── self_driving_car.py     # Python control and processing code for additional testing
│   └── requirements.txt        # Python library dependencies
├── docs/
│   └── setup_guide.md          # Detailed project setup and installation guide
└── README.md                   # This file
```

## Circuit Diagram
For detailed wiring instructions, please refer to the `hardware/circuit_diagram.pdf` file in the repository.

## Troubleshooting
- **Serial Communication Issues:**
  - Ensure that the correct COM port is selected in both the Arduino IDE and Python/ROS2 configuration.
  - Verify that the baud rate in the Python script matches the one set in the Arduino firmware.
- **Sensor or Motor Malfunction:**
  - Check all hardware connections against the circuit diagram.
  - Confirm that sensors and motors are properly powered.
- **Camera or Vision Processing Errors:**
  - Make sure your camera drivers are correctly installed.
  - Verify the correct camera index in the Python code if multiple cameras are in use.
- **ROS2 Networking & Node Communication:**
  - Ensure all devices are on the same network and that `ROS_DOMAIN_ID` is consistently set across devices.

## Contributing
Contributions are welcome! Please follow these guidelines:
- Fork the repository and create a new branch for your feature or bug fix.
- Submit a pull request with a clear description of your changes.
- For major changes, open an issue first to discuss your ideas.

## License
Read the LICENCE.md
