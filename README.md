# SonicCar Documentation
## SonicCar: An Intelligent Voice-Controlled Autonomous Vehicle
<div align="center">
  <img src="https://github.com/user-attachments/assets/321f3eaf-28ea-4087-9bfc-1047f653198e" alt="SonicCar" style="width: 40%;">
</div>

In recent years, the field of autonomous vehicles has made significant strides, creating advancements in **artificial intelligence (AI)**, **sensor technologies**, and **deep learning** to redefine transportation. As part of UCSD’s [ECE/MAE148 (Introduction to Autonomous Vehicles)](https://ucsd-ecemae-148.github.io/) course, our team worked on the development of **SonicCar**, a state-of-the-art autonomous vehicle capable of responding to **voice commands** and navigating its environment intelligently and safely.

The SonicCar system integrates several innovative technologies to achieve its functionality. It uses **speech-to-text recognition** to convert spoken commands into actionable text, which is then processed by a **large language model (LLM)** to interpret and execute the instructions. This unique approach allows SonicCar to understand complex and natural language commands, providing a seamless user experience.

To ensure safety and compliance with road rules, SonicCar incorporates both **LiDAR-based obstacle detection** and a **camera-based stop sign recognition system**. The obstacle detection module enables the vehicle to detect and brake for objects in its path, while the stop sign recognition module, trained using **RoboFlow deep learning**, ensures adherence to traffic regulations. Together, these systems enhance the vehicle's reliability and situational awareness in dynamic environments.

The project represents a synthesis of voice control, AI-based natural language processing, and autonomous vehicle navigation, offering an accessible and interactive user interface while prioritizing safety. SonicCar showcases the potential for integrating **human-machine interaction technologies** with autonomous systems, paving the way for innovative applications in modern transportation.

# Project Goals and Timeline
- **Voice Command Integration:**  
  - Implemented a **speech-to-text recognition** system that transcribes spoken user commands, starting with the safe word "Sonic" in order to activate the system.
  - Integrated a **large language model (LLM)** to interpret and act on natural language commands, enabling the car to respond intuitively to a variety of inputs.  

- **Autonomous Navigation and Safety:**  
  - Integrated a **LiDAR-based obstacle detection system** to ensure the vehicle identifies and brakes for potential hazards in its path.  
  - Developed a **stop sign recognition module** using **RoboFlow deep learning**, ensuring the vehicle complies with traffic signals.  

- **Human-Machine Interaction:**  
  - Achieved smooth interaction between the user and vehicle through real-time processing of voice commands, improving accessibility and user experience.  

- **System Integration:**  
  - Combined inputs from multiple sensors (LiDAR and camera) and software systems (speech-to-text, LLM, deep learning) into a unified decision-making framework.  
  - Maintained consistent performance in diverse scenarios, showcasing the adaptability of the SonicCar system.  

# Mechanical Design
The mechanical design of SonicCar focused on creating custom components to support the functionality of the vehicle. These components were carefully designed, prototyped, and fabricated to ensure flexibility, durability, and compatibility with the onboard systems. Key mechanical elements include:  

- **Platform Design:**  
  - Designed and laser-cut a **custom platform** mounted on top of the car’s base to hold critical components such as the Jetson Nano, smart camera, LiDAR, and GPS.
  - Incorporated **multiple line spaces** in the platform’s design to allow flexibility in the positioning and installation of components.  

- **Camera Mount:**  
  - Developed a **3D-printed adjustable camera mount** to enable easy modification of the camera’s viewing angle.  
  - Integrated a **sunlight protection feature** into the mount to minimize interference from direct sunlight, ensuring consistent and reliable camera feed.
  
- **Other Components:**  
  - Designed and **3D-printed** multiple necessary components to hold every parts in place, including LiDAR mount and emergency stop button mount.

- **Fabrication Process:**  
  - Used **3D printing** to prototype and produce robust, lightweight parts.  
  - Applied **laser cutting techniques** to achieve precise and efficient fabrication of the platform with intricate design features.  

# Electronics
The SonicCar project utilizes a comprehensive set of electronic components to ensure reliable processing, communication, navigation, and power management. The key electronic parts include:  

- **Processing and Communication:**  
  - Jetson Nano  
  - USB Hub (for multiple connections)  
  - GPS Board and GPS Receiver  
  - Logitech Receiver (for console remote control)  
  - WiFi Dongle (to connect to local WiFi)  
  - Camera (Oak-D Lite)  

- **Sensing and Navigation:**  
  - LiDAR Board and LiDAR Sensor  

- **Actuation:**  
  - VESC (Electronic Speed Controller)  
  - DC Motor (XeRun 3660 G2)  
  - Servo PDB (Power Distribution Board)  
  - Servo Motor  

- **Power Management:**  
  - DC/DC Converter  
  - Anti-Spark Switch  
  - Emergency Stop Button  
  - Battery (3 Cell LiPo)  
  - Battery Voltage Checker  

These components are interconnected via a detailed wiring system, ensuring efficient power distribution, signal communication, and system reliability. The **wiring diagram** below provides an overview of these connections, serving as a guide for assembly and troubleshooting.  

![image](https://github.com/user-attachments/assets/8656e657-a212-436b-b657-306ff00b81a1)

# Programming
## Text-to-Speech


## Understanding Intent with LLMs


## Communication with ROS2


## LIDAR-based Collision Avoidance


## Stop Sign Detection
