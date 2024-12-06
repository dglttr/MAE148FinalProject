# SonicCar: An Intelligent Voice-Controlled Autonomous Vehicle
<div align="center">
  <img src="https://github.com/user-attachments/assets/321f3eaf-28ea-4087-9bfc-1047f653198e" alt="SonicCar" style="width: 40%">
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

<div align="center" style="border: 2px solid black; display: inline-block;">
  <img src="https://github.com/user-attachments/assets/53bfc4e6-5f2d-4a59-af19-3724c1d3ac05" alt="Car" height="350">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  <img src="https://github.com/user-attachments/assets/251a9552-a5dc-4f6a-830d-e0891b5de420" alt="Car" height="350">
</div>


# Mechanical Design
The mechanical design of SonicCar focused on creating custom components to support the functionality of the vehicle. These components were designed, prototyped, and fabricated to ensure flexibility, durability, and compatibility with the onboard systems. Key mechanical elements include:  

- **Platform:**  
  - Designed and laser-cut a **custom platform** mounted on top of the car’s base to hold critical components such as the Jetson Nano, camera, LiDAR, and GPS.
  - The platform uses **multiple parallel slots** to allow flexibility in the positioning and installation of components.
  - It also has four pins that connect it to the car body. They are simple, sturdy and very flexible (for example, we simply adjusted the length and reprinted the feet when we needed to raise the platform).
<div align="center">
  <img src="https://github.com/user-attachments/assets/1c2f31f1-a217-452b-93ae-7ebe3370926d" alt="Car Platform" height="350">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  <img src="https://github.com/user-attachments/assets/819c9ce5-c165-415d-8f91-24416138c714" alt="Platform Pins" height="350">
</div>

- **Camera Mount:**  
  - Developed a **3D-printed adjustable camera mount** to enable easy modification of the camera’s viewing angle.  
  - Integrated a ledge for **sunlight protection** to minimize interference from direct sunlight, ensuring a reliable camera feed.
<div align="center">
  <img src="https://github.com/user-attachments/assets/5b0900c5-bfa2-4a3d-b747-38d66222ca1e" alt="Camera Mount" height="350">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  <img src="https://github.com/user-attachments/assets/51780adc-a634-4045-9227-05bc99ba7b4b" alt="Camera Mount" height="350">
</div>

- **Anti-Spark Switch mount:**  
  - Mount to hold the Anti-Spark Switch in place.
<div align="center">
  <img src="https://github.com/user-attachments/assets/690cc514-0458-4111-a6ab-28c815f4038a" alt="Anti-Spark Switch Mount" height="350">
</div>

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
<div align="center">
  <img src="https://github.com/user-attachments/assets/8656e657-a212-436b-b657-306ff00b81a1" style="width: 75%">
</div>

# Programming
## Overview
The chart below shows how the software is structured. Fundamentally, we are using ROS2, especially to provide communication (the blue boxes in the chart represent ROS2 nodes). A high-level overview over the components:
- **Publisher**: Runs on a laptop, uses the laptop's microphone to turn voice commands into text (speech-to-text), then uses a Large Language Model to understand the intent of the text and translates it into commands for the car steering angle, throttle and the runtime (how long a command should be executed). It then publishes these commands to the `steering_commands` topics. This is also where the graphical user interface runs.
- **Subscriber**: Runs on the Jetson Nano, listens to incoming commands on the `steering_commands` topic and publishes them to the VESC node. Also tracks the LIDAR to prevent collisions with appearing objects and uses the OAK-D camera to detect stop signs.
- **VESC node**: Prewritten node from DonkeyCar, tranlates commands on the `/cmd_vel` topic to electronic signals to the servo motor (for steering) and DC motor (for throttle).
- **LIDAR node**: Prewritten node from DonkeyCar, publishes LIDAR measurements to the `/scan` topic.
- **FastDDS discovery server**: Running on the Jetson, used to enable communication between devices (laptop and Jetson). All ROS2 nodes register with the discovery server so they are discoverable to all other nodes.
- **OAK-D Camera**: Directly connected to the Jetson via USB, runs the stop sign detection AI model.

![Software Overview](https://github.com/user-attachments/assets/33715aae-859f-4e02-af10-2c55a63c8c86)

## Speech-to-Text (STT)
For understanding voice commands, we leverage the microphone of the laptop so the user does not have to move along the Jetson Nano. We use the Python package `SpeechRecognition` and concretely, the underlying Google Speech Recognition API, to get the command spoken as text. We typically saw latencies of 400-600 ms, depending on the network connection.

The code listens for 4 seconds (by default) and then sends off anything recorded to the API.

## Understanding Intent with an LLM
- Gemini API
- System Prompt
- Output Formatting
- Text processing
- Case handling (if-else)

## Graphical User Interface
- TODO: Add screenshots
- Explain tkinter
- Explain relation to publisher node (launched via timer, calls some functions, ...)

## Communication with ROS2
- Explain FastDDS discovery server
- Hostname did not work (potentially helpful links)

## LIDAR-based Collision Avoidance
- Subscribing to `/scan`
- Filtering to only front range
- Comparing to minimum allowed distance
- Stopping car

## Stop Sign Detection
- Data collection, labeling, training in Roboflow
- Direct deployment to OAK-D via `roboflowoak` Python package --> running on OAK-D
- Detection with certain confidence
- Latency?
- Compare distance (not working yet)
- Stopping car
