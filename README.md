# SonicCar Documentation
## SonicCar: An Intelligent Voice-Controlled Autonomous Vehicle
![image](https://github.com/user-attachments/assets/8d5669b3-a494-4ba1-876c-e1942d67e9fd)

In recent years, the field of autonomous vehicles has made significant strides, leveraging advancements in artificial intelligence (AI), sensor technologies, and deep learning to redefine transportation. As part of UCSD’s [ECE/MAE148 (Introduction to Autonomous Vehicles)](https://ucsd-ecemae-148.github.io/) course, our team worked on the development of SonicCar, a state-of-the-art autonomous vehicle capable of responding to voice commands and navigating its environment intelligently and safely.

The SonicCar system integrates several innovative technologies to achieve its functionality. It uses speech-to-text recognition to convert spoken commands into actionable text, which is then processed by a large language model (LLM) to interpret and execute the instructions. This unique approach allows SonicCar to understand complex and natural language commands, providing a seamless user experience.

To ensure safety and compliance with road rules, SonicCar incorporates both LiDAR-based obstacle detection and a camera-based stop sign recognition system. The obstacle detection module enables the vehicle to detect and brake for objects in its path, while the stop sign recognition module, trained using RoboFlow deep learning, ensures adherence to traffic regulations. Together, these systems enhance the vehicle's reliability and situational awareness in dynamic environments.

The project represents a synthesis of voice control, AI-based natural language processing, and autonomous vehicle navigation, offering an accessible and interactive user interface while prioritizing safety. SonicCar showcases the potential for integrating human-machine interaction technologies with autonomous systems, paving the way for innovative applications in modern transportation.

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
Fixture (mounting feet), wooden board design, camera mount

# Electronics
List components and show wiring diagram
![image](https://github.com/user-attachments/assets/8656e657-a212-436b-b657-306ff00b81a1)

# Programming
## Text-to-Speech


## Understanding Intent with LLMs


## Communication with ROS2


## LIDAR-based Collision Avoidance


## Stop Sign Detection
