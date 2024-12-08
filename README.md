<div id="top"></div>

<h1 align="center">SonicCar: An Intelligent Voice-Controlled Autonomous Vehicle</h1>
<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://jacobsschool.ucsd.edu/">
    <img src="https://github.com/user-attachments/assets/95dfc96f-5263-4656-88a9-515fe4e0930c" alt="Logo" width="350">
  </a>
<h3>ECE/MAE 148 Final Project</h3>
<p>
Team 12 Fall 2024
</p>

<img src="https://github.com/user-attachments/assets/321f3eaf-28ea-4087-9bfc-1047f653198e" alt="SonicCar" style="width: 40%">
</div>
<br>

In recent years, the field of autonomous vehicles has made significant strides, creating advancements in **artificial intelligence (AI)**, **sensor technologies**, and **deep learning** to redefine transportation. As part of UCSD’s [ECE/MAE148 (Introduction to Autonomous Vehicles)](https://ucsd-ecemae-148.github.io/) course, our team worked on the development of **SonicCar**, a state-of-the-art autonomous vehicle capable of responding to **voice commands** and navigating its environment intelligently and safely.

The SonicCar system integrates several innovative technologies to achieve its functionality. It uses **speech-to-text recognition** to convert spoken commands into actionable text, which is then processed by a **large language model (LLM)** to interpret and execute the instructions. This unique approach allows SonicCar to understand complex and natural language commands, providing a seamless user experience.

To ensure safety and compliance with road rules, SonicCar incorporates both **Li-DAR-based obstacle detection** and a **camera-based stop sign recognition system**. The obstacle detection module enables the vehicle to detect and brake for objects in its path, while the stop sign recognition module, trained using **RoboFlow deep learning**, ensures adherence to traffic regulations. Together, these systems enhance the vehicle's reliability and situational awareness in dynamic environments.

The project represents a synthesis of voice control, AI-based natural language processing, and autonomous vehicle navigation, offering an accessible and interactive user interface while prioritizing safety. SonicCar showcases the potential for integrating **human-machine interaction technologies** with autonomous systems, paving the way for innovative applications in modern transportation.

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#team-members">Team Members</a></li>
    <li><a href="#project-goals-and-timeline">Project Goals and Timeline</a></li>
    <li><a href="#mechanical-design">Mechanical Design</a></li>
    <li><a href="#electronics">Electronics</a></li>
    <li><a href="#programming">Programming</a>
      <ul>
        <li><a href="#speech-to-text-stt">Speech-to-Text (STT)</a></li>
        <li><a href="#understanding-intent-with-an-llm">Understanding Intent with an LLM</a></li>
        <li><a href="#graphical-user-interface-gui">Graphical User Interface (GUI)</a></li>
        <li><a href="#communication-with-ros2">Communication with ROS2</a></li>
        <li><a href="#li-dar-based-collision-avoidance">Li-DAR-based Collision Avoidance</a></li>
        <li><a href="#stop-sign-detection">Stop Sign Detection</a></li>
      </ul>
    </li>
    <li><a href="#how-to-run-step-by-step">How to Run (Step-by-Step)</a></li>
    <li><a href="#authors">Authors</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
    <li><a href="#contacts">Contacts</a></li>
  </ol>
</details>

<!-- TEAM MEMBERS -->
## Team Members
<ul>
  <li>Nick Ji - Electrical Engineering - Class of 2026</li>
  <li>Johnny Li - Mechanical Engineering - Class of 2025</li>
  <li>Daniel Glatter - Mechanical Engineering - Class of 2025</li>
  <li>Shivharsh Kand - Mechanical Engineering - Class of 2025</li>
</ul>

## Project Goals and Timeline
**Project Goals**:
The project aimed to design and implement a sophisticated autonomous vehicle system with the following objectives:

- **Voice Command Integration**: Developing a speech-to-text system to process natural language spoken commands, starting with the safe word "Sonic" in order to activate the system. Aditionally, we employed a large language model (LLM) to translate conversational speech into structured commands. This innovative approach allows the vehicle to understand natural speech patterns, ensuring a human-like interaction and enhancing usability.
- **Human-Machine Interaction**: Fostering smooth communication between the user and vehicle by enabling the system to interpret natural speech patterns, thereby improving accessibility, enhancing the user experience, and allowing intuitive vehicle control.
- **LIDAR-based Collision Avoidance**: Integrating a robust LIDAR-based obstacle detection system to ensure the vehicle detects and avoids hazards in real time, prioritizing safety and navigation reliability.

**Project Scope**:
The project extended its functionality beyond the initial goals with the following additions:

- **Stop Sign Detection**: Implementing a deep-learning-powered stop sign recognition system, utilizing the OAK-D camera to detect stop signs with high confidence, thereby improving situational awareness.
- **Graphical User Interface (GUI)**: Developing an intuitive GUI that allows users to monitor critical vehicle parameters in real time, such as throttle, steering angle, direction, and timeout values. The GUI enhances user understanding of current vehicle operations and provides clarity during system interaction.

The expanded scope reflects the adaptability and robustness of the system, showcasing its ability to integrate additional features seamlessly into its existing architecture. The project's innovative approach, integrating AI-driven conversational capabilities, sensor technologies, and interactive design to create a versatile, safe, and intuitive autonomous vehicle system. These features showcase the potential for real-world deployment in modern transportation systems.

**Timeline**:


## Mechanical Design

<div align="center">
  <img src="https://github.com/user-attachments/assets/53bfc4e6-5f2d-4a59-af19-3724c1d3ac05" alt="Car" height="300">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  <img src="https://github.com/user-attachments/assets/251a9552-a5dc-4f6a-830d-e0891b5de420" alt="Car" height="300">
</div>

The mechanical design of SonicCar focused on creating custom components to support the functionality of the vehicle. These components were designed, prototyped, and fabricated to ensure flexibility, durability, and compatibility with the onboard systems. Key mechanical elements include:  

- **Platform:**  
  - Designed and laser-cut a **custom platform** mounted on top of the car’s base to hold critical components such as the Jetson Nano, camera, Li-DAR, and GPS.
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

## Electronics
The SonicCar project utilizes a comprehensive set of electronic components to ensure reliable processing, communication, navigation, and power management. The key electronic parts include:  

- **Processing and Communication:**  
  - Jetson Nano  
  - USB Hub (for multiple connections)  
  - GPS Board and GPS Receiver  
  - Logitech Receiver (for console remote control)  
  - WiFi Dongle (to connect to local WiFi)  
  - Camera (Oak-D Lite)  

- **Sensing and Navigation:**  
  - Li-DAR Board and Li-DAR Sensor  

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

## Programming
### Overview
The chart below shows how the software is structured. Fundamentally, we are using ROS2, especially to provide communication (the blue boxes in the chart represent ROS2 nodes). A high-level overview over the components:
- **Publisher**: Runs on a laptop, uses the laptop's microphone to turn voice commands into text (speech-to-text), then uses a Large Language Model to understand the intent of the text and translates it into commands for the car steering angle, throttle and the runtime (how long a command should be executed). It then publishes these commands to the `steering_commands` topics. This is also where the graphical user interface runs.
- **Subscriber**: Runs on the Jetson Nano, listens to incoming commands on the `steering_commands` topic and publishes them to the VESC node. Also tracks the Li-DAR to prevent collisions with appearing objects and uses the OAK-D camera to detect stop signs.
- **VESC node**: Prewritten node from DonkeyCar, tranlates commands on the `/cmd_vel` topic to electronic signals to the servo motor (for steering) and DC motor (for throttle).
- **Li-DAR node**: Prewritten node from DonkeyCar, publishes Li-DAR measurements to the `/scan` topic.
- **FastDDS discovery server**: Running on the Jetson, used to enable communication between devices (laptop and Jetson). All ROS2 nodes register with the discovery server so they are discoverable to all other nodes.
- **OAK-D Camera**: Directly connected to the Jetson via USB, runs the stop sign detection AI model.

<div align="center">
  <img src="https://github.com/user-attachments/assets/33715aae-859f-4e02-af10-2c55a63c8c86" alt="Program Overview" style="width: 70%">
</div>

### Speech-to-Text (STT)
For understanding voice commands, we leverage the microphone of the laptop so the user does not have to move along the Jetson Nano. We use the Python package `SpeechRecognition` and concretely, the underlying Google Speech Recognition API, to get the command spoken as text. We typically saw latencies of 400-600 ms, depending on the network connection. Note that the `SpeechRecognition` uses a hardcoded API key for the API and there is a limit on the number of requests you can do per day.

The code listens for 4 seconds (by default) and then sends off anything recorded to the API.

### Understanding Intent with an LLM
The python script used for this project was created using a publisher and subscriber method. The laptop handles speech recognition using a microphone and will extract three variables, direction (angle), speed (ranging from 0 to 1), and the amount of time to do such actions. When the three values have been collected, it is converted from speech to text through Large Language Models (LLMs). This is incredibly useful in this scenario for translating speech into tokens which can be used as key values for the three variables. An additional LLMs systems prompt was added to “train” the AI to look for the three values. This is where the Gemini API system comes into play.
	An additional Gemini API system was used to configure LLMs through system prompts that define their behavior and context, ensuring they respond appropriately to user inputs. LLMs play a crucial role in transforming natural language commands into actionable outputs. Through system prompts, the LLM can be tailored to interpret specific driving instructions like "Go straight" or "Turn left" with higher accuracy. Its text processing capabilities enable tokenization and intent recognition, ensuring that each command is parsed and converted into structured data. This enables the car's control systems to be executed easier. LLMs ensure commands are delivered in a consistent, machine-readable format, while case handling logic allows it to respond appropriately to varied inputs, such as distinguishing between movement, stopping, or error cases. This integration streamlines the communication between the user's voice commands and the car's control systems, making the interaction intuitive, precise, and reliable.
TODO: Add screenshot of LLM prompts
### Graphical User Interface (GUI)
The GUI is consists of a button, a status text box and a timeout counter text box. To interact with it, the user clicks the Start Recording button. After talking, the audio is transcribed and intent understood (as described above). The understood intent is then shown on the user interface. In case there are any errors, they are also shown in the GUI. After the command was sent to the Jetson, a countdown starts indicating how much longer the command will be executed on the Jetson.

Behind the scenes, the GUI leverages the Python `tkinter` package and is launched from the ROS2 publisher node and keeps running continuously. It is in the `graphical_user_interface.py` script and called the `VoiceRecorderUI`.

<div align="center">
  <img src="https://github.com/user-attachments/assets/238156ba-1345-4412-b457-d4bc628c34c9" alt="GUI" style="width: 40%">
</div>

### Communication with ROS2
Because our ROS2 nodes had to communicate across devices (laptop and Jetson), we had some unique challenges with making the nodes talk to each other. In theory, ROS2 should automatically discover all nodes running on the same WiFi network. Unfortunately, this did not work for us. We found a workaround by setting up out own [FastDDS discovery server](https://fast-dds.docs.eprosima.com/en/v2.14.3/fastdds/discovery/discovery_server.html). We have this server running at port `11888` on the Jetson in a separate terminal window. Then, on the laptop and all terminals where we run ROS2 nodes, we set an environment variable for the discovery server IP and port: `export ROS_DISCOVERY_SERVER="[YOUR-IP]:[YOUR-PORT]"` (on the Windows command line, this is `set ROS_DISCOVERY_SERVER=[YOUR-IP]:[YOUR-PORT]`). If this variable is set, ROS2 (more specifically FastDDS) automatically uses the server to discover nodes.

One interesting issue we had: Instead of the IP address (which changes regularly), we wanted to use the Jetson's fixed hostname, `ucsdrobocar-148-12`. However, this resulted in errors since IPv4 hostnames apparently [can only include one dash](https://www.noip.com/support/knowledgebase/what-is-a-valid-hostname). To save time, we did not investigate this further, though there is likely a workaround. Instead, we regularly run `hostname -I` on the Jetson to see the current IP address.

### Li-DAR-based Collision Avoidance
The Li-DAR-based collision avoidance system utilizes the Jetson Nano and ROS2 for seamless integration with various hardware and software components to ensure safe and autonomous vehicle operation. Below is a detailed breakdown of the components and their roles:
- **Li-DAR Node**: Subscribes to the `/scan` topic, which publishes Li-DAR measurements. The system filters the range data to focus on the front third of the vehicle's field of view and calculates the minimum distance to nearby objects. If the minimum detected distance is less than the configured threshold `(0.4 meters)`, the car is stopped immediately to avoid a collision. This ensures obstacle detection and prevention in real time.
- **Subscriber Node**: This node subscribes to the `steering_commands` topic to receive motion commands and to the `/scan` topic for Li-DAR data. It compares the received front-range Li-DAR data to a predefined minimum allowed distance. Commands are published to the `/cmd_vel` topic to control steering and throttle. Additionally, a timer mechanism is employed to maintain movement or stop the vehicle after a timeout if no further commands are received.
- **Collision Avoidance Logic**: The system filters the Li-DAR data from 10% to 40% of the total range to focus on obstacles directly ahead. The `min()` function is used to find the closest object in this range, triggering a stop if the object is too close. This logic is encapsulated in the `Li-DAR_callback` method.
- **Command Integration**: Commands such as steering angles and throttle values are processed using the `command_callback` function. A timer (`keep_moving`) ensures the car continues moving until a timeout occurs or a new command is received.
- **Real-time Actuation**: All decisions, including stopping due to obstacles or stop sign detection (further discussed in the next section), are immediately transmitted to the VESC node through the `/cmd_vel` topic. This ensures the vehicle reacts promptly to its environment.

### Stop Sign Detection
Roboflow streamlines the process of building a robust stop sign detection model by facilitating data collection, labeling, training, and deployment. The dataset used for this project was sourced from Roboflow, which contained 453 labeled stop sign images. These diverse images simulate varied real-world conditions, improving the model's ability to generalize across different environments. The original plan with this was to integrate Roboflow as a secondary emergency stop feature. However, we ran into issues as the way we retrieve results from OAK-D gives faulty values for distance. This is no big deal as originally, we planned on Sonic to stop one meter away from the stop sign but the minimum it can stop was 3 meters. 

Training results were evaluated using key metrics such as mean Average Precision (mAP), which measures precision across all classes, ensuring balanced performance. Precision reflects how often the model's predictions are correct, while recall indicates the percentage of relevant labels successfully identified by the model. These metrics highlight the model's effectiveness and help fine-tune its performance.

The trained model was deployed to the OAK-D camera using the Python package roboflowOAK. This integration enables real-time stop sign detection directly on the OAK-D. A confidence threshold of 90% ensures reliable detections, minimizing false positives while maintaining responsiveness. The reason it was bumped to 90% instead of 80% was because it kept viewing other people as stop signs. The combination of Roboflow's robust tools and the OAK-D's hardware efficiency delivers an optimized solution for stop sign detection.

<div align="center">
  <img src="https://github.com/user-attachments/assets/a61879c2-2880-4ca9-a382-4e58883f334c" alt="stop_sign_dataset" style="width: 70%">
  <img src="https://github.com/user-attachments/assets/13799ae4-6e4d-4efa-b6ad-7a7ab2079f13" alt="stop_sign_detection_model_evaluation" style="width: 70%">
</div>

## How to Run (step-by-step)
Anytime there is a variable here, remove the square brackets as well.

### On the Jetson:

0. Make sure the Jetson is turned on. Wait until the fan starts spinning (pretty good indication on when it has connected to the Wi-Fi. Make sure your laptop is also connected to the `UCSDRoboCar` network. Then, SSH into the Jetson: `ssh jetson@ucsdrobocar-148-[TEAM-NR].local`
1. Get the UCSD DonkeyCar docker image: `docker pull djnighti/ucsd_robocar:devel`
2. Start Docker container: `docker start [CONTAINER-NAME]`
3. Execute bash on container `docker exec -it [CONTAINER-NAME] bash`
4. Source ROS2: `source_ros2`
5. The first time, navigate to the `src` folder and clone the repository `git clone https://github.com/dglttr/MAE148FinalProject.git`. Afterwards, remember to `git pull` and rebuild the package before using it to have the latest changes (`colcon build --merge-install --packages-select listening_bot`).
6. Source ROS2 package: `. /install/setup.bash`
7. Check the current IP address of the Jetson (needed for discovery server later): `hostname -I`
8. Launch FastDDS discovery server: `fastdds discovery --server-id 0 --port 11888`
9. Now, open another terminal and repeat steps 0, 3, 4 and 6 - this will be used to launch the VESC and Li-DAR nodes
10. Set environment variable for FastDDS server: `export ROS_DISCOVERY_SERVER="[IP-ADDRESS]:11888"`
12. Stop ROS2 daemon to make sure the discovery server will be used: `ros2 daemon stop`
13. Launch Li-DAR and VESC nodes with launchfile: `ros2 launch listening_bot listening_bot.launch.py`
14. Now, open another terminal and repeat steps 0, 3, 4, 6 and 10 - this will be used to launch the subscriber node (we do this in a separate terminal so the terminal is not flooded with messages from the VESC and Li-DAR nodes)
15. Set Roboflow environment variable to get the stop sign detection model: `export ROBOFLOW_API_KEY="[YOUR-ROBOFLOW-API-KEY]"`
16. Run subscriber: `ros2 run listening_bot subscriber`

### On PC/Laptop (Windows):

0. First, you will need to install ROS2. We used ROS2 Foxy. Newer ROS2 versions may work, but we did not test that. Here are the [install instructions for Windows](https://docs.ros.org/en/foxy/Installation/Windows-Install-Binary.html).
1. The first time, clone the listening_bot repository into a folder of your choosing: `git clone https://github.com/dglttr/MAE148FinalProject.git`. Afterwards, navigate to that folder: `cd [PATH-TO-FOLDER]`.
2. `git pull` to make sure you are on the newest version.
3. Build package: `colcon build --merge-install --packages-select listening_bot`
4. Source package. On Windows, it works like this: `call install/setup.bat` 
5. Set environment variable for FastDDS server: `set ROS_DISCOVERY_SERVER=[JETSON-IP-ADDRESS]:11888`
6. `ros2 daemon stop`
7. Set the LLM_API_KEY: `set LLM_API_KEY=[API-KEY]`. You can get the API key for the Google Gemini API [here](https://aistudio.google.com/apikey).
8. Run the publisher: `ros2 run publisher`

<!-- Authors -->
## Authors
Nick, Daniel, Johnny, Shiv

<!-- Badges -->
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments
*Huge thanks to the team, the TAs Alexander and Winston for carrying the class, and Professor Jack Silberman for the amazing opportunity to work on such a cool project! Special thanks to Alexander for the README template!*

<!-- CONTACTS -->
## Contacts

* Nick | yji@ieee.org
* Daniel | dglatter@ucsd.edu 
* Johnny | jol048@ucsd.edu
* Shiv | skand@ucsd.edu
