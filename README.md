# SonicCar: An Intelligent Voice-Controlled Autonomous Vehicle
<div align="center">
  <img src="https://github.com/user-attachments/assets/321f3eaf-28ea-4087-9bfc-1047f653198e" alt="SonicCar" style="width: 40%">
</div>

In recent years, the field of autonomous vehicles has made significant strides, creating advancements in **artificial intelligence (AI)**, **sensor technologies**, and **deep learning** to redefine transportation. As part of UCSD’s [ECE/MAE148 (Introduction to Autonomous Vehicles)](https://ucsd-ecemae-148.github.io/) course, our team worked on the development of **SonicCar**, a state-of-the-art autonomous vehicle capable of responding to **voice commands** and navigating its environment intelligently and safely.

The SonicCar system integrates several innovative technologies to achieve its functionality. It uses **speech-to-text recognition** to convert spoken commands into actionable text, which is then processed by a **large language model (LLM)** to interpret and execute the instructions. This unique approach allows SonicCar to understand complex and natural language commands, providing a seamless user experience.

To ensure safety and compliance with road rules, SonicCar incorporates both **LiDAR-based obstacle detection** and a **camera-based stop sign recognition system**. The obstacle detection module enables the vehicle to detect and brake for objects in its path, while the stop sign recognition module, trained using **RoboFlow deep learning**, ensures adherence to traffic regulations. Together, these systems enhance the vehicle's reliability and situational awareness in dynamic environments.

The project represents a synthesis of voice control, AI-based natural language processing, and autonomous vehicle navigation, offering an accessible and interactive user interface while prioritizing safety. SonicCar showcases the potential for integrating **human-machine interaction technologies** with autonomous systems, paving the way for innovative applications in modern transportation.

## Project Goals and Timeline
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

<div align="center">
  <img src="https://github.com/user-attachments/assets/53bfc4e6-5f2d-4a59-af19-3724c1d3ac05" alt="Car" height="350">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  <img src="https://github.com/user-attachments/assets/251a9552-a5dc-4f6a-830d-e0891b5de420" alt="Car" height="350">
</div>


## Mechanical Design
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

## Programming
### Overview
The chart below shows how the software is structured. Fundamentally, we are using ROS2, especially to provide communication (the blue boxes in the chart represent ROS2 nodes). A high-level overview over the components:
- **Publisher**: Runs on a laptop, uses the laptop's microphone to turn voice commands into text (speech-to-text), then uses a Large Language Model to understand the intent of the text and translates it into commands for the car steering angle, throttle and the runtime (how long a command should be executed). It then publishes these commands to the `steering_commands` topics. This is also where the graphical user interface runs.
- **Subscriber**: Runs on the Jetson Nano, listens to incoming commands on the `steering_commands` topic and publishes them to the VESC node. Also tracks the LIDAR to prevent collisions with appearing objects and uses the OAK-D camera to detect stop signs.
- **VESC node**: Prewritten node from DonkeyCar, tranlates commands on the `/cmd_vel` topic to electronic signals to the servo motor (for steering) and DC motor (for throttle).
- **LIDAR node**: Prewritten node from DonkeyCar, publishes LIDAR measurements to the `/scan` topic.
- **FastDDS discovery server**: Running on the Jetson, used to enable communication between devices (laptop and Jetson). All ROS2 nodes register with the discovery server so they are discoverable to all other nodes.
- **OAK-D Camera**: Directly connected to the Jetson via USB, runs the stop sign detection AI model.

![Software Overview](https://github.com/user-attachments/assets/33715aae-859f-4e02-af10-2c55a63c8c86)

### Speech-to-Text (STT)
For understanding voice commands, we leverage the microphone of the laptop so the user does not have to move along the Jetson Nano. We use the Python package `SpeechRecognition` and concretely, the underlying Google Speech Recognition API, to get the command spoken as text. We typically saw latencies of 400-600 ms, depending on the network connection. Note that the `SpeechRecognition` uses a hardcoded API key for the API and there is a limit on the number of requests you can do per day.

The code listens for 4 seconds (by default) and then sends off anything recorded to the API.

### Understanding Intent with an LLM
- Gemini API
- System Prompt
- Output Formatting
- Text processing
- Case handling (if-else)

## Graphical User Interface (GUI)
The GUI is consists of a button, a status text box and a timeout counter text box. To interact with it, the user clicks the Start Recording button. After talking, the audio is transcribed and intent understood (as described above). The understood intent is then shown on the user interface. In case there are any errors, they are also shown in the GUI. After the command was sent to the Jetson, a countdown starts indicating how much longer the command will be executed on the Jetson.

Behind the scenes, the GUI leverages the Python `tkinter` package and is launched from the ROS2 publisher node and keeps running continuously. It is in the `graphical_user_interface.py` script and called the `VoiceRecorderUI`.

<img src="https://github.com/user-attachments/assets/238156ba-1345-4412-b457-d4bc628c34c9" alt="gui" style="width: 40%">

## Communication with ROS2
Because our ROS2 nodes had to communicate across devices (laptop and Jetson), we had some unique challenges with making the nodes talk to each other. In theory, ROS2 should automatically discover all nodes running on the same WiFi network. Unfortunately, this did not work for us. We found a workaround by setting up out own [FastDDS discovery server](https://fast-dds.docs.eprosima.com/en/v2.14.3/fastdds/discovery/discovery_server.html). We have this server running at port `11888` on the Jetson in a separate terminal window. Then, on the laptop and all terminals where we run ROS2 nodes, we set an environment variable for the discovery server IP and port: `export ROS_DISCOVERY_SERVER="[YOUR-IP]:[YOUR-PORT]"` (on the Windows command line, this is `set ROS_DISCOVERY_SERVER=[YOUR-IP]:[YOUR-PORT]`). If this variable is set, ROS2 (more specifically FastDDS) automatically uses the server to discover nodes.

One interesting issue we had: Instead of the IP address (which changes regularly), we wanted to use the Jetson's fixed hostname, `ucsdrobocar-148-12`. However, this resulted in errors since IPv4 hostnames apparently [can only include one dash](https://www.noip.com/support/knowledgebase/what-is-a-valid-hostname). To save time, we did not investigate this further, though there is likely a workaround. Instead, we regularly run `hostname -I` on the Jetson to see the current IP address.

### LIDAR-based Collision Avoidance
- Subscribing to `/scan`
- Filtering to only front range
- Comparing to minimum allowed distance
- Stopping car

### Stop Sign Detection
- Data collection, labeling, training in Roboflow (talk about data set and accuracy)
    - Dataset: Got 453 labelled stop sign images from Roboflow Universe, augmented with cropping, changing hue and changing brightness to make it more robust
    - Training results: mAP (mean average precision; average precision over all classes); precision (how often the model is correct), recall (what percentage of relevant labels were successfully identified)
- Direct deployment to OAK-D via `roboflowoak` Python package --> running on OAK-D
- Detection with certain confidence (90% threshold)

![stop_sign_dataset](https://github.com/user-attachments/assets/a61879c2-2880-4ca9-a382-4e58883f334c)
![stop_sign_detection_model_evaluation](https://github.com/user-attachments/assets/13799ae4-6e4d-4efa-b6ad-7a7ab2079f13)

- Compare distance (not working yet): The way we currently retrieve results from OAK-D gives faulty values for the distance (but not a big deal)
- Stopping car

## How to Run (step-by-step)
Anytime there is a variable here, remove the square brackets as well.

On the Jetson:

0. Make sure the Jetson is turned on. Wait until the fan starts spinning (pretty good indication on when it has connected to the Wi-Fi. Make sure your laptop is also connected to the `UCSDRoboCar` network. Then, SSH into the Jetson: `ssh jetson@ucsdrobocar-148-[TEAM-NR].local`
1. Get the UCSD DonkeyCar docker image: `docker pull djnighti/ucsd_robocar:devel`
2. Start Docker container: `docker start [CONTAINER-NAME]`
3. Execute bash on container `docker exec -it [CONTAINER-NAME] bash`
4. Source ROS2: `source_ros2`
5. The first time, navigate to the `src` folder and clone the repository `git clone https://github.com/dglttr/MAE148FinalProject.git`. Afterwards, remember to `git pull` and rebuild the package before using it to have the latest changes (`colcon build --merge-install --packages-select listening_bot`).
6. Source ROS2 package: `. /install/setup.bash`
7. Check the current IP address of the Jetson (needed for discovery server later): `hostname -I`
8. Launch FastDDS discovery server: `fastdds discovery --server-id 0 --port 11888`
9. Now, open another terminal and repeat steps 0, 3, 4 and 6 - this will be used to launch the VESC and LIDAR nodes
10. Set environment variable for FastDDS server: `export ROS_DISCOVERY_SERVER="[IP-ADDRESS]:11888"`
12. Stop ROS2 daemon to make sure the discovery server will be used: `ros2 daemon stop`
13. Launch LIDAR and VESC nodes with launchfile: `ros2 launch listening_bot listening_bot.launch.py`
14. Now, open another terminal and repeat steps 0, 3, 4, 6 and 10 - this will be used to launch the subscriber node (we do this in a separate terminal so the terminal is not flooded with messages from the VESC and LIDAR nodes)
15. Set Roboflow environment variable to get the stop sign detection model: `export ROBOFLOW_API_KEY="[YOUR-ROBOFLOW-API-KEY]"`
16. Run subscriber: `ros2 run listening_bot subscriber`

On Laptop (this is on a Windows laptop):

0. First, you will need to install ROS2. We used ROS2 Foxy. Newer ROS2 versions may work, but we did not test that. Here are the [install instructions for Windows](https://docs.ros.org/en/foxy/Installation/Windows-Install-Binary.html).
1. The first time, clone the listening_bot repository into a folder of your choosing: `git clone https://github.com/dglttr/MAE148FinalProject.git`. Afterwards, navigate to that folder: `cd [PATH-TO-FOLDER]`.
2. `git pull` to make sure you are on the newest version.
3. Build package: `colcon build --merge-install --packages-select listening_bot`
4. Source package. On Windows, it works like this: `call install/setup.bat` 
5. Set environment variable for FastDDS server: `set ROS_DISCOVERY_SERVER=[JETSON-IP-ADDRESS]:11888`
6. `ros2 daemon stop`
7. Set the LLM_API_KEY: `set LLM_API_KEY=[API-KEY]`. You can get the API key for the Google Gemini API [here](https://aistudio.google.com/apikey).
8. Run the publisher: `ros2 run publisher`
