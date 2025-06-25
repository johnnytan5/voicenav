# VoiceNav

VoiceNav is a ROS- and Python-based voice-activated assistant designed to empower visually impaired individuals through speech-based interaction, real-time camera vision, and responsive feedback via audio, visual overlays, and a web interface. Its core goal is to enable users to navigate their surroundings independently, with minimal external assistance, by providing scene descriptions and timely warnings about nearby obstacles or important environmental cues. In critical situations, VoiceNav also offers a fallback mechanism that allows friends or family members to step in and provide remote assistance when needed most.


## Demo

[![Watch the demo](https://img.shields.io/badge/▶️-Watch%20Demo-red?logo=youtube&style=for-the-badge)](https://youtu.be/h3Y3T1bBZJU)

---

## Features

-  **Speech Recognition** : Use local microphone to capture speech and process with SpeechRecognition Model
-  **Text‑to‑Speech (TTS)** : Output voice from text using your system speaker
-  **Obstacle Detection**: Detects obstacles and emits warning
-  **Depth Camera Sensor**: Uses astra depth camera modules to detect depth information
-  **Scene Descriptor**: Captures image, process and describe the scene
-  **Information Extraction**: Helps visually impaired individuals extract expiry and nutrition information 
-  **Flask Web Interface** : Stream video, 2-way audio communication and logs in real time
-  **Email** : Supports email on startup and on-demand email with public address to web interface for remote help

Web Interface:

<img src="https://innvejlnclxonnanhtbf.supabase.co/storage/v1/object/public/wishes-images/wishes/robot_web_interface.png" alt="Web Interface" width="600"/>


---

## Getting Started

### Prerequisites

- Ubuntu 20.04 + ROS Noetic 1
- Python 3.x
- Hardware: Jupiter IO unit, Astra (Orbbec) camera
- Python packages:
  ```bash
  pip install SpeechRecognition pyaudio flask numpy opencv-python
  
## Installation

### Create and build your workspace
  ```bash
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src
  git clone https://github.com/johnnytan5/voicenav.git
  mv voicenav blindassistant
  cd ..
  catkin_make
  source devel/setup.bash
  ```

---

## Project Structure

### Directory Breakdown
```plaintext
blindassistant/
├── CMakeLists.txt
├── directory_structure.md
├── launch
│   ├── audio_capture_usb.launch
│   └── blind_assistant_startup.launch
├── package.xml
├── scripts
│   ├── audio-in.py
│   ├── camera.py
│   ├── camera_stream.py
│   ├── camera_x_delay.py
│   ├── depth_publishing_images.py
│   ├── email_on_start.py
│   ├── extraction.py
│   ├── static
│   │   ├── kacak.mp3
│   │   ├── main.js
│   │   └── pcm-processor.js
│   ├── templates
│   │   └── index.html
│   ├── text-to-speech-output.py
│   ├── vl_client.py
│   └── web_server.py
├── src
│   └── vosk-model
│       ├── README
│       ├── am
│       ├── conf
│       ├── graph
│       └── ivector
└── srv
    └── VLQuery.srv
```


### ROS Nodes

| Node Name       | Description                                          |
|------------------|------------------------------------------------------|
| `audio_capture`  | Captures mic input and publishes `/audio`            |
| `audio_in.py`    | Speech recognition + command interpreter             |
| `tts_output.py`  | Converts responses to speech                         |
| `camera.py`      | Publishes depth and RGB image streams                |
| `vl_client.py`   | Sends image to VLM API and gets scene description    |
| `web_server.py`  | Flask server for web interface and socket IO         |
| `extraction.py`  | Sends image to VLM API and gets extracted information|
| `email_on_start.py`  | Sends email on start and on demand         |


### ROS Topics


| Topic Name       | Type              | Description                          |
|------------------|-------------------|--------------------------------------|
| `/audio`         | `audio_common_msgs/AudioData` | Raw audio from mic         |
| `/Command`       | `std_msgs/String` | User command (snap, email)     |
| `/Mode`          | `std_msgs/String` | Current mode of the assistant        |
| `/Input`           | `std_msgs/String` | Raw recognition output                    |
| `/robot_news_radio`           | `std_msgs/String` | Text to be spoken                   |

---

## Usage

### Launch
  ```bash
  roslaunch blindassistant blind_assistant_startup.launch
  ```

Once launched:
Wait for:

  "Voice nav activated, web interface activated, please tell me the mode and command, i am delighted to assist you"



### Functions
Command and Modes Available:
- Mode:
  - Walking: Designed for walking assist. Performs obstacle detection warning for close objects and supports snap function to analyze and describe scene
  - Scanning: Designed for static information extraction. Supports snap function which extracts expiry and nutrition information from food packaging
  - Exit: Exit current mode
  - Mode: Read current mode
- Command:
  - Snap: Takes picture in both walking and scanning mode 
  - Email: Send email to email list predefined with public address to web interface


### Testing
To do unit test on each module:
```md
rosrun blindassistant filename.py
```

For instance:

- Run Camera Stream only:
  ```bash
  rosrun blindassistant camera.py
  ```

- Test speech recognizer alone:
  ```bash
  rosrun blindassistant audio_in.
  ```



## Contributors
[![Contributors](https://img.shields.io/github/contributors/johnnytan5/voicenav?style=flat-square)](https://github.com/johnnytan5/voicenav/graphs/contributors)

You can view all contributors [here](https://github.com/johnnytan5/voicenav/graphs/contributors).














