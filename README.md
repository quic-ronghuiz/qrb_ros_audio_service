<div align="center">
  <h1>QRB ROS Audio Service</h1>
  <p align="center">
  </p>
  <p>This ROS package delivers essential audio capabilities for playback and recording</p>

  <a href="https://ubuntu.com/download/qualcomm-iot" target="_blank"><img src="https://img.shields.io/badge/Qualcomm%20Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white" alt="Qualcomm Ubuntu"></a>
  <a href="https://docs.ros.org/en/jazzy/" target="_blank"><img src="https://img.shields.io/badge/ROS%20Jazzy-1c428a?style=for-the-badge&logo=ros&logoColor=white" alt="Jazzy"></a>
 
</div>

---

## 👋 Overview

> 📌 **qrb_ros_audio_service is a ROS package that provides core audio functionalities. It serves as the primary interface for audio playback and recording within the ROS ecosystem.**

<div align="left">
  <img src="./docs/assets/architecture.png" alt="architecture" width="600">
</div>

- QRB ROS Audio Service : A ROS node that creates a service server, responds to requests from the application, and translates them into operations on the Audio Manager.
- QRB Audio Manager : Executes audio operations provided by the Audio Service, based on stream configurations.
- QRB Audio Common Lib : Provides audio functionalities by calling PulseAudio or ALSA APIs, depending on which audio backend is available on the system.

## 🔎 Table of Contents

> 📌 If the content is extensive, we recommend adding a table of contents.

  * [APIs](#-apis)
  * [Supported Targets](#-supported-targets)
  * [Installation](#-installation)
  * [Usage](#-usage)
  * [Build from Source](#-build-from-source)
  * [Contributing](#-contributing)
  * [License](#-license)

## ⚓ APIs

> 📌 `qrb_ros_audio_service` APIs

**ROS Interfaces**
<table>
  <tr>
    <th>Interface</th>
    <th>Name</th>
    <th>Type</th>
    <td>Description</td>
  </tr>
  <tr>
    <td>Service</td>
    <td>/audio_service</td>
    <td>qrb_ros_audio_service_msgs::srv::AudioRequest</td>
    <td>Allows ROS applications to send service requests for audio playback and recording.</td>
  </tr>
  <tr>
    <td>Subscriber</td>
    <td>/qrb_audiodata</td>
    <td>qrb_ros_audio_service_msgs::msg::AudioData</td>
    <td>Used for streaming audio playback. Subscribes to raw audio data from this node. Can modify the topic name using the --topic_name parameter.</td>
  </tr>
  <tr>
    <td>Publisher</td>
    <td>/qrb_audiodata</td>
    <td>qrb_ros_audio_service_msgs::msg::AudioData</td>
    <td>Used for streaming audio recording. Publishes real-time raw audio data during recording. Can modify the topic name using the --topic_name parameter.</td>
  </tr>
</table>

#### ROS parameters
  <table>
    <tr>
      <th>Name</th>
      <th>Type</th>
      <th>Description</th>
      <td>Default Value</td>
    </tr>
    <tr>
      <td>enable_latency_log</td>
      <td>bool</td>
      <td>enable latency log print</td>
      <td>false</td>
    </tr>
  </table>

> 📌  `qrb_audio_common_lib` APIs

<table>
  <tr>
    <th>Function</th>
    <th>Parameters</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>AudioBackend detect_audio_backend()</td>
    <td>None</td>
    <td>Detect which audio backend is available on the system (PulseAudio is preferred, ALSA is used as a fallback). Returns the detected AudioBackend (INVALID/ALSA/PULSEAUDIO).</td>
  </tr>
  <tr>
    <td>uint32_t audio_stream_open(const AudioStreamInfo & stream_info, stream_event_callback_func event_callback)</td>
    <td>
      <strong>stream_info</strong>: A structure that specifies the configuration of the stream (format, rate, channels, type, file_path, volume, device, need_timestamp, pcm_mode, repeat).<br>
      <strong>event_callback</strong>: Callback function invoked for stream events (start/stop/abort/timestamp/data/eos).
    </td>
    <td>Open a stream, returns stream_handle on success, 0 on failure.</td>
  </tr>
  <tr>
    <td>int audio_stream_start(uint32_t stream_handle)</td>
    <td><strong>stream_handle</strong>: A unique identifier for the stream.</td>
    <td>Start stream, return 0 on succeed.</td>
  </tr>
  <tr>
    <td>int audio_stream_mute(uint32_t stream_handle, bool mute)</td>
    <td>
      <strong>stream_handle</strong>: A unique identifier for stream.<br>
      <strong>mute</strong>: Boolean flag indicating whether to mute the stream.
    </td>
    <td>Mute or unmute stream. return 0 on succeed.</td>
  </tr>
  <tr>
    <td>int audio_stream_stop(uint32_t stream_handle)</td>
    <td><strong>stream_handle</strong>: A unique identifier for stream.</td>
    <td>Stop stream, return 0 on succeed</td>
  </tr>
  <tr>
    <td>int audio_stream_close(uint32_t stream_handle)</td>
    <td><strong>stream_handle</strong>: A unique identifier for stream.</td>
    <td>Close stream and release its resources, return 0 on succeed.</td>
  </tr>
  <tr>
    <td>int audio_stream_write(uint32_t stream_handle, const void * buf, size_t length)</td>
    <td>
      <strong>stream_handle</strong>: A unique identifier for stream.<br>
      <strong>buf</strong>: Pointer to the data buffer.<br>
      <strong>length</strong>: Size to write.
    </td>
    <td>Write buffer to a PCM-mode playback stream, return actually written length, negative value on failure.</td>
  </tr>
</table>

## 🎯 Supported Targets

<table >
  <tr>
    <th>Development Hardware</th>
    <td>Qualcomm Dragonwing™ IQ-9075 EVK</td>
  </tr>
  <tr>
    <th>Hardware Overview</th>
    <th><a href="https://www.qualcomm.com/products/internet-of-things/industrial-processors/iq9-series/iq-9075"><img src="https://s7d1.scene7.com/is/image/dmqualcommprod/dragonwing-IQ-9075-EVK?$QC_Responsive$&fmt=png-alpha" width="160"></a></th>
  </tr>
</table>

---

## ✨ Installation

> [!IMPORTANT]
> **PREREQUISITES**: The following steps need to be run on **Qualcomm Ubuntu** and **ROS Jazzy**.<br>
> Reference [Install Ubuntu on Qualcomm IoT Platforms](https://ubuntu.com/download/qualcomm-iot) and [Install ROS Jazzy](https://docs.ros.org/en/jazzy/index.html) to setup environment. <br>
> For Qualcomm Linux, please check out the [Qualcomm Intelligent Robotics Product SDK](https://docs.qualcomm.com/bundle/publicresource/topics/80-70018-265/introduction_1.html?vproduct=1601111740013072&version=1.4&facet=Qualcomm%20Intelligent%20Robotics%20Product%20(QIRP)%20SDK) documents.

### Add Qualcomm IOT PPA for Ubuntu:

```bash
# Install Qualcomm PPA
sudo add-apt-repository ppa:ubuntu-qcom-iot/qcom-ppa
sudo add-apt-repository ppa:ubuntu-qcom-iot/qirp
sudo apt update
```

### Install Packages

```bash
# Install QRB ROS Audio packages
sudo apt ros-jazzy-qrb-ros-audio-service
```

## 🚀 Usage

1. Use this launch file to run this package.
    ```bash
    source /opt/ros/jazzy/setup.sh
    ros2 launch qrb_ros_audio_service audio_service.launch.py
    ```
2. Run test cases on a third ssh terminal.
 - push music file to device path like /tmp/xxx.wav.
 - download test script.
    ```bash
    wget https://raw.githubusercontent.com/qualcomm-qrb-ros/qrb_ros_audio_service/main/tests/audio_service_test.py
    ```
 - setup ros2 env.
     ```bash
    source /opt/ros/jazzy/setup.sh
    ```
 - Step-by-step playback (sound will output on speaker)
    ```bash
    python3 audio_service_test.py --type='playback' --source='/tmp/xxx.wav' --volume=100
    ```
 - Step-by-step record (will record sound input from mic and save to file)
    ```bash
    python3 audio_service_test.py --type='record' --source='/tmp/rec.wav' --channels=1 --sample_rate=16000 --sample_format=16
    ```
 - Streaming playback
    ```bash
    python3 audio_service_test.py --type='playback' --channels=1 --sample_rate=16000 --sample_format=16 --pub_pcm=True --volume=100 --topic_name='loopback'
    ```
 - Streaming record (if start Streaming playback and Streaming record sound will loopback from mic to speaker)
    ```bash
    python3 audio_service_test.py --type='record' --source='/tmp/rec.wav' --channels=1 --sample_rate=16000 --sample_format=16 --topic_name='loopback'
    ```
---

## 👨‍💻 Build from Source

### Step 1: Install dependencies: 
```bash
# Install build tools and dependencies
sudo add-apt-repository ppa:ubuntu-qcom-iot/qcom-ppa
sudo add-apt-repository ppa:ubuntu-qcom-iot/qirp
sudo apt update
sudo apt install build-essential cmake pkg-config

# Install ROS2 Jazzy (if not already installed)
# Follow instructions at https://docs.ros.org/en/jazzy/Installation.html
sudo apt install ros-jazzy-rclcpp ros-jazzy-rclcpp-components ros-jazzy-ament-cmake-auto ros-jazzy-std-msgs libpulse-dev libsndfile1-dev libasound2-dev
```

### Step 2: Clone and Build

```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Clone the repository (if not already cloned)
git clone https://github.com/qualcomm-qrb-ros/qrb_ros_audio_service.git

# Build
cd ~/ros2_ws
colcon build

# Source the workspace
source install/setup.bash
```

## 🤝 Contributing

We love community contributions! Get started by reading our [CONTRIBUTING.md](CONTRIBUTING.md).  
Feel free to create an issue for bug reports, feature requests, or any discussion 💡.

## 📜 License

Project is licensed under the [BSD-3-clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](./LICENSE) for the full license text.
