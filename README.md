# QRB ROS Audio Service

qrb_ros_audio_service is a ROS package that provides core audio functionalities for playback and recording within the ROS ecosystem. It provides:

* A ROS service server that responds to requests from applications and translates them into operations on the Audio Manager.
* Audio playback and recording via ROS actions and topics, supporting both file-based and streaming (PCM) modes.
* Loopback support: streaming playback and recording can be connected via a shared topic name.

> Prerequisite: Qualcomm Ubuntu and ROS Jazzy are required.

## 🔎 Table of contents

  * [APIs](#-apis)
  * [Supported targets](#-supported-targets)
  * [Usage](#-usage)
  * [Build from source](#-build-from-source)
  * [License](#-license)

## ⚓ APIs

### `qrb_ros_audio_service`

| Interface | Name | Type | Description |
| --------- | ---- | ---- | ----------- |
| Service | `/audio_service` | `qrb_ros_audio_service_msgs::srv::AudioRequest` | Allows ROS applications to send service requests for audio playback and recording. |
| Subscriber | `specified by the topic parameter` | `qrb_ros_audio_service_msgs::msg::AudioData` | Used for streaming audio playback. |
| Publisher | `specified by the topic parameter` | `qrb_ros_audio_service_msgs::msg::AudioData` | Used for streaming audio recording. |

### `qrb_audio_common_lib`

| Function | Parameters | Description |
| -------- | ---------- | ----------- |
| `uint32_t audio_stream_open(const audio_stream_info & stream_info, stream_event_callback_func event_callback)` | `stream_info`: stream configuration; `event_callback`: callback function. | Open stream, returns stream handle on success. |
| `int audio_stream_start(uint32_t stream_handle)` | `stream_handle`: unique stream identifier. | Start stream, returns 0 on success. |
| `int audio_stream_mute(uint32_t stream_handle, bool mute)` | `stream_handle`: unique stream identifier; `mute`: mute flag. | Mute or unmute stream, returns 0 on success. |
| `int audio_stream_stop(uint32_t stream_handle)` | `stream_handle`: unique stream identifier. | Stop stream, returns 0 on success. |
| `size_t audio_stream_write(uint32_t stream_handle, size_t length, void * buf)` | `stream_handle`: unique stream identifier; `length`: size to write; `buf`: data buffer pointer. | Write buffer to playback stream, returns actually written length. |
| `AudioBackend detect_audio_backend();` | `void` | detect supported audio backend, returns backend type on succeed. |

## 🎯 Supported targets

- IQ10
- Qualcomm Dragonwing™ RB3 Gen2
- Qualcomm Dragonwing™ IQ-9075 EVK
---

## 🚀 Usage

1. Launch the audio service:

    ```bash
    source /opt/ros/jazzy/setup.sh
    ros2 launch qrb_ros_audio_service audio_service.launch.py
    ```

2. Push test script and push a WAV file to the device (e.g. `/tmp/xxx.wav`):

    ```
    adb push sources/quic-qrb-ros/qrb_ros_audio_service/tests/audio_service_test.py /tmp/
    ```
    ```bash
    cd /tmp/
    source /opt/ros/jazzy/setup.sh
    ```

3. Run test cases:

    - Step-by-step playback (output on speaker):

        ```bash
        python3 audio_service_test.py --type='playback' --source='/tmp/xxx.wav' --volume=100
        ```

    - Step-by-step record (save mic input to file):

        ```bash
        python3 audio_service_test.py --type='record' --source='/tmp/rec.wav' --channels=1 --sample_rate=16000 --sample_format=16
        ```

    - Streaming playback:

        ```bash
        python3 audio_service_test.py --type='playback' --channels=1 --sample_rate=16000 --sample_format=16 --pub_pcm=True --volume=100 --topic_name='loopback'
        ```

    - Streaming record (loopback when combined with streaming playback):

        ```bash
        python3 audio_service_test.py --type='record' --source='/tmp/rec.wav' --channels=1 --sample_rate=16000 --sample_format=16 --topic_name='loopback'
        ```

---

## 👨‍💻 Build from source

Source is located at `sources/quic-qrb-ros/qrb_ros_audio_service/` in the workspace.

```bash
cd build-utils/ubuntu/
python3 build.py --gen-debians --package ros-jazzy-qrb-ros-audio-service
```

Built `.deb` files are output to:

```
<workspace>/debian_packages/oss/ros-jazzy-qrb-ros-audio-service/
<workspace>/debian_packages/oss/qrb-audio-manager/
<workspace>/debian_packages/oss/qrb-audio-common-lib
```

## 📜 License

Project is licensed under the [BSD-3-Clause](https://spdx.org/licenses/BSD-3-Clause.html) License. See [LICENSE](./LICENSE) for the full license text.