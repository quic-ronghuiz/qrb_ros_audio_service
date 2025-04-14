
# QRB ROS Audio Service
<update with your project name and a short description>
<Table of Contents?>

## Overview
The ROS package provides essential audio capabilities, it is the entry point for ROS to provide audio capabilities.

### Features
It supports both step-by-step and one-touch playback, allowing playback from built-in sounds. Additionally, it offers recording functionality, with the option to save to a local file or a topic.


## Quick Start

> **Note：**
> This document 's build & run is the latest.
> If it conflict with the online document, please follow this.

We provide two ways to use this package.

<details>
<summary>Docker</summary>

#### Setup
1. Please follow this [steps](https://github.com/quic-qrb-ros/qrb_ros_docker?tab=readme-ov-file#quickstart) to setup docker env.
2. Install depency packages.
    ```bash
    (docker) sudo apt install libpulse-dev libpulse-dev
    ```
3. Clone this repository and interface repository.
    ```bash
    (docker) cd ${QRB_ROS_WS}/src
    (docker) git clone https://github.com/quic-qrb-ros/qrb_ros_audio_service.git
    (docker) git clone https://github.com/quic-qrb-ros/qrb_ros_interfaces.git
    ```

#### Build
    ```
	(docker) cd ${QRB_ROS_WS}
    (docker) colcon build
    ```

#### Run
1. Source this file to set up the environment.
    ```bash
    (docker) cd ${QRB_ROS_WS}/
    (docker) source install/local_setup.sh
    ```
2. Use this launch file to run this package.
    ```bash
    (docker) ros2 launch qrb_ros_audio_common component.launch.py
    ```
3. On another terminal.
    ```bash
    (docker) ros2 launch qrb_ros_audio_service audio_service.launch.py
    ```
4. Run test cases on third terminal.
 - Get build-in sound names
    ```bash
    (docker) python3 audio_service_test.py –get-buildin-sound
    ```
 - One-touch playback build-in sound
    ```bash
    (docker) python3 audio_service_test.py –mode=’one-touch’ –source=’security’ –volume=100
    ```
 - One-touch playback and repeat build-in sound
    ```bash
    (docker) python3 audio_service_test.py –mode=’one-touch’ –source=’security’ –volume=100 –repeat=-1
    ```
 - Step-by-step playback
    ```bash
    (docker) python3 audio_service_test.py --type='playback' --source='/tmp/xxx.wav' --volume=100
    ```
 - Step-by-step record
    ```bash
    (docker) python3 audio_service_test.py --type='record' --source='/tmp/rec.wav' --channels=1 --sample_rate=16000 --sample_format=16
    ```

For more case see [qrb_ros_audio_service_msgs](https://github.com/quic-qrb-ros/qrb_ros_interfaces/tree/main/qrb_ros_audio_service_msgs) and [here](https://quic-qrb-ros.github.io/main/index.html).

</details>


<details>
<summary>QIRP-SDK</summary>

#### Setup
Please follow this [steps](https://quic-qrb-ros.github.io/main/getting_started/index.html) to setup qirp-sdk env.

1. Create ros_ws directory in <qirp_decompressed_workspace>/qirp-sdk/
2. Clone this repository and interface repository under <qirp_decompressed_workspace>/qirp-sdk/ros_ws
    ```bash
    git clone https://github.com/quic-qrb-ros/qrb_ros_audio_service.git
    git clone https://github.com/quic-qrb-ros/qrb_ros_interfaces.git
    ```

#### Build
1. Build this project.
    ```bash
    export AMENT_PREFIX_PATH="${OECORE_NATIVE_SYSROOT}/usr:${OECORE_TARGET_SYSROOT}/usr"
    export PYTHONPATH=${PYTHONPATH}:${OECORE_NATIVE_SYSROOT}/usr/lib/python3.12/site-packages/:${OECORE_TARGET_SYSROOT}/usr/lib/python3.12/site-packages/

    colcon build --merge-install --cmake-args \
        -DPython3_NumPy_INCLUDE_DIR=${OECORE_TARGET_SYSROOT}/usr/lib/python3.12/site-packages/numpy/core/include \
        -DPYTHON_SOABI=cpython-312-aarch64-linux-gnu -DCMAKE_STAGING_PREFIX="$(pwd)/install" \
        -DCMAKE_PREFIX_PATH="$(pwd)/install/share" \
        -DBUILD_TESTING=OFF
    ```

2. Install the package.
    ```bash
    cd `<qirp_decompressed_workspace>/qirp-sdk/ros_ws/install`
    tar czvf qrb_ros_audio.tar.gz lib share
    scp qrb_ros_audio.tar.gz root@[ip-addr]:/opt/
    ssh root@[ip-addr]
    (ssh) tar -zxf /opt/qrb_ros_audio.tar.gz -C /opt/qcom/qirp-sdk/usr/
    ```
#### Run
1. Source this file to set up the environment on your device:
    ```bash
    ssh root@[ip-addr]
	(ssh) export HOME=/home
	(ssh) setenforce 0
	(ssh) source /usr/bin/ros_setup.sh && source /usr/share/qirp-setup.sh
    ```
2. Use this launch file to run this package.
    ```bash
    (ssh) ros2 launch qrb_ros_audio_common component.launch.py
    ```
3. On another ssh terminal.
    ```bash
    (ssh) ros2 launch qrb_ros_audio_service audio_service.launch.py
    ```
4. Run test cases on a third ssh terminal.
 - Get build-in sound names
    ```bash
    python3 audio_service_test.py –get-buildin-sound
    ```
 - One-touch playback build-in sound
    ```bash
    python3 audio_service_test.py –mode=’one-touch’ –source=’security’ –volume=100
    ```
 - One-touch playback and repeat build-in sound
    ```bash
    python3 audio_service_test.py –mode=’one-touch’ –source=’security’ –volume=100 –repeat=-1
    ```
 - Step-by-step playback
    ```bash
    python3 audio_service_test.py --type='playback' --source='/tmp/xxx.wav' --volume=100
    ```
 - Step-by-step record
    ```bash
    python3 audio_service_test.py --type='record' --source='/tmp/rec.wav' --channels=1 --sample_rate=16000 --sample_format=16
    ```

For more case see [qrb_ros_audio_service_msgs](https://github.com/quic-qrb-ros/qrb_ros_interfaces/tree/main/qrb_ros_audio_service_msgs) and [here](https://quic-qrb-ros.github.io/main/index.html).

</details>

<br>

You can get more details from [here](https://quic-qrb-ros.github.io/main/index.html).

## Contributing

We would love to have you as a part of the QRB ROS community. Whether you are helping us fix bugs, proposing new features, improving our documentation, or spreading the word, please refer to our [contribution guidelines](./CONTRIBUTING.md) and [code of conduct](./CODE_OF_CONDUCT.md).

- Bug report: If you see an error message or encounter failures, please create a [bug report](../../issues)
- Feature Request: If you have an idea or if there is a capability that is missing and would make development easier and more robust, please submit a [feature request](../../issues)

<Update link with template>


## Authors

* **Yuchao Pan** - *Initial work* - [YuchPan](https://github.com/yuchpan)

See also the list of [contributors](https://github.com/QUIC-QRB-ROS/qrb_ros_audio_service/contributors) who participated in this project.


## License

Project is licensed under the [BSD-3-clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](./LICENSE) for the full license text.


