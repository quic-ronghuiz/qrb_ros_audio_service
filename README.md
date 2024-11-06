# QRB ROS Audio Service

## Overview

The ROS package provides essential audio capabilities, it is the entry point for ROS to provide audio capabilities.

It supports both step-by-step and one-touch playback, allowing playback from built-in sounds. Additionally, it offers recording functionality, with the option to save to a local file or a topic.

Playback and record capabilities depend on [qrb_ros_audio_common](https://github.qualcomm.com/QUIC-QRB-ROS/qrb_ros_audio_common).

## Build

Use QIRP SDK to build.

1. Build and install QIRP SDK follow [qirp-sdk-workflows](https://docs.qualcomm.com/bundle/publicresource/topics/80-65220-2/introduction_1.html?vproduct=1601111740013072&versionId=1befb000-28cc-4b51-8b35-81601178edee&facet=Qualcomm%20Intelligent%20Robotics%20(QIRP)%20Product%20SDK#qirp-sdk-workflows) or [Quick start (using the prebuild package)](https://docs.qualcomm.com/bundle/publicresource/topics/80-65220-2/quick-start_3.html?vproduct=1601111740013072&versionId=1befb000-28cc-4b51-8b35-81601178edee&facet=Qualcomm%20Intelligent%20Robotics%20(QIRP)%20Product%20SDK)

2. Setup environments follow [Set up the cross-compile environment](https://docs.qualcomm.com/bundle/publicresource/topics/80-65220-2/develop-your-first-application_6.html?product=1601111740013072&facet=Qualcomm%20Intelligent%20Robotics%20(QIRP)%20Product%20SDK&state=releasecandidate)

3. Create `ros_ws` directory in `<qirp_decompressed_workspace>/qirp-sdk/`

4. Put qrb_audio_manager, qrb_ros_audio_service, qrb_ros_audio_service_msgs and qrb_ros_audio_common_msgs packages under `<qirp_decompressed_workspace>/qirp-sdk/ros_ws`

     qrb_audio_manager and qrb_ros_audio_service in qrb_ros_audio_service:
     ```bash
     git clone https://github.qualcomm.com/QUIC-QRB-ROS/qrb_ros_audio_service.git
     ```
     qrb_ros_audio_service_msgs and qrb_ros_audio_common_msgs in qrb_ros_interfaces:
     ```bash
     git clone https://github.qualcomm.com/QUIC-QRB-ROS/qrb_ros_interfaces.git
     ```
6. Build this project
     ```bash
     export AMENT_PREFIX_PATH="${OECORE_TARGET_SYSROOT}/usr;${OECORE_NATIVE_SYSROOT}/usr"
     export PYTHONPATH=${PYTHONPATH}:${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages

     colcon build --merge-install --cmake-args \
       -Dpython3_ROOT_DIR=${OECORE_TARGET_SYSROOT}/usr \
       -Dpython3_NumPy_INCLUDE_DIR=${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages/numpy/core/include \
       -DPYTHON_SOABI=cpython-310-aarch64-linux-gnu -DCMAKE_STAGING_PREFIX=$(pwd)/install \
       -DCMAKE_PREFIX_PATH=$(pwd)/install/share \
       -DBUILD_TESTING=OFF
     ```
7. Push to the device & Install
     ```bash
     cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws/install
     tar czvf qrb_ros_audio.tar.gz lib share
     scp qrb_ros_audio.tar.gz root@[ip-addr]:/opt/
     ssh root@[ip-addr]
     tar -zxf /opt/qrb_ros_audio.tar.gz -C /opt/qcom/qirp-sdk/usr/
     ```

## Run

1. Source this file to set up the environment on your device
    ```bash
    ssh root@[ip-addr]
    export HOME=/opt
    source /opt/qcom/qirp-sdk/qirp-setup.sh
    export ROS_DOMAIN_ID=xx
    source /usr/bin/ros_setup.bash
    ```

2. Use this launch file to run this package
    ```bash
    ros2 launch qrb_ros_audio_service audio_service.launch.py
    ```

3. Run qrb_ros_audio_common on another ssh terminal

4. Run below test cases on third ssh terminal
   
     stream_handle indicate the stream created by Audio Service when command is "create". The value can be found on third ssh terminal:
     if use "ROS Command":
     `stream_handle=***`
     if use "Python Script":
     `command create success 1 stream_handle ***`
   
    | Test Case     | Using ROS Command   | Using Python Script      | 
    | :--------- |:----------| ----------|
    | Get build-in sound names | mkdir -p /opt/qcom/qirp-sdk/usr/share/qrb-audio-manager/sounds<br>chmod 0755 /opt/qcom/qirp-sdk/usr/share/qrb-audio-manager -R<br>adb push clip.wav /opt/qcom/qirp-sdk/usr/share/qrb-audio-manager/sounds<br>chmod 0644 /opt/qcom/qirp-sdk/usr/share/qrb-audio-manager/sounds/*<br><br>ros2 service call /audio_server qrb_ros_audio_service_msgs/srv/AudioRequest "{<br>command: "get-buildin-sound"<br>}"| python3 audio_service_test.py --get-buildin-sound |
    | One-touch playback | ros2 service call /audio_server qrb_ros_audio_service_msgs/srv/AudioRequest "{<br>command: "play",<br>source: "clip",<br>volume: 100,<br>}" | python3 audio_service_test.py --mode='one-touch' --source='security' --volume=100 |
    |One-touch playback and repeat|ros2 service call /audio_server qrb_ros_audio_service_msgs/srv/AudioRequest "{<br>command: "play",<br>source: "clip",<br>volume: 100,<br>repeat: -1,<br>}"<br><br>ros2 service call /audio_server qrb_ros_audio_service_msgs/srv/AudioRequest "{<br>command: "stop",<br>stream_handle: 3520332881,<br>}"|python3 audio_service_test.py --mode='one-touch' --source='security' --volume=100 --repeat=-1|
    |Step-by-step playback|ros2 service call /audio_server qrb_ros_audio_service_msgs/srv/AudioRequest "{<br>type: "playback",<br>command: "create",<br>source: "/tmp/music.wav",<br>volume: 100,<br>}"<br><br>ros2 service call /audio_server qrb_ros_audio_service_msgs/srv/AudioRequest "{<br>command: "start",<br>stream_handle: 2551300426,<br>}"<br><br>ros2 service call /audio_server qrb_ros_audio_service_msgs/srv/AudioRequest "{<br>command: "mute",<br>mute: true,<br>stream_handle: 2551300426,<br>}"<br><br>ros2 service call /audio_server qrb_ros_audio_service_msgs/srv/AudioRequest "{<br>command: "mute",<br>mute: false,<br>stream_handle: 2551300426,<br>}"<br><br>ros2 service call /audio_server qrb_ros_audio_service_msgs/srv/AudioRequest "{<br>command: "stop",<br>stream_handle: 2551300426,<br>}"<br><br>ros2 service call /audio_server qrb_ros_audio_service_msgs/srv/AudioRequest "{<br>command: "release",<br>stream_handle: 2551300426,<br>}"|python3 audio_service_test.py --type='playback' --source='/tmp/yesterday_48KHz.wav' --volume=100<br><br>python3 audio_service_test.py --set-mute --mute=True --stream_handle=2124364840<br><br>python3 audio_service_test.py --set-mute --mute=False --stream_handle=2124364840|
    |Step-by-step record|ros2 service call /audio_server qrb_ros_audio_service_msgs/srv/AudioRequest "{<br>audio_info: {<br>channels: 1,<br>sample_rate: 16000,<br>sample_format: 16,<br>},<br>type: "record",<br>command: "create",<br>source: "/tmp/rec.wav",<br>}"<br><br>ros2 service call /audio_server qrb_ros_audio_service_msgs/srv/AudioRequest "{<br>command: "start",<br>stream_handle: 1502099078,<br>}"<br><br>ros2 service call /audio_server qrb_ros_audio_service_msgs/srv/AudioRequest "{<br>command: "stop",<br>stream_handle: 1502099078,<br>}"<br><br>ros2 service call /audio_server qrb_ros_audio_service_msgs/srv/AudioRequest "{<br>command: "release",<br>stream_handle: 1502099078,<br>}"|python3 audio_service_test.py --type='record' --source='/tmp/rec.wav' --channels=1 --sample_rate=16000 --sample_format=16|
    |Publish recording data|publish recording data to /qrb_audiodata:<br>ros2 service call /audio_server qrb_ros_audio_service_msgs/srv/AudioRequest "{<br>audio_info: {<br>channels: 1,<br>sample_rate: 16000,<br>sample_format: 16,<br>},<br>type: "record",<br>command: "create",<br>pub_pcm: true,<br>}"<br><br>publish recording data to /qrb_audiodata. meanwhile, save it to file:<br>ros2 service call /audio_server qrb_ros_audio_service_msgs/srv/AudioRequest "{<br>audio_info: {<br>channels: 1,<br>sample_rate: 16000,<br>sample_format: 16,<br>},<br>type: "record",<br>command: "create",<br>pub_pcm: true,<br>source: "/tmp/rec.wav",<br>}"<br><br>ros2 service call /audio_server qrb_ros_audio_service_msgs/srv/AudioRequest "{command: "start",<br>stream_handle: 806639317,<br>}"<br>ros2 topic echo /qrb_audiodata<br><br>ros2 service call /audio_server qrb_ros_audio_service_msgs/srv/AudioRequest "{<br>command: "stop",<br>stream_handle: 806639317,<br>}"<br><br><br>ros2 service call /audio_server qrb_ros_audio_service_msgs/srv/AudioRequest "{<br>command: "release",<br>stream_handle: 806639317,<br>}"|python3 audio_service_test.py --type='record' --source='/tmp/rec.wav' --channels=1 --sample_rate=16000 --sample_format=16 --pub_pcm=True<br><br>ros2 topic echo /qrb_audiodata|

## License

qrb_ros_audio_service is licensed under the BSD-3-clause "New" or "Revised" License. 

Check out the [LICENSE](LICENSE) for more details.
