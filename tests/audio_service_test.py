import sys
import time
import argparse

import rclpy
from rclpy.node import Node

from qrb_ros_audio_service_msgs.srv import AudioRequest
from qrb_ros_audio_service_msgs.msg import AudioInfo


class AudioServiceClient(Node):
    def __init__(self):
        super().__init__("audio_service_client")
        self.cli = self.create_client(AudioRequest, "audio_server")
        self.stream_handle = 0
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

    def send_request(self, req):
        self.req = req
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is not None:
            self.get_logger().info(
                "command %s success %d stream_handle %d buildin_sound_name %s"
                % (
                    self.req.command,
                    response.success,
                    response.stream_handle,
                    response.buildin_sound_name,
                )
            )
            if response.success:
                if (response.stream_handle != 0) and (
                    (self.req.command == "create") or (self.req.command == "play")
                ):
                    self.set_handle(response.stream_handle)
                if (self.mode == "step-by-step") and (self.req.command == "create"):
                    req = AudioRequest.Request()
                    req.stream_handle = self.stream_handle
                    req.command = "start"
                    time.sleep(0.5)
                    self.send_request(req)

    def on_shutdown(self):
        if self.stream_handle:
            req = AudioRequest.Request()
            req.stream_handle = self.stream_handle

            print("stop stream")
            req.command = "stop"
            self.send_request(req)

        if self.stream_handle and (self.mode == 'step-by-step'):
            time.sleep(0.5)

            print("release stream")
            req.command = "release"
            audio_service_client.send_request(req)

    def set_handle(self, handle):
        self.stream_handle = handle

    def set_mode(self, mode):
        self.mode = mode


def str2bool(v):
    if v.lower() in ("yes", "true", "t", "y", "1"):
        return True
    elif v.lower() in ("no", "false", "f", "n", "0"):
        return False
    else:
        raise argparse.ArgumentTypeError("invalid value.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--get-buildin-sound", action="store_true")
    parser.add_argument("--set-mute", action="store_true")
    parser.add_argument("--type", type=str, default="")
    parser.add_argument("--mode", type=str, default="step-by-step")
    parser.add_argument("--source", type=str, default="security")
    parser.add_argument("--volume", type=int, default=100)
    parser.add_argument("--repeat", type=int, default=0)
    parser.add_argument("--channels", type=int, default=1)
    parser.add_argument("--sample_rate", type=int, default=16000)
    parser.add_argument("--sample_format", type=int, default=16)
    parser.add_argument("--pub_pcm", type=str2bool, default=False)
    parser.add_argument("--stream_handle", type=int, default=0)
    parser.add_argument("--mute", type=str2bool, default=False)
    args = parser.parse_args()

    global audio_service_client

    rclpy.init()
    audio_service_client = AudioServiceClient()

    exit = False

    req = AudioRequest.Request()
    if args.get_buildin_sound:
        req.command = "get-buildin-sound"
        audio_service_client.get_logger().info("request get-buildin-sound")
        exit = True
    elif (args.mode == "one-touch") or (
        (args.type == "playback") and (args.mode == "step-by-step")
    ):
        if args.mode == "one-touch":
            req.command = "play"
            req.type = "playback"
        else:
            req.command = "create"
        req.type = args.type
        req.source = args.source
        req.volume = args.volume
        req.repeat = args.repeat
        audio_service_client.get_logger().info(
            "request playback. mode %s source %s volume %d repeat %d"
            % (args.mode, req.source, req.volume, req.repeat)
        )
    elif args.type == "record":
        req.audio_info = AudioInfo(
            channels=args.channels,
            sample_rate=args.sample_rate,
            sample_format=args.sample_format,
        )
        req.type = args.type
        req.command = "create"
        req.source = args.source
        req.pub_pcm = args.pub_pcm
        audio_service_client.get_logger().info(
            "request record. source %s channels %d sample_rate %d sample_format %d pub_pcm %d"
            % (
                req.source,
                args.channels,
                args.sample_rate,
                args.sample_format,
                req.pub_pcm,
            )
        )
    elif args.set_mute and (args.stream_handle != 0):
        req.command = "mute"
        req.mute = args.mute
        req.stream_handle = args.stream_handle
        audio_service_client.get_logger().info(
            "request set-mute. stream_handle %d mute %d"
            % (args.stream_handle, args.mute)
        )
        exit = True
    else:
        audio_service_client.get_logger().error("unsupported command")
        sys.exit()

    audio_service_client.set_mode(args.mode)
    audio_service_client.send_request(req)

    if exit:
        audio_service_client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit()

    try:
        rclpy.spin(audio_service_client)
    except KeyboardInterrupt:
        print("Ctrl+C received. Shutting down.")
    finally:
        audio_service_client.on_shutdown()
        audio_service_client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
