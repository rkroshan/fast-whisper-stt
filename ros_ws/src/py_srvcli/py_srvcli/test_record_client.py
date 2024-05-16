import sys
from record_modules.record_client import record_client
from tutorial_interfaces.srv import StringInOut
import rclpy
from rclpy.node import Node


class record_client_async(Node):

    def __init__(self):
        super().__init__('record_client_async')
        self.cli = self.create_client(StringInOut, 'faster_whisper_stt')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = StringInOut.Request()

        self.recorder = None
        self._initialize_recorder()

    def _initialize_recorder(self):
        self.recorder = record_client(
            int_wav_file_dir='/home/rkroshan/fast-whisper-stt/ros_ws/wavfiles',
            record_seconds=3
        )

    def start_recording(self):
        # start the recording and convert it into wavefile
        self.recorder._start_recording()
        self.req.filename = self.recorder.get_wavefilename()
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        # del the recording since we got the text
        self.recorder.del_wavefile()
        return self.future.result()

def main():
    rclpy.init()

    client = record_client_async()
    while True:
        response = client.start_recording()
        client.get_logger().info(
            'Resp: %s' % (response.text))

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()