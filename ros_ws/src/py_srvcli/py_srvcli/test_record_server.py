from tutorial_interfaces.srv import StringInOut
from record_modules.record_server import record_server
import rclpy
from rclpy.node import Node


class record_server_async(Node):

    def __init__(self):
        super().__init__('record_server_async')
        self.srv = self.create_service(StringInOut, 'faster_whisper_stt', self.StringInOut_callback)
        # init the model to run
        self.server = None
        self._initialize_server_stt()

    def _initialize_server_stt(self):
        self.server = record_server()

    def StringInOut_callback(self, request, response):
        response.text = self.server._transcribe(request.filename)
        self.get_logger().info('Incoming request\na: %s' % (request.filename))
        return response


def main():
    rclpy.init()

    server = record_server_async()

    rclpy.spin(server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()