import sys
import socket
from faster_whisper import WhisperModel
import logging


class record_server:

    def __init__(
        self,
        model_size = 'tiny.en',
        device = 'cpu',
        compute_type = 'int8',
        beam_size = 5,
        host = '127.0.0.1',
        port = 8090
    ):
        self.model_size = model_size
        self.device = device
        self.compute_type = compute_type
        self.beam_size = beam_size
        self.host = host
        self.port = port
        self.model = None
        self.socket = None
        # open server
        self._open_server()
        #init model
        self._init_model()

    def run(self):
        self.socket.listen(1)
        while True:
            try:
                conn, addr = self.socket.accept()
                # first wait for response from client
                data = conn.recv(1024)
                if not data:
                    print("No Data received")
                    continue
                print("Recieved from client",data.decode())
                text = self._transcribe(data.decode())
                # send the transcribed data
                conn.send(text.encode())
            except Exception as e:
                print("run error :")
                print(e)
                # close the server socket
                self._close_server()
                # exit the client
                sys.exit(1)

    def _transcribe(self, file_name):
        try:
            segments, info = self.model.transcribe(file_name, beam_size=self.beam_size)
            # print("Detected language '%s' with probability %f" % (info.language, info.language_probability))
            text_list = []
            for segment in segments:
                # print("[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))
                text_list.append(segment.text)
            if not text_list:
                return "[NO SPEECH]"
            return " ".join(text_list)
        except Exception as e:
            print("_transcribe error :")
            print(e)
            # close the server socket
            self._close_server()
            # exit the client
            sys.exit(1)
        

    def _init_model(self):
        try:
            self.model = WhisperModel(self.model_size, device=self.device, compute_type=self.compute_type)
            # logging.basicConfig()
            # logging.getLogger("faster_whisper").setLevel(logging.DEBUG)
        except Exception as e:
            print("_init_model error :")
            print(e)
            # close the server socket
            self._close_server()
            # exit the client
            sys.exit(1)

    def _close_server(self):
        self.socket.close()

    def _open_server(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.bind((self.host, self.port))
        except socket.error as e:
            print("_open_server error :")
            print(e)
            # exit the client
            sys.exit(1)
