import pyaudio
import socket
import sys
import wave
import os
import shutil

class record_client:
    def __init__(
        self,
        format=pyaudio.paInt16,
        channels=1,
        rate=44100,
        chunk=512,
        record_seconds=5,
        device_index=-1,
        host='127.0.0.1',
        port=8090,
        int_wav_file_dir = '/home/rkrosha/faster-whisper/wavfiles/'
    ):
        self.format = format
        self.channels = channels
        self.rate = rate
        self.chunk = chunk
        self.record_seconds = record_seconds
        self.device_index = device_index
        self.audio = pyaudio.PyAudio()
        self.stream = None
        self.recordframes = []
        self.host = host
        self.port = port
        self.socket = None
        self.int_wav_file_dir = int_wav_file_dir
        self.wavefile = None
        if not os.path.exists(self.int_wav_file_dir):
            os.makedirs(self.int_wav_file_dir)

        if self.device_index == -1:
            self._select_record_device()
        
    def __call__(self):
        # called when record client object run
        # open stream and start recording
        self._start_recording()
    
    def _start_recording(self):
        while True:
            try:
                x = input("press any key to start recording...")
                # now try to connect to server
                self._connect_to_server()
                print("recording started")
                self._open_record_stream()
                self._record()
                print ("recording stopped")
                self._convert_to_wavefile_n_st_server()
                self._stop_stream()
                self._close_stream()
                self._get_STT_server_response()
                self._close_socket_connection()
            except KeyboardInterrupt:
                print("Interrupted by user")
                self._close_everything()

    def _get_STT_server_response(self):
        # print("Waiting for response...")
        data = self.socket.recv(1024)
        # print("Response:>>>")
        print(data.decode())
        # del the wav file
        os.remove(self.wavefilename)

    def _del_wavefiles(self):
        for root, dirs, files in os.walk(self.int_wav_file_dir):
            for f in files:
                os.unlink(os.path.join(root, f))
            for d in dirs:
                shutil.rmtree(os.path.join(root, d))

    def _convert_to_wavefile_n_st_server(self):
        try:
            # create wavefile 
            self.wavefilename = self.int_wav_file_dir + "wav"
            waveFile = wave.open(self.wavefilename, 'wb')
            waveFile.setnchannels(self.channels)
            waveFile.setsampwidth(self.audio.get_sample_size(self.format))
            waveFile.setframerate(self.rate)
            waveFile.writeframes(b''.join(self.recordframes))
            waveFile.close()
            self.recordframes = []
            # send to server
            self._send_packets_to_server(self.wavefilename.encode())
        except Exception as e:
            print("socket error :")
            print(e)
            self._close_everything()

    def _send_packets_to_server(self, packet):
        try:
            print("Sent packets:", packet)
            self.socket.sendall(packet)
        except socket.error as e:
            print("socket error :")
            print(e)
            self._close_everything()

    def _close_everything(self):
        self._stop_stream()
        self._close_stream()
        self._terminate_audio_connection()
        self._close_socket_connection()
        # self._del_wavefiles()
        sys.exit(1)

    def _record(self):
        try:
            for i in range(0, int(self.rate / self.chunk * self.record_seconds)):
                data = self.stream.read(self.chunk)
                self.recordframes.append(data)
        except Exception as e:
            print("_record error :")
            print(e)
            self._close_everything()

    def _open_record_stream(self):
        try:
            self.stream = self.audio.open(format=self.format, channels=self.channels,
                            rate=self.rate, input=True,input_device_index = self.device_index,
                            frames_per_buffer=self.chunk)
        except Exception as e:
            print("_open_record_stream error:")
            print(e)
            self._close_socket_connection()
            self._terminate_audio_connection()
            sys.exit(1)

    def _connect_to_server(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
        except socket.error as e:
            print("socket error :")
            print(e)
            # need to terminate audio connection as well
            self._terminate_audio_connection() 
            # exit the client
            sys.exit(1)

    def _close_socket_connection(self):
        self.socket.close()

    def _close_stream(self):
        self.stream.close()

    def _stop_stream(self):
        self.stream.stop_stream()
    
    def _start_stream(self):
        self.stream.start_stream()

    def _terminate_audio_connection(self):
        self.audio.terminate()

    def _select_record_device(self):
        print("----------------------INVALID INPUT DEVICE INDEX-------------")
        print("----------------------Select record device list--------------")
        info = self.audio.get_host_api_info_by_index(0)
        numdevices = info.get('deviceCount')

        for i in range(0, numdevices):
                if (self.audio.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
                    print("Input Device id ", i, " - ", self.audio.get_device_info_by_host_api_device_index(0, i).get('name'))
        print("-------------------------------------------------------------")

        self.device_index = int(input())
        print("recording via index "+str(self.device_index))