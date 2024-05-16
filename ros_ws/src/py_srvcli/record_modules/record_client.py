import pyaudio
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
        try:
            x = input("press any key to start recording...")
            print("recording started")
            self._open_record_stream()
            self._record()
            print ("recording stopped")
            self._convert_to_wavefile()
            self._stop_stream()
            self._close_stream()
        except KeyboardInterrupt:
            print("Interrupted by user")
            self._close_everything()

    def _del_wavefiles(self):
        for root, dirs, files in os.walk(self.int_wav_file_dir):
            for f in files:
                os.unlink(os.path.join(root, f))
            for d in dirs:
                shutil.rmtree(os.path.join(root, d))

    def get_wavefilename(self):
        return self.wavefilename

    def del_wavefile(self):
        # del the wav file
        os.remove(self.wavefilename)

    def _convert_to_wavefile(self):
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
        except Exception as e:
            print("_convert_to_wavefile error :")
            print(e)
            self._close_everything()

    def _close_everything(self):
        self._stop_stream()
        self._close_stream()
        self._terminate_audio_connection()
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
            self._terminate_audio_connection()
            sys.exit(1)

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