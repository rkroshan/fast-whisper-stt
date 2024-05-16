import sys
from faster_whisper import WhisperModel
import logging


class record_server:

    def __init__(
        self,
        model_size = 'tiny.en',
        device = 'cpu',
        compute_type = 'int8',
        beam_size = 5,
    ):
        self.model_size = model_size
        self.device = device
        self.compute_type = compute_type
        self.beam_size = beam_size
        self.model = None
        #init model
        self._init_model()

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
            # exit the server
            sys.exit(1)
        

    def _init_model(self):
        try:
            print("initialising model...")
            self.model = WhisperModel(self.model_size, device=self.device, compute_type=self.compute_type)
            # logging.basicConfig()
            # logging.getLogger("faster_whisper").setLevel(logging.DEBUG)
        except Exception as e:
            print("_init_model error :")
            print(e)
            # exit the server
            sys.exit(1)
