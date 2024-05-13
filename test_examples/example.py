from faster_whisper import WhisperModel
import logging

'''
basic example to decode test from wav file via faster-whisper
tiny.en model, referenced from hugging face
'''

logging.basicConfig()
logging.getLogger("faster_whisper").setLevel(logging.DEBUG)

model_size = "tiny.en"

# Run on GPU with FP16
model = WhisperModel(model_size, device="cpu", compute_type="int8")

# or run on GPU with INT8
# model = WhisperModel(model_size, device="cuda", compute_type="int8_float16")
# or run on CPU with INT8
# model = WhisperModel(model_size, device="cpu", compute_type="int8")

segments, info = model.transcribe("/home/rkrosha/faster-whisper/tests/data/jfk.flac", beam_size=5)

print("Detected language '%s' with probability %f" % (info.language, info.language_probability))

for segment in segments:
    print("[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))