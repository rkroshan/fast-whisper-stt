from record_client import record_client

client = record_client(
    host='127.0.0.1',
    port=8091,
    int_wav_file_dir='/home/rkroshan/fast-whisper-stt/wavfiles',
    record_seconds=3
)

client()