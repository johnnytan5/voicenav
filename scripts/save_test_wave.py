# save_test_wav.py
import wave
import numpy as np

RATE = 16000
CHANNELS = 1
DURATION = 1  # seconds

samples = (np.sin(2 * np.pi * 440 * np.arange(RATE * DURATION) / RATE) * 32767).astype(np.int16)

with wave.open("/tmp/test_audio.wav", 'wb') as wf:
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(2)
    wf.setframerate(RATE)
    wf.writeframes(samples.tobytes())
