#!/usr/bin/env python3
from flask import Flask, render_template, Response
from flask_socketio import SocketIO
import rospy
from rosgraph_msgs.msg import Log
from camera_stream import gen_frames, gen_depth_frames, init_camera_streams
import threading
import queue
import sounddevice as sd
import numpy as np
import time
import wave
import os
from audio_common_msgs.msg import AudioData
import io
from pyngrok import conf, ngrok
import subprocess
import tempfile

conf.get_default().auth_token = "2yiQ6hGsXRX0zPcbzjZw9twUDaL_3oDCjGudobXxPiVtPdwGS"
public_url = ngrok.connect(5000, domain="suitable-phoenix-suitable.ngrok-free.app")
print(f" * Ngrok Tunnel: {public_url}")

CHUNK = 1024
RATE = 16000
CHANNELS = 1
audio_buffer = queue.Queue()
listening = True

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
os.environ["SDL_AUDIODRIVER"] = "dummy"

@app.route('/')
def index():
   return render_template('index.html')


@app.route('/depth_feed')
def depth_feed():
   return Response(gen_depth_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def ros_log_callback(msg):
   log_data = {
       'node': msg.name,
       'level': msg.level,
       'msg': msg.msg,
       'timestamp': msg.header.stamp.to_sec()
   }
   socketio.emit('ros_log', log_data)


def ros_log_listener():
   rospy.Subscriber("/rosout", Log, ros_log_callback)
   init_camera_streams()
   rospy.spin()

@socketio.on('audio_chunk')
def handle_audio_chunk(data):
    rospy.loginfo(f"[Talk] Received {len(data)} bytes from browser")

    with tempfile.NamedTemporaryFile(delete=False, suffix='.webm') as tmp:
        tmp.write(data)
        tmp.flush()

        # decode to WAV PCM (mono, 16kHz, 16-bit signed)
        try:
            subprocess.run([
                'ffmpeg', '-y', '-i', tmp.name,
                '-f', 's16le', '-acodec', 'pcm_s16le',
                '-ac', '1', '-ar', '16000',  
                '/tmp/walkie.raw'
            ], check=True)

            with open('/tmp/walkie.raw', 'rb') as f:
                raw_pcm = f.read()

            audio_buffer.put(raw_pcm)

        except subprocess.CalledProcessError:
            rospy.logerr("[Talk] ffmpeg failed to decode audio")

@socketio.on('toggle_listen')
def handle_toggle_listen(state):
   global listening
   listening = bool(state)
   rospy.loginfo(f"[WebUI] Listening state set to: {listening}")

def to_wav_bytes(pcm_bytes):
   np_audio = np.frombuffer(pcm_bytes, dtype=np.int16)

   # Normalize volume 
   volume_factor = 0.5  # Reduce if noisy, increase if too quiet (try 0.3 ~ 1.0)
   np_audio = (np_audio * volume_factor).astype(np.int16)


   buffer = io.BytesIO()
   with wave.open(buffer, 'wb') as wf:
       wf.setnchannels(CHANNELS)
       wf.setsampwidth(2)
       wf.setframerate(RATE)
       wf.writeframes(np_audio.tobytes())
   return buffer.getvalue()

def audio_capture_callback(msg):
#    wav_bytes = to_wav_bytes(bytes(msg.data))
   socketio.emit('robot_audio', msg.data)

def robot_mic_stream():
   rospy.Subscriber('/audio', AudioData, audio_capture_callback)
   rospy.loginfo("[Mic] Subscribed to /audio")
   rospy.spin()

audio_buffer = queue.Queue(maxsize=50)  

def playback_worker():
    try:
        with sd.OutputStream(
            channels=CHANNELS, 
            samplerate=RATE, 
            blocksize=CHUNK,
            dtype='int16',               
            latency='high'
        ) as stream:
            rospy.loginfo("[Speaker] Playback started")
            last_time = time.time()

            while True:
                try:
                    elapsed = time.time() - last_time
                    sleep_time = max(0, (CHUNK/RATE) - elapsed)
                    time.sleep(sleep_time)

                    data = audio_buffer.get()

                    # Convert to numpy int16 safely
                    if isinstance(data, bytes):
                        if len(data) % 2 != 0:
                            rospy.logwarn(f"[Speaker] Dropping packet, invalid size: {len(data)}")
                            continue
                        buffer = np.frombuffer(data, dtype=np.int16)
                    elif isinstance(data, np.ndarray) and data.dtype == np.float32:
                        buffer = (data * 32767).astype(np.int16)
                    elif isinstance(data, np.ndarray):
                        buffer = data.astype(np.int16)
                    else:
                        rospy.logwarn("[Speaker] Unexpected data format.")
                        continue

                    if buffer.size == 0:
                        continue

                    stream.write(buffer)
                    last_time = time.time()

                except queue.Empty:
                    time.sleep(0.001)
                except Exception as e:
                    rospy.logerr(f"[Speaker] Playback error: {e}")
    except Exception as e:
        rospy.logerr(f"[Speaker] Stream failed: {e}")
        
if __name__ == '__main__':
   rospy.init_node('web_ros_interface', anonymous=True)

   # ROS log tab listener
   threading.Thread(target=ros_log_listener, daemon=True).start()

   # Receive audio from browser and play via speaker
   threading.Thread(target=playback_worker, daemon=True).start()

   # Start looped playback of WAV file to browser
   threading.Thread(target=robot_mic_stream, daemon=True).start()

   # Start Flask server
   socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)




