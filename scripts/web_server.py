#!/usr/bin/env python3
from pyngrok import conf, ngrok
from flask import Flask, render_template, Response
from flask_socketio import SocketIO
import rospy
from rosgraph_msgs.msg import Log
import threading
from camera_stream import gen_frames, gen_depth_frames, init_camera_streams
import queue


conf.get_default().auth_token = "2yiQ6hGsXRX0zPcbzjZw9twUDaL_3oDCjGudobXxPiVtPdwGS"


public_url = ngrok.connect(5000, domain="suitable-phoenix-suitable.ngrok-free.app")


print(f" * Ngrok Tunnel: {public_url}")
# Flask + SocketIO setup
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# Mic streaming toggle
listening = True
audio_buffer = queue.Queue()

# ===============================
# Flask routes
# ===============================
@app.route('/')
def index():
    return render_template('index.html')

# @app.route('/video_feed')
# def video_feed():
#     return Response(gen_frames(),
#                     mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/depth_feed')
def depth_feed():
    return Response(gen_depth_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# ===============================
# ROS Logging
# ===============================
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

# ===============================
# SocketIO Event Handlers
# ===============================
@socketio.on('audio_chunk')
def handle_audio_chunk(audio_data):
    global listening
    if listening:
        # Save or forward audio_data to speaker/audio player
        audio_buffer.put(audio_data)
        # Optionally: publish to a ROS topic or play directly via PyAudio

@socketio.on('toggle_listen')
def handle_toggle_listen(state):
    global listening
    listening = bool(state)
    rospy.loginfo(f"[WebUI] Listening state set to: {listening}")

# ===============================
# Main
# ===============================
if __name__ == '__main__':
    rospy.init_node('web_ros_interface', anonymous=True)

    # Start ROS log thread
    t = threading.Thread(target=ros_log_listener)
    t.daemon = True
    t.start()

    # Start Flask web server
    socketio.run(app, host='0.0.0.0', port=5000)
