#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
from vosk import Model, KaldiRecognizer
import json
import time

last_mode_query_time = 0
current_mode = ""
last_published_mode = ""
pub_mode = None
pub_command = None
pub_sound = None
pub = None

# Load Vosk model
model = Model("/home/mustar/catkin_ws/src/blindassistant/src/vosk-model")
recognizer = KaldiRecognizer(model, 16000)

def audio_callback(msg):
    global current_mode, last_published_mode

    pcm_data = bytes(msg.data)

    if recognizer.AcceptWaveform(pcm_data):
        result = recognizer.Result()
        text = json.loads(result).get("text", "").lower().strip()

        if text:
            rospy.loginfo("You said: %s", text)
            pub.publish(text)

            if "walking" in text and current_mode != "walking":
                current_mode = "walking"
                rospy.loginfo("Mode set to walking")

            if "scanning" in text and current_mode != "scanning":
                current_mode = "scanning"
                rospy.loginfo("Mode set to scanning")

            if "exit" in text:
                current_mode = ""
                rospy.loginfo("Exited mode; publishing empty")

            # Only publish if the mode has changed
            if current_mode != last_published_mode:
                pub_mode.publish(current_mode)
                last_published_mode = current_mode

            if "snap" in text:
                pub_command.publish("snap")
                rospy.loginfo("Command Published: snap")

            if "email" in text:
                pub_command.publish("email")
                rospy.loginfo("Command Published: email")

            if "mode" in text:
                pub_sound.publish(f"Current status: {current_mode or 'none'}")
                rospy.loginfo("Current Mode updated to user.")
                

def main():
    global pub, pub_mode, pub_command, pub_sound

    rospy.init_node('audio_in')
    pub = rospy.Publisher('/Input', String, queue_size=10)
    pub_mode = rospy.Publisher('/Mode', String, queue_size=10)
    pub_command = rospy.Publisher('/Command', String, queue_size=10)
    pub_sound = rospy.Publisher('/robot_news_radio', String, queue_size=10)

    rospy.Subscriber("/audio", AudioData, audio_callback)

    rospy.loginfo("Vosk speech recognizer node started, listening to /audio")

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
