#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
from vosk import Model, KaldiRecognizer
import wave
import json

current_mode = ""
pub_mode = None
pub_command = None
pub_sound = None
pub = None

# Load Vosk model
model = Model("/home/mustar/catkin_ws/src/blindassistant/src/vosk-model")  # Change to actual path
recognizer = KaldiRecognizer(model, 16000)

def mode_timer_callback(event):
    global current_mode
    if current_mode:
        pub_mode.publish(current_mode)
        rospy.loginfo("Mode Node Re-published: %s", current_mode)

def audio_callback(msg):
    global current_mode

    # Convert AudioData msg to PCM bytes
    pcm_data = bytes(msg.data)

    if recognizer.AcceptWaveform(pcm_data):
        result = recognizer.Result()
        text = json.loads(result).get("text", "").lower().strip()

        if text:
            rospy.loginfo("You said: %s", text)
            pub.publish(text)
            if "walking" in text:
                current_mode = "walking"
                rospy.loginfo("Mode set to walking")

            if "scanning" in text:
                current_mode = "scanning"
                rospy.loginfo("Mode set to scanning")

            if "exit" in text:
                current_mode = ""
                rospy.loginfo("Exited mode; publishing empty")
                pub_mode.publish("")

            if "snap" in text:
                pub_command.publish("snap")
                rospy.loginfo("Command Published: snap")

            if "email" in text:
                pub_command.publish("email")
                rospy.loginfo("Command Published: email")

            if "mode" in text:
                pub_sound.publish(f"Current Mode: {current_mode}")
                rospy.loginfo("Current Mode updated to user.")
                msg = ""

            
    else:
        # Partial result (not used here)
        pass

def main():
    global pub, pub_mode, pub_command, pub_sound

    rospy.init_node('audio_in_vosk')
    pub = rospy.Publisher('/Input', String, queue_size=10)
    pub_mode = rospy.Publisher('/Mode', String, queue_size=10)
    pub_command = rospy.Publisher('/Command', String, queue_size=10)
    pub_sound = rospy.Publisher('/robot_news_radio', String, queue_size=10)

    rospy.Subscriber("/audio", AudioData, audio_callback)

    rospy.loginfo("Vosk speech recognizer node started, listening to /audio")

    # Timer to re-publish current mode every 1s
    rospy.Timer(rospy.Duration(1.0), mode_timer_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass