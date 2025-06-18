#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from gtts import gTTS
import playsound

def callback_receive_radio_data(msg):

    rospy.loginfo("Message received: ")
    text = msg.data
    rospy.loginfo(text)

    tts = gTTS (text = text, lang = 'en')
    tts.save("msg.mp3")
    playsound.playsound("msg.mp3")

if __name__ == '__main__':
    rospy.init_node('text-to-speech-output')

    sub = rospy.Subscriber("/robot_news_radio", String, callback_receive_radio_data)

    rospy.spin()