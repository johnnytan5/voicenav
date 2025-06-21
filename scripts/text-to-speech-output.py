#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from gtts import gTTS
import pygame
import threading
import os

current_audio_lock = threading.Lock()
current_audio_channel = None

def play_audio(text):
    global current_audio_channel

    # Generate audio file
    tts = gTTS(text=text, lang='en')
    tts.save("/tmp/msg.mp3")

    # Load and play using pygame mixer
    pygame.mixer.init()
    pygame.mixer.music.load("/tmp/msg.mp3")
    pygame.mixer.music.play()

    # Wait for the sound to finish playing
    while pygame.mixer.music.get_busy():
        rospy.sleep(0.1)

def callback_receive_radio_data(msg):
    global current_audio_channel

    text = msg.data
    rospy.loginfo(f"🔊 Message received: {text}")

    with current_audio_lock:
        if pygame.mixer.get_init() and pygame.mixer.music.get_busy():
            rospy.loginfo("⏹ Stopping current audio.")
            pygame.mixer.music.stop()  # Interrupt playback

        threading.Thread(target=play_audio, args=(text,)).start()

if __name__ == '__main__':
    rospy.init_node('smartphone')
    rospy.loginfo("📱 Audio subscriber started.")

    sub = rospy.Subscriber("/robot_news_radio", String, callback_receive_radio_data)

    rospy.spin()
