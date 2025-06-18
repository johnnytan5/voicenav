#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import speech_recognition as sr

current_mode = ""

def mode_timer_callback(event):
    global current_mode
    if current_mode:
        pub_mode.publish(current_mode)
        rospy.loginfo("Mode Node Re-published: %s", current_mode)

def main():
    global current_mode, pub_mode
    rospy.init_node('audio-in')
    pub = rospy.Publisher('/Input', String, queue_size=10)
    pub_mode = rospy.Publisher('/Mode', String, queue_size=10)
    pub_command = rospy.Publisher('/Command', String, queue_size=10)

    recognizer = sr.Recognizer()
    mic = sr.Microphone()

    with mic as source:
        recognizer.adjust_for_ambient_noise(source)
        rospy.loginfo("Speech recognizer ready. Speak something!")

    # Timer to re-publish mode every 0.1s (10Hz)
    rospy.Timer(rospy.Duration(1), mode_timer_callback)

    while not rospy.is_shutdown():
        with mic as source:
            rospy.loginfo("Listening...")
            try:
                audio = recognizer.listen(source, timeout=5, phrase_time_limit=2)
                rospy.loginfo("Recognizing...")
                text = recognizer.recognize_google(audio).lower().strip()
                rospy.loginfo("You said: %s", text)
                pub.publish(text)

                if text == "walking":
                    current_mode = "walking"
                    rospy.loginfo("Mode set to walking")

                elif text == "scanning":
                    current_mode = "scanning"
                    rospy.loginfo("Mode set to scanning")

                elif text == "exit":
                    current_mode = ""
                    rospy.loginfo("Exited mode; publishing empty")
                    pub_mode.publish("")

                elif text == "scan":
                    pub_command.publish("scan")
                    rospy.loginfo("Command Published: scan")

            except sr.WaitTimeoutError:
                rospy.logwarn("Listening timed out. No speech detected.")
            except sr.UnknownValueError:
                rospy.logwarn("Could not understand the audio.")
            except sr.RequestError as e:
                rospy.logerr("Could not request results; check your Internet: %s", e)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
