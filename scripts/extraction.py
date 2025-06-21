#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from dotenv import load_dotenv
import os
import cv2
import base64
import requests

class Extraction:
    def __init__(self):
        # Load .env.local
        env_path = os.path.join(os.path.dirname(__file__), '..', '.env.local')
        load_dotenv(dotenv_path=env_path)

        self.api_key = os.getenv("OPENAI_API_KEY")

        rospy.init_node("extraction")
        self.bridge = CvBridge()
        self.latest_command = ""
        self.latest_mode = ""
        self.has_processed = False
        self.latest_image = None

        # Subscribers
        self.command_sub = rospy.Subscriber("/Command", String, self.command_callback)
        self.mode_sub = rospy.Subscriber("/Mode", String, self.mode_callback)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        # Publisher
        self.tts_pub = rospy.Publisher("robot_news_radio", String, queue_size=10)

        rospy.loginfo("🟢 Extraction node is active. Waiting for /Command and /Mode...")

    def command_callback(self, msg):
        self.latest_command = msg.data.strip().lower()
        rospy.loginfo(f"📥 Received /Command: {self.latest_command}")
        self.check_trigger()

    def mode_callback(self, msg):
        self.latest_mode = msg.data.strip().lower()
        rospy.loginfo(f"📥 Received /Mode: {self.latest_mode}")
        self.check_trigger()

    def check_trigger(self):
        if self.has_processed:
            return

        rospy.loginfo(f"Checking criteria, /Command: {self.latest_command}, /Mode: {self.latest_mode}, /isProcessing: {self.has_processed}")
        if self.latest_command == "snap" and self.latest_mode == "scanning":
            rospy.loginfo("✅ Trigger condition met: scanning for extraction.")
            self.has_processed = True
            if self.latest_image is not None:
                self.send_to_openai(self.latest_image)
            else:
                rospy.logwarn("⚠️ No image available yet. Will process next image.")

    def image_callback(self, msg):
        if self.has_processed:
            return

        if self.latest_command == "snap" and self.latest_mode == "scanning":
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                self.latest_image = cv_image
                rospy.loginfo("📸 Captured image from camera.")
                self.send_to_openai(cv_image)
                self.has_processed = True
            except Exception as e:
                rospy.logerr(f"❌ Failed to convert image: {e}")
        else:
            try:
                self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            except:
                pass  # don't crash on occasional conversion errors

    def send_to_openai(self, cv_image):
        try:
            _, buffer = cv2.imencode('.png', cv_image)
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            image_url = f"data:image/png;base64,{image_base64}"

            headers = {
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json"
            }

            payload = {
                "model": "gpt-4o-mini",
                "messages": [
                    {"role": "system", "content": "You are a helpful assistant that describes images."},
                    {
                        "role": "user",
                        "content": [
                            {"type": "image_url", "image_url": {"url": image_url}},
                            {"type": "text", "text": "Look at the picture. If there are text information, help me extract important information such as expiry date and nutrition information. If not, just describe what object is there. Note that you are helping a visually impaired individual, so be concise and informative. Maximum response: 2 sentences with max 15 words."}
                        ]
                    }
                ],
                "temperature": 0.2
            }

            response = requests.post("https://api.openai.com/v1/chat/completions", headers=headers, json=payload)
            result = response.json()
            message = result["choices"][0]["message"]["content"]
            rospy.loginfo(f"🧠 OpenAI GPT-4o response:\n{message}")
            self.tts_pub.publish(message)

        except Exception as e:
            rospy.logerr(f"❌ Failed to call OpenAI API: {e}")

        # Reset after a delay
        rospy.Timer(rospy.Duration(1.0), self.reset_flags, oneshot=True)

    def reset_flags(self, event):
        self.latest_command = ""
        self.has_processed = False
        rospy.loginfo("🔁 Trigger reset. Ready for next command.")

if __name__ == "__main__":
    try:
        Extraction()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
