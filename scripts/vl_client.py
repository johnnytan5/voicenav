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



class VLClient:
    def __init__(self):
        # Get path to .env.local in the parent directory
        env_path = os.path.join(os.path.dirname(__file__), '..', '.env.local')
        load_dotenv(dotenv_path=env_path)

        # Access your keys
        openai_key = os.getenv("OPENAI_API_KEY")

        rospy.init_node("vl_client_http")
        self.bridge = CvBridge()
        self.latest_command = ""
        self.latest_mode = ""
        self.has_processed = False  # avoid multiple triggers
        self.api_key = openai_key  # Keep this safe

        # Subscribers
        self.command_sub = rospy.Subscriber("/Command", String, self.command_callback)
        self.mode_sub = rospy.Subscriber("/Mode", String, self.mode_callback)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        #Publishers
        self.tts_pub = rospy.Publisher("robot_news_radio", String, queue_size=10)

        self.latest_image = None

        rospy.loginfo("🟢 VLClient is running. Waiting for /Command and /Mode...")

    def command_callback(self, msg):
        self.latest_command = msg.data.strip().lower()
        rospy.loginfo(f"📥 Received /Command: {self.latest_command}")
        self.check_trigger()

    def mode_callback(self, msg):
        self.latest_mode = msg.data.strip().lower()
        rospy.loginfo(f"📥 Received /Mode: {self.latest_mode}")
        self.check_trigger()

    def check_trigger(self):
        rospy.loginfo(f"Checking criteria, /Command: {self.latest_command}, /Mode: {self.latest_mode}, /isProcessing: {self.has_processed}")
        if self.latest_command == "scan" and self.latest_mode == "walking" and not self.has_processed:
            rospy.loginfo("✅ Trigger condition met: scanning while walking.")
            self.has_processed = True
            if self.latest_image is not None:
                self.send_to_openai(self.latest_image)
            else:
                rospy.logwarn("⚠️ No image available yet. Will process next image.")

    def image_callback(self, msg):
        if not self.has_processed and self.latest_command == "scan" and self.latest_mode == "walking":
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
                pass  # ignore non-process image updates

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
                            {"type": "text", "text": "Imagine this is a camera held by a visually impaired individual. Describe the environment and guide them. Be informative but concise."}
                        ]
                    }
                ],
                "temperature": 0.2
            }

            response = requests.post("https://api.openai.com/v1/chat/completions",
                                     headers=headers, json=payload)
            result = response.json()
            message = result["choices"][0]["message"]["content"]
            rospy.loginfo(f"🧠 OpenAI GPT-4o response:\n{message}")
            self.tts_pub.publish(message)

            
        except Exception as e:
            rospy.logerr(f"❌ Failed to call OpenAI API: {e}")

        # Reset trigger so you can say scan again later
        self.latest_command = ""
        self.has_processed = False

if __name__ == "__main__":
    try:
        VLClient()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
