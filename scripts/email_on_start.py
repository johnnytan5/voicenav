#!/usr/bin/env python3

import smtplib
from email.mime.text import MIMEText
import rospy
from std_msgs.msg import String

class EmailSender:
    def __init__(self):
        rospy.init_node("email_sender")

        self.latest_command = ""
        self.latest_mode = ""
        self.has_processed = False

        # Subscribers
        self.command_sub = rospy.Subscriber("/Command", String, self.command_callback)
        self.mode_sub = rospy.Subscriber("/Mode", String, self.mode_callback)

        # Publishers
        self.command_pub = rospy.Publisher("/Command", String, queue_size=10)

        rospy.loginfo("EmailSender is running. Waiting for /Command == 'email'...")

        # Optional: send once on startup
        self.send_email(startup=True)

    def command_callback(self, msg):
        self.latest_command = msg.data.strip().lower()
        rospy.loginfo(f"Received /Command: {self.latest_command}")
        self.check_trigger()

    def mode_callback(self, msg):
        self.latest_mode = msg.data.strip().lower()
        rospy.loginfo(f"Received /Mode: {self.latest_mode}")
        self.check_trigger()

    def check_trigger(self):
        # Avoid race conditions: immediately lock trigger
        if self.has_processed:
            return

        if self.latest_command == "email":
            rospy.loginfo("Trigger condition met: sending email.")
            self.has_processed = True  # lock
            self.send_email()
            rospy.sleep(1.0)
            self.command_pub.publish("")  # Clear command
            self.latest_command = ""
            rospy.Timer(rospy.Duration(0.5), self.reset_flags, oneshot=True)


    def send_email(self, startup=False):
        recipients = ["limziyang5119@gmail.com", "haowentan5@gmail.com"]

        body = """
Dear friend,

I hope this message finds you well.

My name is Hao Wen Tan, and I’m writing to kindly ask for your help. As someone who is visually impaired, I depend on the support of friends like you to help me monitor and manage the Blind Assistant system, which just launched successfully on my robot.

The system has automatically started and all critical components are now active:

- Core ROS nodes have initialized  
- The camera and audio modules are up and running  
- The remote interface is now online and ready for access

You can connect to the system using the link below:

Link: http://suitable-phoenix-suitable.ngrok-free.app

If you could spare a moment to check that the system is working as expected — video feed visible, audio functional, and no unusual behavior — I would deeply appreciate it. If you notice anything out of place or if the system appears unresponsive, please let me know so I can try to fix it.

Your help means more than I can express. Being able to rely on your support allows me to run this project more independently and safely.

Thank you so much for your kindness and your time.

With warmest regards,  
Hao Wen Tan  
(Blind Assistant System – Jupiter)
"""

        msg = MIMEText(body, "plain", "utf-8")
        msg["Subject"] = "Blind Assistant Started – Kindly Requesting Your Help"
        msg["From"] = "zylim.test@gmail.com"
        msg["To"] = ", ".join(recipients)

        try:
            server = smtplib.SMTP_SSL("smtp.gmail.com", 465)
            server.login("zylim.test@gmail.com", "ehhursreaupxhqmk")
            server.send_message(msg)
            server.quit()
            status = "startup" if startup else "command"
            rospy.loginfo(f"Email sent successfully via {status} trigger.")
        except Exception as e:
            rospy.logerr(f"Failed to send email: {e}")

    def reset_flags(self, event):
        self.has_processed = False
        rospy.loginfo("Email trigger reset. Ready for next email command.")

if __name__ == "__main__":
    try:
        EmailSender()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
