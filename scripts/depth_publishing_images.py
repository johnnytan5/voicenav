import rospy
from sensor_msgs.msg import Image

def callback(msg):
    print("Encoding:", msg.encoding)
    rospy.signal_shutdown("Got encoding")

rospy.init_node("encoding_checker")
rospy.Subscriber("/camera/depth/image_raw", Image, callback)
rospy.spin()


