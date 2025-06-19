import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
import threading

bridge = CvBridge()

# Shared frames with locks
rgb_frame = None
depth_frame = None
rgb_lock = threading.Lock()
depth_lock = threading.Lock()

# Callback for RGB stream
def rgb_callback(data):
    global rgb_frame
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        with rgb_lock:
            rgb_frame = cv_image.copy()
    except Exception as e:
        rospy.logerr(f"RGB callback error: {e}")

# Callback for depth stream
def depth_callback(data):
    global depth_frame
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        with depth_lock:
            depth_frame = cv_image.copy()
    except Exception as e:
        rospy.logerr(f"Depth callback error: {e}")

# Frame generator for RGB
def gen_frames():
    global rgb_frame
    while not rospy.is_shutdown():
        with rgb_lock:
            if rgb_frame is not None:
                small_frame = cv2.resize(rgb_frame, (320, 220))
                ret, buffer = cv2.imencode('.jpg', small_frame)
                if not ret:
                    continue
                frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# Frame generator for Depth Overlay
def gen_depth_frames():
    global depth_frame
    while not rospy.is_shutdown():
        with depth_lock:
            if depth_frame is not None:
                depth_small_frame = cv2.resize(depth_frame, (320, 220))
                ret, buffer = cv2.imencode('.jpg', depth_small_frame)
                if not ret:
                    continue
                frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# Register subscribers once
def init_camera_streams():
    rospy.Subscriber('/camera/depth_overlay/image_raw', Image, depth_callback, queue_size=1, buff_size=2**20)
