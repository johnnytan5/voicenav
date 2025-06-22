#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

class DepthOverlay:
    def __init__(self):
        rospy.init_node("depth_overlay_node")
        self.bridge = CvBridge()

        # Subscribers
        self.rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        self.mode_sub = rospy.Subscriber("/Mode", String, self.mode_callback)

        # Publisher
        self.tts_pub = rospy.Publisher("robot_news_radio", String, queue_size=10)

        self.rgb_image = None
        self.depth_image = None
        self.latest_mode = ""

        # Test the publisher once
        rospy.sleep(1.0)
        self.tts_pub.publish("Voice nav activated, web interface activated, please tell me the mode and command, i am delighted to assist you")

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def mode_callback(self, msg):
        self.latest_mode = msg.data.strip()


    def depth_callback(self, msg):
        depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if depth_raw.dtype == np.uint16:
            self.depth_image = depth_raw.astype(np.float32) / 1000.0
        else:
            self.depth_image = depth_raw

    def run(self):
        rospy.loginfo("Running Depth Overlay...")
        rate = rospy.Rate(0.2)

        while not rospy.is_shutdown():
            if self.rgb_image is not None and self.depth_image is not None:
                rgb = self.rgb_image.copy()
                depth = self.depth_image.copy()

                if rgb.shape[:2] != depth.shape[:2]:
                    depth = cv2.resize(depth, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)

                depth_norm = np.clip((depth - 0.3) / (4.0 - 0.3), 0.0, 1.0)

                near_mask = (depth <= 1.0).astype(np.uint8) * 255
                medium_mask = ((depth > 1.0) & (depth <= 2.5)).astype(np.uint8) * 255
                far_mask = ((depth > 2.5) & (depth <= 4.0)).astype(np.uint8) * 255

                red = np.full_like(rgb, (0, 0, 255))
                yellow = np.full_like(rgb, (0, 255, 255))
                blue = np.full_like(rgb, (255, 0, 0))

                blend_ratio = 0.3
                blended = rgb.copy()

                blended = np.where(near_mask[:, :, np.newaxis] == 255,
                    (blended * (1 - blend_ratio) + red * blend_ratio).astype(np.uint8), blended)
                blended = np.where(medium_mask[:, :, np.newaxis] == 255,
                    (blended * (1 - blend_ratio) + yellow * blend_ratio).astype(np.uint8), blended)
                blended = np.where(far_mask[:, :, np.newaxis] == 255,
                    (blended * (1 - blend_ratio) + blue * blend_ratio).astype(np.uint8), blended)

                for mask, label_color, label_text in zip(
                    [near_mask, medium_mask, far_mask],
                    [(0, 0, 255), (0, 255, 255), (255, 0, 0)],
                    ["Near", "Medium", "Far"]
                ):
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    for cnt in contours:
                        if cv2.contourArea(cnt) > 500:
                            x, y, w, h = cv2.boundingRect(cnt)
                            cv2.rectangle(blended, (x, y), (x + w, y + h), label_color, 2)
                            M = cv2.moments(cnt)
                            if M["m00"] != 0:
                                cX = int(M["m10"] / M["m00"])
                                cY = int(M["m01"] / M["m00"])
                                patch = depth[max(cY - 2, 0):cY + 3, max(cX - 2, 0):cX + 3]
                                valid = patch[(patch > 0) & (~np.isnan(patch))]
                                if valid.size > 0:
                                    center_depth = float(np.median(valid))
                                    label = f"{label_text}: {center_depth:.2f}m"
                                    cv2.circle(blended, (cX, cY), 4, label_color, -1)
                                    cv2.putText(blended, label, (cX - 40, cY - 10),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, label_color, 2)

                # Zone-based detection
                height, width = depth.shape
                zone_height = height // 3
                zone_width = width // 3

                directions = [
                    "top left", "top center", "top right",
                    "middle left", "center", "middle right",
                    "bottom left", "bottom center", "bottom right"
                ]

                obstacle_detected = False
                for i in range(3):
                    for j in range(3):
                        y_start, y_end = i * zone_height, (i + 1) * zone_height
                        x_start, x_end = j * zone_width, (j + 1) * zone_width
                        region = depth[y_start:y_end, x_start:x_end]
                        valid_depth = region[(region > 0) & (~np.isnan(region))]

                        if valid_depth.size > 0:
                            min_depth = np.min(valid_depth)
                            rospy.loginfo(f"Zone [{i},{j}] '{directions[i*3 + j]}': min depth = {min_depth:.2f}m, count = {valid_depth.size}")

                            if min_depth < 0.8 and self.latest_mode.lower() == "walking":  
                                direction = directions[i * 3 + j]
                                msg = f"Obstacle detected in {direction}"
                                rospy.logwarn(msg)
                                self.tts_pub.publish(msg)
                                cv2.putText(blended, msg, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                                obstacle_detected = True
                                break
                    if obstacle_detected:
                        break

                if cv2.waitKey(1) == 27:
                    break

            rate.sleep()

if __name__ == '__main__':
    try:
        overlay = DepthOverlay()
        overlay.run()
    except rospy.ROSInterruptException:
        pass
