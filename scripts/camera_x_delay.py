#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DepthOverlay:
    def __init__(self):
        self.bridge = CvBridge()

        # Subscribers for RGB and Depth images
        self.rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

        # Publisher for depth overlay image
        self.overlay_pub = rospy.Publisher("/camera/depth_overlay/image_raw", Image, queue_size=1)

        self.rgb_image = None
        self.depth_image = None

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def depth_callback(self, msg):
        depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if depth_raw.dtype == np.uint16:
            self.depth_image = depth_raw.astype(np.float32) / 1000.0  # mm to meters
        else:
            self.depth_image = depth_raw

    def run(self):
        rospy.loginfo("Publishing depth overlay...")
        rate = rospy.Rate(30)

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

                alpha = 1.0 - depth_norm
                alpha = np.repeat(alpha[:, :, np.newaxis], 3, axis=2)

                blended = rgb.copy()
                blend_ratio = 0.3

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

                # Publish the blended overlay image
                try:
                    overlay_msg = self.bridge.cv2_to_imgmsg(blended, encoding="bgr8")
                    self.overlay_pub.publish(overlay_msg)
                except Exception as e:
                    rospy.logerr(f"Failed to publish overlay image: {e}")

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("depth_x_delay_overlay_node")
    overlay = DepthOverlay()
    overlay.run()
