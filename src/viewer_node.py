#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage

class MinimalViewer(Node):
    def __init__(self):
        super().__init__('minimal_viewer')
        self.create_subscription(CompressedImage, '/camera/camera/color/image_raw/compressed', self.color_cb, 10)
        self.create_subscription(CompressedImage, '/camera/camera/depth/image_rect_raw/compressedDepth', self.depth_cb, 10)

    def color_cb(self, msg):
        cv2.imshow("Color", cv2.imdecode(np.frombuffer(msg.data, np.uint8), 1))
        cv2.waitKey(1)

    def depth_cb(self, msg):
        #alpha scaling and colormap can be modified!!!
        cv2.imshow("Depth", cv2.applyColorMap(cv2.convertScaleAbs(cv2.imdecode(np.frombuffer(msg.data[12:], np.uint8), -1), alpha=0.1), cv2.COLORMAP_JET))
        cv2.waitKey(1)

def main():
    rclpy.init()
    rclpy.spin(MinimalViewer())
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()    