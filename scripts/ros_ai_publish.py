#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import traceback

import numpy as np
from PIL import Image as PILImage

import torch
from torchvision.transforms import functional as F

from ultralytics import YOLO

model = YOLO("best.pt")

# Callback function to process the received image
def image_callback(msg, pub):
    try:
        # Convert the ROS image message to OpenCV format
        bridge = CvBridge()
        np_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Pass the image through the model for inference
        result = model(np_image)

        # Get the visualized image with bounding boxes and labels
        visualized_image = result[0].plot()

        # Convert the visualized image to OpenCV format
        cv_image = cv2.cvtColor(np.array(visualized_image), cv2.COLOR_RGB2BGR)

        # Convert the OpenCV image to ROS Image message
        modified_msg = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

        # Publish the modified image message
        pub.publish(modified_msg)

    except Exception as e:
        rospy.logerr("Error: %s" % str(e))
        rospy.logdebug("Error traceback: %s" % traceback.format_exc())


def main():
    # Initialize the ROS node
    rospy.init_node('image_processor', anonymous=True, log_level=rospy.DEBUG)

    pub = rospy.Publisher("/ai/processed", Image, queue_size=10)

    # Create a subscriber to the image topic
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback, pub)

    # Spin the ROS node
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
