Using the image stream from ROS passes the data to the AI model


**FUTURE***:
- Create a ROS topic which returns the bounding box image to ROS


```python
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from roboflow import Roboflow

#API KEY
project = rf.workspace().project("buoys-4naae")
model = project.version(6).model

# Callback function to process the received image
def image_callback(msg):
    try:
        # Convert the ROS image message to OpenCV format
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # infer from image stream
        print(model.predict(cv_image, confidence=40, overlap=30).json())
        
    except Exception as e:
        rospy.logerr(e)

def main():
    # Initialize the ROS node
    rospy.init_node('image_saver', anonymous=True)
    
    # Create a subscriber to the image topic
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    
    # Spin the ROS node
    rospy.spin()

if __name__ == '__main__':
    main()
```
