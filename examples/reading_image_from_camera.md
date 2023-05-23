The following code block creates a node called image_saver, which subscribes to the /camera/color/image_raw topic and saves it to a file
since ROS images are their own format, we are using cv2 to convert the image to a regular image format.

**FUTURE**:
- Instead of using cv2.imwrite() to write the file, we could pass the image to the ai
```
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Callback function to process the received image
def image_callback(msg):
    try:
        # Convert the ROS image message to OpenCV format
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Save the image to a file
        cv2.imwrite("image.jpg", cv_image)
        
        rospy.loginfo("Image saved successfully!")
        
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
