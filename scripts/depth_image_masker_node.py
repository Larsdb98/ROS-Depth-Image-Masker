#!/usr/bin/env python

########################################################################

import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
import cv_bridge
# from cv_bridge import CvBridge

import numpy as np


class DepthImageMasker:
    def __init__(self):
        # In case this node needs to be used multiple times:
        # argument "anonymous" needs to be true.
        rospy.init_node("depth_image_masker", anonymous=True)

        # Fetch private params
        self.__image_in = rospy.get_param("~depth_image_in_topic", "/wrist_camera/depth/image_rect_raw")
        self.__image_out = rospy.get_param("~depth_image_out_topic", "/wrist_camera/depth/image_rect_raw_masked")

        # Get all mask parameters
        self.__x_lower = rospy.get_param("~x_lower", 200)
        self.__x_upper = rospy.get_param("~x_upper", 300)
        self.__y_lower = rospy.get_param("~y_lower", 200)
        self.__y_upper = rospy.get_param("~y_upper", 300)

        # Subscribers publishers
        self.image_sub = rospy.Subscriber(self.__image_in, Image, self.depth_image_callback)
        self.image_pub = rospy.Publisher(self.__image_out, Image, queue_size=1)

        # Bridge instance
        self.bridge = cv_bridge.CvBridge()
        self.depth_image_curr = None



    def depth_image_callback(self, depth_image):
        self.depth_image_curr = depth_image # store message, important for header information 
        try:
            # Convert ROS Image message to OpenCV imahe
            cv_image = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)
            return
        
        # Apply rectangular mask
        mask = np.zeros_like(cv_image)
        mask[self.__y_lower:self.__y_upper, self.__x_lower:self.__x_upper] = 1

        # Apply the mask to set values inside the rectangle to 0:
        masked_depth_image = cv_image * (1 - mask)

        if self.depth_image_curr is not None:
            # Publish the new depth image
            try:
                masked_depth_image_msg = self.bridge.cv2_to_imgmsg(masked_depth_image, encoding="passthrough")

                # Set the timestamp & frame_id
                # Simply copy them from the recieved camera_info messages
                masked_depth_image_msg.header.stamp = self.depth_image_curr.header.stamp
                masked_depth_image_msg.header.frame_id = self.depth_image_curr.header.frame_id
                self.image_pub.publish(masked_depth_image_msg)
            except cv_bridge.CvBridgeError as e:
                rospy.logerr(e)



    def run(self):
        rospy.spin()


### Main function
def main():
    process = DepthImageMasker()
    process.run()



#####################################################3
if __name__ == "__main__":
    main()
        