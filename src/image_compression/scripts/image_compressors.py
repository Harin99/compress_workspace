#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridgeError, CvBridge
import numpy as np
import cv2

class ImageCompressor:
    def __init__(self, input_topic, output_topic, encoding):
        self.bridge = CvBridge()
        self.encoding = encoding
        self.pub = rospy.Publisher(output_topic, CompressedImage, queue_size=1)
        self.sub = rospy.Subscriber(input_topic, Image, self.callback)

    def callback(self, data):
        try:
            if self.encoding == 'bgr8': # RGB 
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # Convert the RGB image.

                # Compress the image using JPG format.
                jpg_img = cv2.imencode('.jpg', cv_image)[1]

                # Create a CompressedImage message.
                compressed_image = CompressedImage()
                compressed_image.header.stamp = data.header.stamp
                compressed_image.header.frame_id = data.header.frame_id
                compressed_image.format = "jpeg"
                compressed_image.data = np.array(jpg_img).tostring()
            
            elif self.encoding == '32FC1': # Depth
                cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")  # Convert the depth image.

                # Convert the depth image to a suitable type. (Unit: meter -> mm)
                 # uint16으로 하면 63m를 벗어난 depth 는 모두 63으로 들어가짐 ( 이를 방지하기위해 uint32 )
                cv_image = (cv_image * 1000).astype(np.uint16)

                # Compress the image using PNG format.
                png_img = cv2.imencode('.png', cv_image)[1]

                # Create a CompressedImage message.
                compressed_image = CompressedImage()
                compressed_image.header.stamp = data.header.stamp
                compressed_image.header.frame_id = data.header.frame_id
                compressed_image.format = "png"
                compressed_image.data = np.array(png_img).tostring()
            
            # publish compressed topic 
            self.pub.publish(compressed_image)
            
        except CvBridgeError as e:
            rospy.logerr(e)


if __name__ == "__main__":
    rospy.init_node('image_compressor')
    topics_rgb = ['/carter1/rgb_left', '/carter1/rgb_right', '/carter2/rgb_left', '/carter2/rgb_right', '/carter3/rgb_left', '/carter3/rgb_right']
    # topics_depth = ['/carter1/depth_left', '/carter1/depth_right']
    compressors_rgb = [ImageCompressor(topic, topic + "/compressed", "bgr8") for topic in topics_rgb]
    # compressors_depth = [ImageCompressor(topic, topic + "/compressed", "32FC1") for topic in topics_depth]
    rospy.spin()
