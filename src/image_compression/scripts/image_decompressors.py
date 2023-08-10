#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridgeError, CvBridge
import numpy as np
import cv2
import multiprocessing


class ImageDecompressor:
    def __init__(self, input_topic, output_topic):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher(output_topic, Image, queue_size=1)
        self.sub = rospy.Subscriber(input_topic, CompressedImage, self.callback)

    def callback(self, data):
        try:
            # Decompress the image using OpenCV.
            # uint8 : 이미지 데이터는 보통 8비트 부호 없는 정수(uint8)로 표현됨 (0 ~ 255)
            # np_arr = np.fromstring(data.data, np.uint8)
            np_arr = np.frombuffer(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_ANYCOLOR)
            
            # Convert the image to a ROS Image message.
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            image_msg.header.stamp = data.header.stamp
            image_msg.header.frame_id = data.header.frame_id
            
            # Publish the decompressed image.
            self.pub.publish(image_msg)

        except CvBridgeError as e:
            rospy.logerr(e)


def worker(input_topic, output_topic):
    rospy.init_node('image_decompressor_{}'.format(input_topic.replace("/", "_")))
    ImageDecompressor(input_topic, output_topic)
    rospy.spin()


if __name__ == "__main__":
    input_topics = ['/carter1/rgb_left/compressed', '/carter1/rgb_right/compressed', '/carter2/rgb_left/compressed', '/carter2/rgb_right/compressed', '/carter3/rgb_left/compressed', '/carter3/rgb_right/compressed']
    output_topics = ['/camera/left/image_raw', '/camera/right/image_raw', '/camera/left/image_raw2', '/camera/right/image_raw2', '/camera/left/image_raw3', '/camera/right/image_raw3']
    processes = []
    
    for i in range(len(input_topics)):
        p = multiprocessing.Process(target=worker, args=(input_topics[i], output_topics[i]))
        processes.append(p)
        p.start()
    
    for p in processes:
        p.join()
