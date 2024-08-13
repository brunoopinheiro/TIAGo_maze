#! /usr/bin/python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from tmc.srv import ProcessImg, ProcessImgResponse
from sensor_msgs.msg import Image


class MyCamera:
    def __init__(self) -> None:
        print('init camera')
        self.bridge = CvBridge()
        # Subscriber to the camera image
        self.image_sub = rospy.Subscriber(
            "/xtion/rgb/image_color",
            Image,
            self.callback_SubscribeCamera,
            queue_size=1
        )
        self.width = 0
        self.height = 0

    def callback_ServiceCamera(self, request):
        threshold = 60
        r_count = 0
        g_count = 0
        for w in range(self.width):
            for h in range(self.height):
                if self.cv_image[h, w, 2] < threshold:
                    r_count += 1
                    self.cv_image[h, w, 2] = 255
                if self.cv_image[h, w, 1] < threshold:
                    g_count += 1
                    self.cv_image[h, w, 1] = 255
        # TESTING:
        print('--------------------------------------------')
        print(g_count)
        print('--------------------------------------------')
        print(r_count)
        res = ProcessImgResponse()
        if r_count > 4000 and g_count < 1000:
            res.result = 1
            return res.result
        elif g_count > 4000 and r_count < 1000:
            res.result = 2
            return res.result
        else:
            res.result = 0
            return res.result

    def callback_SubscribeCamera(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.width = int(msg.width/2)
        self.height = int(msg.height/2)
        self.cv_image = cv2.resize(self.cv_image, (self.width, self.height))


if __name__ == '__main__':
    rospy.init_node('camera_node')
    camera = MyCamera()
    camera_service = rospy.Service(
        "camera_service",
        ProcessImg,
        camera.callback_ServiceCamera,
    )
    print("Ready to process image.")
    rospy.spin()
