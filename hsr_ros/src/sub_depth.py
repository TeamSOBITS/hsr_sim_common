#!/usr/bin/env python
# coding: utf-8
import rospy
import cv2
import numpy as np
from hsr_ros.srv import object_depth, object_depthResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class DepthHSR:
    def __init__(self):
        self.servise = rospy.Service(
            'object_depth', object_depth, self.object_depth)
        topic_name = "/hsrb/head_rgbd_sensor/depth/image"
        self.depth_sub = rospy.Subscriber(topic_name, Image, self.img_cb)
        self.saved_depth = []
        self.bridge = CvBridge()

    def object_depth(self, req_msg):
        self.min_depth = -1.0
        # 窓画像の左上座標
        x, y = 180, 320
        # 窓画像の幅・高さ
        width, height = 160, 160
        # 切り抜き版
        try:
            dst = self.depth_img[y:y+height, x:x+width]
            depth_array = np.array(dst, dtype=np.float32)
            #cv2.imshow("window",self.depth_img)#デバック用
            #cv2.imshow("window_2",dst)#デバック用
            for w in range(width):
                for h in range(height):
                    self.saved_depth.append(depth_array[w][h])

            self.min_depth = min(self.saved_depth)
            print self.min_depth  # 持ってない状態：0.50200003
            #cv2.waitKey(0)#デバック用
            depth = 0.50200003 - self.min_depth
            if depth < 0:
                print '深度が0以下の場合'
                return object_depthResponse(-1.0)
            else:
                return object_depthResponse(depth)
        except (IndexError, AttributeError), e:
		    print "Object_depth Service call failed: %s" % e
        return object_depthResponse(-1.0)

    def img_cb(self, msg):
        #rospy.loginfo("Subscribed Image Topic")#デバック用
        try:
            self.depth_img = self.bridge.imgmsg_to_cv2(msg)
        except CvBridgeError, e:
            rospy.logerr(e)

        #cv2.waitKey(1)


def main():

    rospy.init_node("depth_img_subscriber")
    DepthHSR()
    rospy.spin()


if __name__ == "__main__":
    main()
