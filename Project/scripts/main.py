#!/usr/bin/env python

import rospy
from message_ui.msg import sent_recipe
from message_ui.msg import reply_msg
import time
from test_pick_bottle import *
from test_pick_cup import goto_cup, pick_stick
from mix import mix
from utils import *
import cv2
from cv_bridge import CvBridge
from autolab_core import RigidTransform, Point
from perception import CameraIntrinsics
from frankapy import FrankaArm

rLower = np.array([160,59,20])
rUpper = np.array([179,255,255])

gLower = np.array([70,69,20])
gUpper = np.array([83,255,255])

bLower = np.array([102,7,28])
bUpper = np.array([111,196,255])


class Test(object):
    def __init__(self):
        self.pub = rospy.Publisher("reply_msg", reply_msg, queue_size=1000)
        rospy.Subscriber('sent_recipe', sent_recipe, self.start_mixing_cb)
        self.status = reply_msg()
        rospy.on_shutdown(self.shutdownhook)

        print('Starting robot')
        self.fa = FrankaArm()    

        print('Opening Grippers')
        #Open Gripper
        self.fa.open_gripper()

        #Reset Pose
        self.fa.reset_pose() 
        #Reset Joints
        self.fa.reset_joints()

        cv_bridge = CvBridge()
        self.azure_kinect_intrinsics = CameraIntrinsics.load('calib/azure_kinect.intr')
        self.azure_kinect_to_world_transform = RigidTransform.load('calib/azure_kinect_overhead/azure_kinect_overhead_to_world.tf')    

        self.azure_kinect_rgb_image = get_azure_kinect_rgb_image(cv_bridge)
        self.azure_kinect_depth_image = get_azure_kinect_depth_image(cv_bridge)

        cv2.imwrite('rgb.png', self.azure_kinect_rgb_image)
        cv2.imwrite('depth.png', self.azure_kinect_depth_image)

        # border = define_borders(self.azure_kinect_rgb_image)
        # print(border)
        border = [[403, 191], [1657, 899]]
        mask = np.zeros(self.azure_kinect_rgb_image.shape[:2], np.uint8)
        mask[border[0][1]:border[1][1], border[0][0]:border[1][0]] = 255
        self.rgb_image = cv2.bitwise_and(self.azure_kinect_rgb_image, self.azure_kinect_rgb_image, mask=mask)
        self.cup_world = [0.5, 0, 0]
        self.stick_world = [0.35, -0.29, 0]


    def start_mixing_cb(self, msg):
        self.status.message = "Franka starts making your drink..."
        self.pub.publish(self.status)
        
        initial_pose = self.fa.get_pose().copy()

        if msg.Blue > 0:
            print("Adding Blueberry Juice")
            center = find_drink(self.rgb_image, bLower, bUpper)
            pick_up_bottle(msg.Blue, initial_pose, self.fa, center, self.azure_kinect_depth_image, self.azure_kinect_intrinsics, self.azure_kinect_to_world_transform, 0.005, self.cup_world)
            # self.fa.reset_joints()

        if msg.Green > 0:
            print("Adding Green Tea")
            center = find_drink(self.rgb_image, gLower, gUpper)
            pick_up_bottle(msg.Green, initial_pose, self.fa, center, self.azure_kinect_depth_image, self.azure_kinect_intrinsics, self.azure_kinect_to_world_transform, 0.015, self.cup_world)
            # self.fa.reset_joints()

        if msg.Red > 0:
            print("Adding Watermelon Juice")
            center = find_drink(self.rgb_image, rLower, rUpper)
            pick_up_bottle(msg.Red, initial_pose, self.fa, center, self.azure_kinect_depth_image, self.azure_kinect_intrinsics, self.azure_kinect_to_world_transform, 0.01, self.cup_world)
            # self.fa.reset_joints()

        reset(self.fa)
        pick_stick(self.fa, self.stick_world)
        goto_cup(self.fa, self.cup_world, self.stick_world)
        mix(self.fa)
        self.fa.open_gripper()

        self.status.message = "Done. Enjoy!"
        self.pub.publish(self.status)

        reset(self.fa)


    def shutdownhook(self):
        # works better than the rospy.is_shut_down()
        global ctrl_c
        self.status.message = "Oops, it's time to close. Franka looks forward to serving you next time!"
        self.pub.publish(self.status)
        ctrl_c = True


if __name__ == "__main__":
    ctrl_c = False

    Test()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutdown time! Stop the robot")