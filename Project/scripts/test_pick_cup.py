from frankapy import FrankaArm
import numpy as np
import argparse
import cv2
from cv_bridge import CvBridge
from autolab_core import RigidTransform, Point
from perception import CameraIntrinsics
from utils import *
from RobotUtil import *

AZURE_KINECT_INTRINSICS = 'calib/azure_kinect.intr'
AZURE_KINECT_EXTRINSICS = 'calib/azure_kinect_overhead/azure_kinect_overhead_to_world.tf'

cup_world = [0.55, 0, 0]  # TODO: record a fixed position


def define_borders(img):
    border = []
    
    def drawBorder(action, x, y, flags, param):
        if action == cv2.EVENT_LBUTTONDOWN:
           param.append([x,y])
        elif action == cv2.EVENT_LBUTTONUP:
            param.append([x,y])
        
    cv2.namedWindow('image')
    cv2.imshow('image', img)
    cv2.setMouseCallback('image', drawBorder, border)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return border


def find_cup(rgb_image):
    gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.3, 50, param1=80, param2=140, minRadius=10, maxRadius=150)
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
        
            cv2.circle(rgb_image, (x, y), r, (0, 255, 0), 2)
            print(x,y,r)
    # cv2.imshow('circles', rgb_image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    return sorted(circles, key=lambda x: x[1])[0]


def pick_n_place_cup(fa, cup_world):
    object_z_height = 0.08

    # cup_edge_in_world = get_object_center_point_in_world(circle[0],
    #                                                     circle[1],
    #                                                     azure_kinect_depth_image, azure_kinect_intrinsics,
    #                                                     azure_kinect_to_world_transform)
    cup_edge_in_world = [0.3, -0.3, 0]

    object_center_pose = fa.get_pose()
    object_center_pose.translation = [cup_edge_in_world[0], cup_edge_in_world[1], object_z_height]

    intermediate_robot_pose = object_center_pose.copy()
    intermediate_robot_pose.translation[2] += 0.05
    fa.goto_pose(intermediate_robot_pose, 5)

    #Cup
    fa.goto_pose(object_center_pose, 5, force_thresholds=[10, 10, 10, 10, 10, 10])

    #Close Gripper
    fa.goto_gripper(0.001, grasp=True, force=60.0)

    #Lift
    lift_pose = fa.get_pose()
    lift_pose.translation[2] += 0.2
    fa.goto_pose(lift_pose)

    #Put to center in the world
    cup_center_world_pose = object_center_pose.copy()
    cup_center_world_pose.translation[:2] = cup_world[:2]
    # cup_center_world_pose.translation[2] += 0.01  # to counter the effect of incorrect IK without use_impedance
    fa.goto_pose(cup_center_world_pose)

    #Open Gripper
    fa.open_gripper()

    return cup_center_world_pose


def mix_drink_n_serve(fa, cup_center_world_pose):
    #Grasp Cup
    cup_pose = fa.get_pose()
    cup_pose.translation = cup_center_world_pose.translation
    fa.goto_pose(cup_pose)
    fa.goto_gripper(0.001, grasp=True, force=60.0)

    #Lift
    lift_pose = fa.get_pose()
    lift_pose.translation[2] += 0.2
    fa.goto_pose(lift_pose)
    
    #Mix
    cur_joints = fa.get_joints()
    angle = 25
    for rot_delta in [-angle, angle, -angle, angle, 0]:
        cur_joints_rotated = cur_joints.copy()
        cur_joints_rotated[6] += np.deg2rad(rot_delta)
        fa.goto_joints(cur_joints_rotated, 0.5)

    #Serve
    serve_pose = cup_pose
    serve_pose.translation[0] = 0.6
    fa.goto_pose(serve_pose)
    
    #Open Gripper
    fa.open_gripper()

    #Reset Pose
    fa.reset_pose() 
    #Reset Joints
    fa.reset_joints()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--intrinsics_file_path', type=str, default=AZURE_KINECT_INTRINSICS)
    parser.add_argument('--extrinsics_file_path', type=str, default=AZURE_KINECT_EXTRINSICS) 
    args = parser.parse_args()
    
    print('Starting robot')
    fa = FrankaArm()    

    print('Opening Grippers')
    #Open Gripper
    fa.open_gripper()

    #Reset Pose
    fa.reset_pose() 
    #Reset Joints
    fa.reset_joints()

    cv_bridge = CvBridge()
    azure_kinect_intrinsics = CameraIntrinsics.load(args.intrinsics_file_path)
    azure_kinect_to_world_transform = RigidTransform.load(args.extrinsics_file_path)    

    azure_kinect_rgb_image = get_azure_kinect_rgb_image(cv_bridge)
    azure_kinect_depth_image = get_azure_kinect_depth_image(cv_bridge)

    cv2.imwrite('rgb.png', azure_kinect_rgb_image)
    cv2.imwrite('depth.png', azure_kinect_depth_image)

    # border = define_borders(azure_kinect_rgb_image)
    # print(border)
    border = [[400, 178], [1641, 881]]
    mask = np.zeros(azure_kinect_rgb_image.shape[:2], np.uint8)
    mask[border[0][1]:border[1][1], border[0][0]:(border[1][0] + border[0][0]) // 2] = 255
    rgb_image = cv2.bitwise_and(azure_kinect_rgb_image, azure_kinect_rgb_image, mask=mask)

    # circle = find_cup(rgb_image)

    object_z_height = 0.08

    # cup_edge_in_world = get_object_center_point_in_world(circle[0],
    #                                                     circle[1],
    #                                                     azure_kinect_depth_image, azure_kinect_intrinsics,
    #                                                     azure_kinect_to_world_transform)
    cup_edge_in_world = [0.3, -0.3, 0]

    object_center_pose = fa.get_pose()
    object_center_pose.translation = [cup_edge_in_world[0], cup_edge_in_world[1], object_z_height]

    intermediate_robot_pose = object_center_pose.copy()
    intermediate_robot_pose.translation[2] += 0.05
    fa.goto_pose(intermediate_robot_pose, 5, use_impedance=False)

    #Cup
    fa.goto_pose(object_center_pose, 5, force_thresholds=[10, 10, 10, 10, 10, 10])

    #Close Gripper
    fa.goto_gripper(0.001, grasp=True, force=60.0)

    #Lift
    lift_pose = fa.get_pose()
    lift_pose.translation[2] += 0.2
    fa.goto_pose(lift_pose)

    #Put to center in the world
    cup_center_world_pose = object_center_pose.copy()
    cup_center_world_pose.translation[:2] = cup_world[:2]
    cup_center_world_pose.translation[2] += 0.01  # to counter the effect of incorrect IK without use_impedance
    fa.goto_pose(cup_center_world_pose, use_impedance=False)

    #Open Gripper
    fa.open_gripper()
    
    '''
    PLACEHOLDER: pouring drinks now
    '''
    fa.reset_pose() 
    fa.reset_joints()
    '''
    END OF PLACEHOLDER
    '''

    #Grasp Cup
    cup_pose = fa.get_pose()
    cup_pose.translation = cup_center_world_pose.translation
    fa.goto_pose(cup_pose)
    fa.goto_gripper(0.001, grasp=True, force=60.0)

    #Lift
    lift_pose = fa.get_pose()
    lift_pose.translation[2] += 0.2
    fa.goto_pose(lift_pose)
    
    #Mix
    '''
        -
      -   -
    -       -
      -   -
        -
    '''
    # cur_pose = fa.get_pose()
    # x, y = cur_pose.translation[:2]
    # radius = 0.05
    # circle_movement_x_y = [ [x + radius / 2, y - radius / 2],
    #                         [x + radius, y - radius],
    #                         [x + radius * 1.5, y - radius / 2],
    #                         [x + 2 * radius, y],
    #                         [x + radius * 1.5, y + radius / 2],
    #                         [x + radius, y + radius],
    #                         [x + radius / 2, y + radius / 2],
    #                         [x, y]]
    # for pts in circle_movement_x_y:
    #     cur_pose.translation[:2] = pts[:2]
    #     fa.goto_pose(cur_pose, 0.1)

    cur_joints = fa.get_joints()
    angle = 25
    for rot_delta in [-angle, angle, -angle, angle, 0]:
        cur_joints_rotated = cur_joints.copy()
        cur_joints_rotated[6] += np.deg2rad(rot_delta)
        fa.goto_joints(cur_joints_rotated, 0.5)

    #Serve
    serve_pose = cup_pose
    serve_pose.translation[0] += 0.15
    fa.goto_pose(serve_pose)
    
    #Open Gripper
    fa.open_gripper()

    #Reset Pose
    fa.reset_pose() 
    #Reset Joints
    fa.reset_joints()