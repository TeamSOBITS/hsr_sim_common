#!/usr/bin/env python3
# coding: utf-8
import rospy
import tf
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from hsr_ros.srv import robot_motion, robot_motionResponse
from hsr_ros.srv import odom_base
from hsr_ros.srv import gripper_move, gripper_moveResponse
from hsr_ros.srv import gripper_ctrl, gripper_ctrlResponse
from hsr_ros.srv import is_grasped, is_graspedResponse


class JointController:

    def __init__(self):
        self.pub_arm_control = rospy.Publisher("/hsrb/arm_trajectory_controller/command", JointTrajectory, queue_size=10)
        self.pub_head_control = rospy.Publisher("/hsrb/head_trajectory_controller/command", JointTrajectory, queue_size=10)
        self.pub_gripper_control = rospy.Publisher("/hsrb/gripper_controller/command", JointTrajectory, queue_size=10)
        self.servise = rospy.Service("gripper_open_and_close", gripper_ctrl, self.open_and_close_gripper_server)
        self.servise = rospy.Service("gripper_move_to_target", gripper_move, self.move_gripper_to_target_server)
        self.servise = rospy.Service("motion_ctrl", robot_motion, self.move_to_registered_motion_server)
        self.servise = rospy.Service("is_grasped", is_grasped, self.is_grasped_server)
        self.listener = tf.TransformListener()
        self.arm_control_data = JointTrajectory()
        self.head_control_data = JointTrajectory()
        self.gripper_control_data = JointTrajectory()
        self.arm_control_data.points = [JointTrajectoryPoint()]
        self.head_control_data.points = [JointTrajectoryPoint()]
        self.gripper_control_data.points = [JointTrajectoryPoint()]
        self.has_getted_data = False
        self.hand_l_spring_proximal_joint_state = 0.

    def check_publishers_connection(self, publisher):
        loop_rate_to_check_connection = rospy.Rate(1)
        while (publisher.get_num_connections() == 0 and not rospy.is_shutdown()):
            try:
                loop_rate_to_check_connection.sleep()
            except rospy.ROSInterruptException:
                pass

    def add_arm_control_data_to_storage(self, joint_name, rad):
        self.arm_control_data.joint_names.append(joint_name)
        self.arm_control_data.points[0].positions.append(rad)
        self.arm_control_data.points[0].velocities.append(0)
        self.arm_control_data.points[0].accelerations.append(0)
        self.arm_control_data.points[0].effort.append(0)

    def add_head_control_data_to_storage(self, joint_name, rad):
        self.head_control_data.joint_names.append(joint_name)
        self.head_control_data.points[0].positions.append(rad)
        self.head_control_data.points[0].velocities.append(0)
        self.head_control_data.points[0].accelerations.append(0)
        self.head_control_data.points[0].effort.append(0)

    def add_gripper_control_data_to_storage(self, joint_name, rad):
        self.gripper_control_data.joint_names.append(joint_name)
        self.gripper_control_data.points[0].positions.append(rad)
        self.gripper_control_data.points[0].velocities.append(0)
        self.gripper_control_data.points[0].accelerations.append(0)
        self.gripper_control_data.points[0].effort.append(0)

    def publish_arm_control_data(self, time_from_start_sec):
        self.arm_control_data.points[0].time_from_start = rospy.Duration(time_from_start_sec)
        self.check_publishers_connection(self.pub_arm_control)
        self.pub_arm_control.publish(self.arm_control_data)
        self.arm_control_data = JointTrajectory()
        self.arm_control_data.points = [JointTrajectoryPoint()]

    def publish_head_control_data(self, time_from_start_sec):
        self.head_control_data.points[0].time_from_start = rospy.Duration(time_from_start_sec)
        self.check_publishers_connection(self.pub_head_control)
        self.pub_head_control.publish(self.head_control_data)
        self.head_control_data = JointTrajectory()
        self.head_control_data.points = [JointTrajectoryPoint()]

    def publish_gripper_control_data(self, time_from_start_sec):
        self.gripper_control_data.points[0].time_from_start = rospy.Duration(time_from_start_sec)
        self.check_publishers_connection(self.pub_head_control)
        self.pub_head_control.publish(self.gripper_control_data)
        self.gripper_control_data = JointTrajectory()
        self.gripper_control_data.points = [JointTrajectoryPoint()]

    def move_gripper_to_target_server(self, req_msg):
        """
            hand_motor_link を指定したtfに動かすサービス
            target_name : 目標のtf名
            shift       : 目標のtfの位置からどれだけずらすか
        """
        target_object = req_msg.target_name
        try:
            self.listener.waitForTransform("/head_pan_link", target_object, rospy.Time(0), rospy.Duration(2.0))
            (trans, _) = self.listener.lookupTransform("/head_pan_link", target_object, rospy.Time(0))
            self.listener.waitForTransform("/base_footprint", "head_pan_link", rospy.Time(0), rospy.Duration(2.0))
            (_, rot) = self.listener.lookupTransform("/base_footprint", "head_pan_link", rospy.Time(0))
            # print a
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("TF lookup error /base_footprint to {}".format(target_object))
            return gripper_moveResponse(False)
        except Exception as e:
            rospy.logerr("joint_controller unknown error")
            rospy.logerr(e)
            return gripper_moveResponse(False)
        object_x_cm = (trans[0] + req_msg.shift.x) * 100
        object_y_cm = (trans[1] + req_msg.shift.y) * 100
        object_z_cm = (trans[2] + req_msg.shift.z + 0.8269999) * 100
        (_, _, object_yaw_rad) = tf.transformations.euler_from_quaternion(rot)
        object_yaw_deg = math.degrees(object_yaw_rad)
        print ("ts")
        print (object_x_cm)
        print (object_y_cm)
        print (trans)
        if target_object == 'game_controller' or target_object == 'hand_palm_link':
            if 14 <= object_z_cm and object_z_cm <= 83:  # 腕を水平にして把持できる高さ範囲
                rospy.loginfo("コントローラ ver.1")
                arm_lift_joint_m = (object_z_cm - 34) * 0.01 + 0.3
                arm_flex_joint_rad = -math.radians(90)
                arm_roll_joint_rad = 0.0
                wrist_roll_joint_rad = - 3.14 / 3
                wrist_flex_joint_rad = -1.57
                move_x_cm = (object_x_cm - 71.5)  # 71.5はbase_footprintからfingerまでのx軸距離
                move_y_cm = (object_y_cm - 7.8)  # 7.8はbase_footprintからfingerまでのy軸距離

            elif object_z_cm < 14:
                rospy.loginfo("コントローラ ver.2")
                arm_lift_joint_m = 0.0
                wrist_flex_joint_rad = math.asin((34 - object_z_cm) / 34.5) - 1.57
                arm_flex_joint_rad = -(math.radians(90) + wrist_flex_joint_rad)
                arm_roll_joint_rad = 0.0
                wrist_roll_joint_rad = - 3.14 / 3
                base_to_finger_cm = 37 + (34.5 * math.cos(wrist_flex_joint_rad))
                move_x_cm = object_x_cm - base_to_finger_cm
                move_y_cm = object_y_cm - 7.8  # 7.8 は base_footprint から finger までのy軸距離

            elif object_z_cm > 83:
                rospy.loginfo("コントローラ ver.3")
                tmp_wrist_flex_joint_rad = math.asin((object_z_cm - 103) / 34.5)
                base_to_finger_cm = 37 + (34.5 * math.cos(tmp_wrist_flex_joint_rad))
                arm_lift_joint_m = 0.69
                arm_flex_joint_rad = -math.radians(90) + tmp_wrist_flex_joint_rad
                arm_roll_joint_rad = 0.0
                wrist_flex_joint_rad = -tmp_wrist_flex_joint_rad - 1.57
                wrist_roll_joint_rad = - 3.14 / 3
                move_x_cm = object_x_cm - base_to_finger_cm
                move_y_cm = object_y_cm - 7.8
        else:
            if 34 <= object_z_cm and object_z_cm <= 103:  # 腕を水平にして把持できる高さ範囲
                rospy.loginfo("腕を水平にして把持できる高さ範囲")
                arm_lift_joint_m = (object_z_cm - 34) * 0.01
                arm_flex_joint_rad = -math.radians(90)
                arm_roll_joint_rad = 0.0
                wrist_roll_joint_rad = 0.0
                wrist_flex_joint_rad = 0.0
                move_x_cm = (object_x_cm - 71.5)  # 71.5はbase_footprintからfingerまでのx軸距離
                move_y_cm = (object_y_cm - 7.8)  # 7.8はbase_footprintからfingerまでのy軸距離

            elif object_z_cm < 34:
                rospy.loginfo("object_z_cm < 34")
                arm_lift_joint_m = 0.0
                wrist_flex_joint_rad = math.asin((34 - object_z_cm) / 34.5)
                arm_flex_joint_rad = -(math.radians(90) + wrist_flex_joint_rad)
                arm_roll_joint_rad = 0.0
                wrist_roll_joint_rad = 0.0
                base_to_finger_cm = 37 + (34.5 * math.cos(wrist_flex_joint_rad))
                move_x_cm = object_x_cm - base_to_finger_cm
                move_y_cm = object_y_cm - 7.8  # 7.8 は base_footprint から finger までのy軸距離

            elif object_z_cm > 103:
                rospy.loginfo("object_z_cm > 103")
                tmp_wrist_flex_joint_rad = math.asin((object_z_cm - 103) / 34.5)
                base_to_finger_cm = 37 + (34.5 * math.cos(tmp_wrist_flex_joint_rad))
                arm_lift_joint_m = 0.69
                arm_flex_joint_rad = -math.radians(90) + tmp_wrist_flex_joint_rad
                arm_roll_joint_rad = 0.0
                wrist_flex_joint_rad = -tmp_wrist_flex_joint_rad
                wrist_roll_joint_rad = 0.0
                move_x_cm = object_x_cm - base_to_finger_cm
                move_y_cm = object_y_cm - 7.8

        if abs(object_yaw_deg) > 1:
            self.move_wheel("T:" + str(object_yaw_deg))
        if abs(move_x_cm) > 1:
            self.move_wheel("X:" + str(move_x_cm))
        if abs(move_y_cm) > 1:
            self.move_wheel("Y:" + str(move_y_cm))
        print (move_x_cm)
        print (move_y_cm)
        self.add_arm_control_data_to_storage('arm_lift_joint', arm_lift_joint_m)
        self.add_arm_control_data_to_storage('arm_flex_joint', arm_flex_joint_rad)
        self.add_arm_control_data_to_storage('arm_roll_joint', arm_roll_joint_rad)
        self.add_arm_control_data_to_storage('wrist_flex_joint', wrist_flex_joint_rad)
        self.add_arm_control_data_to_storage('wrist_roll_joint', wrist_roll_joint_rad)
        rospy.loginfo(self.arm_control_data)
        self.add_head_control_data_to_storage('head_pan_joint', 0.0)
        self.add_head_control_data_to_storage('head_tilt_joint', -0.35)
        self.publish_arm_control_data(2.0)
        self.publish_head_control_data(2.0)
        rospy.sleep(2.0)
        return gripper_moveResponse(True)

    def open_and_close_gripper_server(self, req_msg):
        hand_motor_joint_rad = req_msg.rad
        time_from_start_sec = 1
        self.add_gripper_control_data_to_storage("hand_motor_joint", hand_motor_joint_rad)
        self.publish_gripper_control_data(time_from_start_sec)
        rospy.sleep(time_from_start_sec)
        return gripper_ctrlResponse(True)

    def joint_satae_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == "hand_l_spring_proximal_joint":
                self.hand_l_spring_proximal_joint_state = msg.position[i]
                self.has_getted_data = True

    def is_grasped_server(self, req_msg):
        rospy.Subscriber("/hsrb/joint_states", JointState, self.joint_satae_callback)
        now = rospy.Time.now()
        self.has_getted_data = False
        while rospy.Time.now() < now + rospy.Duration(5.0):
            if self.has_getted_data:
                if not self.hand_l_spring_proximal_joint_state == 0:
                    return is_graspedResponse(True)
                else:
                    return is_graspedResponse(False)
            else:
                rospy.sleep(0.5)
        return is_graspedResponse(False)

    def move_to_registered_motion_server(self, req_msg):
        motion_type = req_msg.motion_type
        if motion_type == "INITIAL_POSE":
            self.move_to_initial_pose()
        elif motion_type == "DETECTING_POSE":
            self.move_to_detecting_pose()
        elif motion_type == "DETECTING_POSE2":
            self.move_to_detecting_pose2()
        elif motion_type == "DETECTING_BOX_POSE":
            self.move_to_detecting_box_pose()
        elif motion_type == "MEASUREMENT_POSE":
            self.move_to_measurement_pose()
        elif motion_type == "PERSON_DETECT_POSE":
            self.move_to_person_detect_pose()
        elif motion_type == "GAMING_POSE":
            self.move_to_game_detect_pose()
        elif motion_type == "CUSTOM_POSE":
            self.move_to_target_pose()

        return robot_motionResponse(True)

    def move_wheel(self, str_distance):
        rospy.wait_for_service('/robot_ctrl/base_ctrl')
        try:
            wheel_ctrl_service = rospy.ServiceProxy('/robot_ctrl/base_ctrl', odom_base)
            res = wheel_ctrl_service(str_distance)
            return res.res_str
        except rospy.ServiceException as e:
            print ("Service call failed: %s" % e)

    def move_to_initial_pose(self):
        time_from_start_sec = 1.0
        self.add_head_control_data_to_storage('head_pan_joint', 0.0)
        self.add_head_control_data_to_storage('head_tilt_joint', 0.0)
        self.add_arm_control_data_to_storage('arm_lift_joint', 0.0)
        self.add_arm_control_data_to_storage('arm_flex_joint', 0.0)
        self.add_arm_control_data_to_storage('arm_roll_joint', -1.5708)
        self.add_arm_control_data_to_storage('wrist_flex_joint', -1.5708)
        self.add_arm_control_data_to_storage('wrist_roll_joint', 0)
        self.publish_arm_control_data(time_from_start_sec)
        rospy.sleep(1.0)
        self.publish_head_control_data(time_from_start_sec)
        rospy.sleep(1.0)

    def move_to_detecting_pose(self):
        time_from_start_sec = 1.0
        self.add_head_control_data_to_storage('head_pan_joint', -0.52)
        self.add_head_control_data_to_storage('head_tilt_joint', -0.35)
        self.add_arm_control_data_to_storage('arm_lift_joint', 0.15)
        self.add_arm_control_data_to_storage('arm_flex_joint', 0.0)
        self.add_arm_control_data_to_storage('arm_roll_joint', 1.5708)
        self.add_arm_control_data_to_storage('wrist_flex_joint', -1.35)
        self.add_arm_control_data_to_storage('wrist_roll_joint', 0.0)
        self.publish_arm_control_data(time_from_start_sec)
        rospy.sleep(1.0)
        self.publish_head_control_data(time_from_start_sec)
        self.move_wheel("T:29")
        rospy.sleep(1.0)

    def move_to_detecting_pose2(self):
        time_from_start_sec = 1.0
        self.add_head_control_data_to_storage('head_pan_joint', -0.52)
        self.add_head_control_data_to_storage('head_tilt_joint', -0.1)
        self.add_arm_control_data_to_storage('arm_lift_joint', 0.16)
        self.add_arm_control_data_to_storage('arm_flex_joint', 0.0)
        self.add_arm_control_data_to_storage('arm_roll_joint', 1.5708)
        self.add_arm_control_data_to_storage('wrist_flex_joint', -1.35)
        self.add_arm_control_data_to_storage('wrist_roll_joint', 0.0)
        self.publish_arm_control_data(time_from_start_sec)
        rospy.sleep(1.0)
        self.publish_head_control_data(time_from_start_sec)
        self.move_wheel("T:29")
        rospy.sleep(1.0)

    def move_to_detecting_box_pose(self):
        time_from_start_sec = 1.0
        self.add_head_control_data_to_storage('head_pan_joint', 0.0) #-0.52
        self.add_head_control_data_to_storage('head_tilt_joint', -0.55)
        self.add_arm_control_data_to_storage('arm_lift_joint', 0.15)
        self.add_arm_control_data_to_storage('arm_flex_joint', 0.0)
        self.add_arm_control_data_to_storage('arm_roll_joint', 1.5708)
        self.add_arm_control_data_to_storage('wrist_flex_joint', -1.35)
        self.add_arm_control_data_to_storage('wrist_roll_joint', 0.0)
        self.publish_arm_control_data(time_from_start_sec)
        rospy.sleep(1.0)
        self.publish_head_control_data(time_from_start_sec)
        rospy.sleep(1.0)

    def move_to_measurement_pose(self):
        time_from_start_sec = 1.0
        self.add_head_control_data_to_storage('head_pan_joint', 0.0)
        self.add_head_control_data_to_storage('head_tilt_joint', -0.2)#-0.2
        self.add_arm_control_data_to_storage('arm_lift_joint', 0.0)
        self.add_arm_control_data_to_storage('arm_flex_joint', -1.0)
        self.add_arm_control_data_to_storage('arm_roll_joint', 3.14)
        self.add_arm_control_data_to_storage('wrist_flex_joint', -0.8)
        self.add_arm_control_data_to_storage('wrist_roll_joint', 0)
        self.publish_arm_control_data(time_from_start_sec)
        rospy.sleep(1.0)
        self.publish_head_control_data(time_from_start_sec)
        rospy.sleep(1.0)

    def move_to_person_detect_pose(self):
        time_from_start_sec = 1.0
        self.add_head_control_data_to_storage('head_pan_joint', 0.0)
        self.add_head_control_data_to_storage('head_tilt_joint', 0.0)
        self.add_arm_control_data_to_storage('arm_lift_joint', 0.0)
        self.add_arm_control_data_to_storage('arm_flex_joint', 0.0)
        self.add_arm_control_data_to_storage('arm_roll_joint',  -1.5708)
        self.add_arm_control_data_to_storage('wrist_flex_joint',  -1.5708)
        self.add_arm_control_data_to_storage('wrist_roll_joint', 0.0)
        self.publish_arm_control_data(time_from_start_sec)
        rospy.sleep(1.0)
        self.publish_head_control_data(time_from_start_sec)
        self.move_wheel("T:30")
        rospy.sleep(1.0)

    #コントローラをつかむためだけの設定
    def move_to_game_detect_pose(self):
        time_from_start_sec = 1.0
        self.add_head_control_data_to_storage('head_pan_joint', 0.0)
        self.add_head_control_data_to_storage('head_tilt_joint', 0.0)
        self.add_arm_control_data_to_storage('arm_lift_joint', 0.69)
        self.add_arm_control_data_to_storage('arm_flex_joint', -1.5708)
        self.add_arm_control_data_to_storage('arm_roll_joint',  0.0)
        self.add_arm_control_data_to_storage('wrist_flex_joint',  -1.5708)
        self.add_arm_control_data_to_storage('wrist_roll_joint', -1.5708)
        self.publish_arm_control_data(time_from_start_sec)
        rospy.sleep(1.0)
        self.publish_head_control_data(time_from_start_sec)
        rospy.sleep(1.0)
    
    #各関節の引数を手動で設定してポーズ変更できる関数
    def move_to_target_pose(self, pose={'head_pan_joint' : 0.0, 'head_tilt_joint' : 0.0, 'arm_lift_joint' : 0.0, 'arm_flex_joint' : 0.0, 'arm_roll_joint' :  -1.5708, 'wrist_flex_joint' :  -1.5708, 'wrist_roll_joint'  : 0.0}):
        time_from_start_sec = 1.0
        self.add_head_control_data_to_storage('head_pan_joint', pose['head_pan_joint'])
        self.add_head_control_data_to_storage('head_tilt_joint', pose['head_tilt_joint'])
        self.add_arm_control_data_to_storage('arm_lift_joint', pose['arm_lift_joint'])
        self.add_arm_control_data_to_storage('arm_flex_joint', pose['arm_flex_joint'])
        self.add_arm_control_data_to_storage('arm_roll_joint', pose['arm_roll_joint']),
        self.add_arm_control_data_to_storage('wrist_flex_joint', pose['wrist_flex_joint'])
        self.add_arm_control_data_to_storage('wrist_roll_joint', pose['wrist_roll_joint'])
        self.publish_arm_control_data(time_from_start_sec)
        rospy.sleep(1.0)
        self.publish_head_control_data(time_from_start_sec)
        rospy.sleep(1.0)

if __name__ == "__main__":
    rospy.init_node("joint_controller")
    jc = JointController()
    # jc.move_to_initial_pose()
    rospy.spin()
