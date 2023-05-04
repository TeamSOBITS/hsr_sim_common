# Common ROS package for HSR in SIGVerse

HSRをSIGVerse上で動かすためのリポジトリです．

## Prerequisites

以下の環境で動作します．  
・OS: Ubuntu 20.04  
・ROS distribution: Noetic Ninjemys  

## How to use
まず，以下のコマンドを入力して，HSRを動かすための環境設定を行います．  
この設定は，初回のみに行う作業ですので，1度行ったことのある人は飛ばしてください．

```bash:
$ roscd hsr_ros
$ chmod 755 install.sh
$ sudo ./install.sh
```

以下のコマンドを端末から入力することで，SIGVerse環境と接続することができます．  

```bash:
$ roslaunch hsr_ros sigverse.launch
```

さらに，以下のコマンドを各端末から入力することで，SIGVerse環境上のHSRを擬似的に起動することができます．  
これにより，各センサ情報を扱ったり，制御を行うことができます．

```bash:
$ roslaunch hsr_ros minimal.launch
```
## Example
### 物体把持を行うプログラム（python）

物体検出をした際，object_x_cm，object_y_cm，object_z_cmにそれぞれの座標情報を入力することでその位置の物体の把持をすることができます.
※デフォルトでは各値50が入っています。
```
#!/usr/bin/env python3
# coding: utf-8
import rospy
import math
from hsr_ros.srv import gripper_moveResponse
from hsr_ros.srv import gripper_ctrl
from joint_controller import JointController

def test_grasp():
    rospy.init_node('test_grasp')
    jc = JointController()
    # 任意の場所の物体を把持する
    # 認識した物体のtfを元に位置の情報を以下に代入することで把持が可能になります.
    # 現在は各座標50に設定されており決め打ちの把持の処理を行います.
    object_x_cm = 50
    object_y_cm = 50
    object_z_cm = 50
    

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
        # base_to_finger_cm = 37 + (34.5 * math.cos(tmp_wrist_flex_joint_rad))
        arm_lift_joint_m = 0.69
        arm_flex_joint_rad = -math.radians(90) + tmp_wrist_flex_joint_rad
        arm_roll_joint_rad = 0.0
        wrist_flex_joint_rad = -tmp_wrist_flex_joint_rad
        wrist_roll_joint_rad = 0.0
        move_x_cm = object_x_cm - base_to_finger_cm
        move_y_cm = object_y_cm - 7.8

    rospy.loginfo("把持処理")
    jc.move_to_initial_pose()
    open_gripper()
    if abs(move_x_cm) > 1:
        jc.move_wheel("X:" + str(move_x_cm))
    if abs(move_y_cm) > 1:
        jc.move_wheel("Y:" + str(move_y_cm))
    jc.add_arm_control_data_to_storage('arm_lift_joint', arm_lift_joint_m)
    jc.add_arm_control_data_to_storage('arm_flex_joint', arm_flex_joint_rad)
    jc.add_arm_control_data_to_storage('arm_roll_joint', arm_roll_joint_rad)
    jc.add_arm_control_data_to_storage('wrist_flex_joint', wrist_flex_joint_rad)
    jc.add_arm_control_data_to_storage('wrist_roll_joint', wrist_roll_joint_rad)
    rospy.loginfo(jc.arm_control_data)
    jc.add_head_control_data_to_storage('head_pan_joint', 0.0)
    jc.add_head_control_data_to_storage('head_tilt_joint', -0.35)
    jc.publish_arm_control_data(2.0)
    jc.publish_head_control_data(2.0)
    rospy.sleep(2.0)
    close_gripper()
    rospy.sleep(2.0)
    jc.move_to_initial_pose()
    return gripper_moveResponse(True)


def open_gripper():
    rospy.wait_for_service("/robot_ctrl/gripper_open_and_close", 3.0)
    try:
        gripper_open_service = rospy.ServiceProxy("/robot_ctrl/gripper_open_and_close", gripper_ctrl)
        res = gripper_open_service(0.92)
        return res.is_moved
    except rospy.ServiceException as e:
        rospy.logerr("Gripper_open Service call failed: %s", e)
    return False


def close_gripper():
    rospy.wait_for_service("/robot_ctrl/gripper_open_and_close", 3.0)
    try:
        gripper_open_service = rospy.ServiceProxy("/robot_ctrl/gripper_open_and_close", gripper_ctrl)
        res = gripper_open_service(0.00)
        return res.is_moved
    except rospy.ServiceException as e:
        rospy.logerr("Gripper_close Service call failed: %s", e)
    return False



if __name__ == '__main__':
    try:
        test_grasp()
    except rospy.ROSInterruptException: pass
```

## HSRの姿勢を変える
hsr_rosパッケージのjoint_controller.py(hsr_ros/src)の関数を呼び出すことでHSRを以下のようなPose（姿勢）にすることができます.
<div align="center">
 <p>
    <img src="hsr_ros/img/initial.png" title="initial_pose" width="280">
    <img src="hsr_ros/img/detect.png" title="detecting_pose" width="280"> 
    <img src="hsr_ros/img/measure.png" title="measurement_pose" width="280"> 
 </p>
</div>
左から

### ①initial_pose
自律移動をする際に用いるpose(姿勢)
アームが移動中に衝突しないようにする姿勢です.
関数名：move_to_initial_pose（joint_controller.py）

### ②detecting_pose
物体認識の際に用いるpose(姿勢)
物体を認識する際に，カメラのフレーム内に
アームが映らないようにする姿勢です.
関数名：move_to_detecting_pose（joint_controller.py）

### ③measurement_pose
物体の高さを計測する際に用いるpose(姿勢)
この姿勢を用いることで，物体の高さを求めることができ，安全な物体の配置が可能になります.
関数名：move_to_measurement_pose（joint_controller.py）
