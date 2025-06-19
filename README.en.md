<a name="readme-top"></a>

[JP](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# hsr_sim_common

<!-- 目次 -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#introduction">Introduction</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li>
    　<a href="#launch-and-usage">Launch and Usage</a>
    </li>
    <li>
    　<a href="#software">Software</a>
      <ul>
        <li><a href="#point-cloud">Point Cloud</a></li>
        <li><a href="#library-server">Library Server</a></li>
        <li><a href="#changing-poses">Changing Poses</a></li>
      </ul>
    </li>
    <li><a href="#milestones">Milestones</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
     <!-- <li><a href="#参考文献">参考文献</a></li> -->
  </ol>
</details>



<!-- レポジトリの概要 -->
## Introduction

This package contains the necessary components for operating the HSR (SIGVerse). It handles the installation of robot meshes and descriptions, and defines functions for object grasping and poses.

<!-- セットアップ -->
## Getting Started

This section describes how to set up this repository.

<p align="right">(<a href="#readme-top">Back to the Top</a>)</p>


### Prerequisites

First, ensure the following environment is configured before proceeding with the installation steps:

| System  | Version |
| ------------- | ------------- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS | Humble Hawksbill |
| Python | 3.0~ |

<p align="right">(<a href="#readme-top">Back to the Top</a>)</p>


### Installation

1. Clone the hsr_sim_common package using the following commands
   ```sh
   cd ~/colcon_ws/src/
   ```
   ```sh
   git clone -b humble-devel https://github.com/TeamSOBITS/hsr_sim_common.git
   ```
2. Install the dependencies using install.sh. This primarily includes:
   - Mongo C driver
     - Necessary for controlling the HSR in the SIGVerse environment using Mongo.
     - Mongo is a popular NoSQL document-oriented database.
   - Mongo C++ driver
     - Installs the C++ driver similarly.
   - sigverse_ros_bridge setup
     - Installed to bridge SIGVerse on Windows and ROS 2 on Ubuntu.
   ```sh
   cd hsr_sim_common
   ```
   ```sh
   bash install.sh
   ```
3. Then, build the colcon workspace
   ```sh
   cd ~/colcon_ws/
   ```
   ```sh
   colcon build
   ```
<p align="right">(<a href="#readme-top">Back to the Top</a>)</p>


<!-- 実行・操作方法 -->
## Launch and Usage
1. After connecting to SIGVerse, execute [`minimal.launch.py`](launch/minimal.launch.py).
   ```sh
    ros2 launch hsr_sim_common minimal.launch.py
   ```
2. You can control the robot by launching [teleop_key.launch.py](launch/teleop_key.launch.py)．
   ```sh
   ros2 launch hsr_sim_common teleop_key.launch.py
   ```
  <details>
    <summary>Keyboard Control Method</summary>
    
- Moving around
    
  | | | |
  |---|---|---|
  | u | i | o |
  | j | k | l |
  | m | , | . |
  
- For Holonomic mode (strafing)
  - Hold down the shift key
  
  | | | |
  |---|---|---|
  | U | I | O |
  | J | K | L |
  | M | < | > |
  
- Simple Teleoperation with arrow keys
  
  | | | |
  |---|---| --- |
  | | ↑ | |
  | ← |  | → |
  | | ↓ | |
  
- Speed Adjustment and Joint Operations
  - q / z: Increase / decrease max speeds by 10%
  - w / x: Increase / decrease linear speed only by 10%
  - e / c: Increase / decrease angular speed only by 10%
  - a + arrow keys: Control arm_lift_joint
  - s + arrow keys: Control arm_flex_joint and arm_roll_joint
  - d + arrow keys: Control wrist_flex_joint and wrist_roll_joint
  - f + arrow keys: Control head_pan_joint and head_tilt_joint
  - y + arrow keys: Control linear_x, linear_y, and angular_z
  - g: Toggle hand open/close
  - h: Move to initial pose

  </details>

<p align="right">(<a href="#readme-top">Back to the Top</a>)</p>


## Software

### Point Cloud
- You can publish point clouds by launching [generate_pointcloud.launch.py](launch/generate_pointcloud.launch.py).
- This file is automatically executed when [minimal.launch.py](launch/minimal.launch.py) is run.

### Library Server
- Launching [library_server.launch.py](launch/library_server.launch.py) starts an Action Server for exchanging information related to changing poses, horizontal movement, rotation, altering individual joint angles, and moving the hand to a specified TF.
- This file is also automatically executed when [minimal.launch.py](launch/minimal.launch.py) is run.

### Changing Poses
- You can modify the available poses by editing [pose_list.yaml](config/pose_list.yaml).

<div align="center">
 <p>
    <img src="img/initial.png" title="initial_pose" width="280">
    <img src="img/detect.png" title="detecting_pose" width="280"> 
    <img src="img/measure.png" title="measurement_pose" width="280"> 
 </p>
</div>

From left to right:

#### ①initial_pose  
Purpose: Used for autonomous navigation.
Description: A posture that prevents the arm from colliding during movement.

#### ②detecting_pose  
Purpose: Used for object recognition.
Description: A posture that ensures the arm does not appear in the camera's frame during object recognition.

#### ③measurement_pose  
Purpose: Used for measuring object height.
Description: This posture allows the robot to determine an object's height, enabling safe object placement.


<p align="right">(<a href="#readme-top">Back to the Top</a>)</p>


<!-- マイルストーン -->
## Milestones

Please check the [Issue page][issues-url] to view current bugs and requests for new features.

<p align="right">(<a href="#readme-top">Back to the Top</a>)</p>


<!-- CONTRIBUTING -->
<!-- ## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">上に戻る</a>)</p> -->


<!-- LICENSE -->
<!-- ## License

Distributed under the MIT License. See `LICENSE.txt` for more NOTErmation.

<p align="right">(<a href="#readme-top">上に戻る</a>)</p> -->


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/hsr_sim_common.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/hsr_sim_common/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/hsr_sim_common.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/hsr_sim_common/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/hsr_sim_common.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/hsr_sim_common/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/hsr_sim_common.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/hsr_sim_common/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/hsr_sim_common.svg?style=for-the-badge
[license-url]: LICENSE

