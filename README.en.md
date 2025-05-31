<a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

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
      <a href="#Overview">Overview</a>
    </li>
    <li>
      <a href="#Setup">Setup</a>
      <ul>
        <li><a href="#Environment Requirements">Environment Requirements</a></li>
        <li><a href="#Installation">Installation</a></li>
      </ul>
    </li>
    <li>
    　<a href="#Execution and Operation">Execution and Operation</a>
    </li>
    <li>
    　<a href="#Software">Software</a>
      <ul>
        <li><a href="#Point Cloud">Point Cloud</a></li>
        <li><a href="#Library Server">Library Server</a></li>
        <li><a href="#Changing Poses">Changing Poses</a></li>
      </ul>
    </li>
    <li><a href="#Milestones">Milestones</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
     <!-- <li><a href="#参考文献">参考文献</a></li> -->
  </ol>
</details>



<!-- レポジトリの概要 -->
## Overview

This package contains the necessary components for operating the HSR (SIGVerse). It handles the installation of robot meshes and descriptions, and defines functions for object grasping and poses.

<!-- セットアップ -->
## Setup

This section describes how to set up this repository.

<p align="right">(<a href="#readme-top">Back to the Top</a>)</p>


### Environment Requirements

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
## Execution and Operation
After connecting to SIGVerse, execute [minimal.launch.py](launch/minimal.launch.py).
   ```sh
    ros2 launch hsr_sim_common minimal.launch.py
   ```

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

