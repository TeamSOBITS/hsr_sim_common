<a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# HSR_sim_common

<!-- 目次 -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#Introduction">Introduction</a>
    </li>
    <li>
      <a href="#Set Up">Set Up</a>
      <ul>
        <li><a href="#prerequisites">prerequisites</a></li>
        <li><a href="#installation">installation</a></li>
      </ul>
    </li>
    <li>
    　<a href="#launch-and-usage">launch-and-usage</a>
      <ul>
        <li><a href="#Launch">Launch</a></li>
      </ul>
    </li>
    <li>
    　<a href="#SoftWere">SoftWere</a>
      <ul>
        <li><a href="#Grasp">Grasp</a></li>
        <li><a href="#Change Pose">Change Pose</a></li>
      </ul>
    </li>
    <li><a href="#milestone">milestone</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#References">References</a></li>
  </ol>
</details>



<!-- レポジトリの概要 -->
## Introduction

This is the package required to run HSR (SIGVerse).
The mesh and description of the robot are installed here. Object grasping and posing functions are also specified in this package.


<!-- セットアップ -->
## Set Up

This section describes how to set up this repository.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Prerequisites

First, please set up the following environment before proceeding to the next installation stage.

| System  | Version |
| ------------- | ------------- |
| Ubuntu | 20.04 (Focal Fossa) |
| ROS | Noetic Ninjemys |
| Python | 3.8 |

> [!NOTE]
> If you need to install `Ubuntu` or `ROS`, please check our [SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6).


<p align="right">(<a href="#readme-top">Back to top</a>)</p>


### Installation

1. Go to the `src` folder of ROS.
   ```sh
   $ roscd
   # Or just use "cd ~/catkin_ws/" and change directory.
   $ cd src/
   ```
2. Clone this repository.
   ```sh
   $ git clone https://github.com/TeamSOBITS/hsr_sim_common
   ```
3. Navigate into the repository.
   ```sh
   $ cd hsr_sim_common/
   ```
4. Install the dependent packages.
   ```sh
   $ bash install.sh
   ```

    ```bash:
    $ roscd hsr_ros
    $ chmod 755 install.sh
    $ sudo ./install.sh
    ```

5. Compile the package.
   ```sh
   $ roscd
   # Or just use "cd ~/catkin_ws/" and change directory.
   $ catkin_make
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- 実行・操作方法 -->
## Launch and Usage

1. Set the parameters inside[minimal.launch](hsr_sim_common/launch/minimal.launch)and select the functions to launch with HSR
   ```xml
    roslaunch hsr_sim_common minimal.launch
    ...
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Software

### Grasp
Please refer to `grasp.py` located in the `example` folder.

#### Function Descriptions

1. `grasp_to_target_coord`
   - A function that allows HSR to grasp an object located at the specified 3D coordinates.

2. `open_gripper`
   - A function to open the gripper.

3. `close_gripper`
   - A function to close the gripper.

*Additional examples will be added as needed.*

### Changing Pose
By calling the functions in `joint_controller.py` (`hsr_ros/src`) of the `hsr_ros` package, you can change the HSR to the following poses.

<div align="center">
 <p>
    <img src="hsr_ros/img/initial.png" title="initial_pose" width="280">
    <img src="hsr_ros/img/detect.png" title="detecting_pose" width="280"> 
    <img src="hsr_ros/img/measure.png" title="measurement_pose" width="280"> 
 </p>
</div>

#### ①initial_pose
   - Purpose: Used when performing autonomous movement.
   - Description: A pose that ensures the arm does not collide during movement.
   - Function Name: `move_to_initial_pose` (`joint_controller.py`)

#### ②detecting_pose
   - Purpose: Used when performing object recognition.
   - Description: A pose that ensures the arm does not appear within the camera frame during object recognition.
   - Function Name: `move_to_detecting_pose` (`joint_controller.py`)

#### ③measurement_pose
   - Purpose: Used when measuring the height of an object.
   - Description: This pose allows for the measurement of object height, enabling safe object placement.
   - Function Name: `move_to_measurement_pose` (`joint_controller.py`)

<p align="right">(<a href="#readme-top">Back to top</a>)</p>

<!-- Milestones -->
## Milestones

- [x] Modify example files
- [x] OSS
    - [x] Enhance documentation
    - [x] Unify coding style

To check the current bugs and new feature requests, please refer to the [Issue page][issues-url].

<p align="right">(<a href="#readme-top">Back to top</a>)</p>


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

