# Avitra Mobile Manipulator

<!-- TABLE OF CONTENTS -->

## Table of Contents

- [About the Project](#about-the-project)
  - [Built With](#built-with)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
- [Usage](#usage)
- [License](#license)
- [Contact](#contact)
- [Acknowledgements](#acknowledgements)

<!-- ABOUT THE PROJECT -->

## About The Project

The ultimate goal of Autonomous Mobile Manipulator is the execution of complex manipulation tasks in unstructured and dynamic environments, in which cooperation with humans may be required. The goal of this project is to develop a mobile manipulator that can traverse a given mapped area and then perform manipulation on a given object using a 5 DOF manipulator.

Mobile manipulation systems must perform a variety of tasks, acquire new skills, and apply these skills towards the tasks. Thus, requiring Simultaneous Localization and Mapping(SLAM) of the environment to locomote and successfully complete the tasks.

Every planning problem in robotics involves constraints. Some constraints are straightforward to satisfy while others can be so stringent that feasible states are very difcult to nd. What makes planning with constraints challenging is that, for many constraints, it is impossible or impractical to provide the planning algorithm with the allowed states explicitly; it must discover these states as it plans. Mobile manipulation systems require the integration of a large number of hardware components for sensing, manipulation, and locomotion as well as the orchestration of algorithmic capabilities in perception, manipulation, learning, control, planning, etc.

### Built With

- [ROS Kinetic](http://wiki.ros.org/)
- [MoveIt Motion Planning Framework](https://moveit.ros.org/)

### Prerequisites

- Python 2.7.x

```sh
sudo apt-get install python2
```

- ROS

```sh
sudo apt-get install ros-kinetic
```

<!-- USAGE EXAMPLES -->

## Usage

- [Motion Planning Pipeline](https://github.com/atharvkhadtare/fyp_ws/tree/manipulator_with_gripper)
- [Perception Pipeline](https://github.com/atharvkhadtare/fyp_ws/tree/3d_perception)
- [Velocity Control of Motor](https://github.com/rshah4356/velocity_control_of_motor)

NOTE: While compiling IKFast delete the devel/, logs/ and build/ folders and stop all ros processes

<!-- CONTACT -->

## Contact

- Atharva Bhave - atharvabhave21@gmail.com
- Atharv Khadtare - atharva860@gmail.com
- Chinmay Khopade - chinmaykhopde@gmail.com
- Shubham Patil - patilshubham1002@gmail.com
- Suyash Junnarkar -smjunn22@gmail.com
- Rishabh Shah - rshah4356@gmail.com
- Viraj Sonawane - majorviraj@gmail.com

<!-- Acknowledgements -->

## Acknowledgements

- [Avitra : 2017-2018](https://github.com/sachin0x18/Imitation)

<!-- LICENSE -->

## License

Distributed under the MIT License. See `LICENSE` for more information.
