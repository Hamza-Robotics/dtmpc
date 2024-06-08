<div align="center">
<img src="utils/LOGO.png" alt="Header Image" width="300"/>
</div>

# Decentralized Tube-Based Model Predictive Controller.

![ROS 2](https://img.shields.io/badge/ROS-2-blue.svg)![Python 3](https://img.shields.io/badge/python-3-blue.svg)![GitHub stars](https://img.shields.io/github/stars/hamza-robotics/dtmpc.svg?style=social)

## Table of Contents

- [Introduction](#introduction)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Introduction
This is the implementation of the Master thesis titled : Tube-Based NMPC for non-holonomic multi-agent system in unknown enviorments. 

## Article 

Comming soon. 

## Installation
```bash
git clone https://github.com/Hamza-Robotics/dtmpc
```
- Install acados. [Here](src/mpc/README.md) there is instruction on how to install Acados.




Remember to source ROS2 in your workspace. 
```bash
source /opt/ros/humble/setup.bash
```

```bash
sudo apt install python3-pip
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

```bash
source setup_acados.sh 
colcon build && source install/setup.bash
ros2 launch simulation_numerical numerical_simulation_launch.py 
```


```bash
colcon build && source install/setup.bash && 
source setup_acados.sh  && ros2 run mpc mpc_controller_robot1.py
```

```bash
colcon build && source install/setup.bash && 
source setup_acados.sh  && ros2 run mpc mpc_controller_robot2.py
```

```bash
colcon build && source install/setup.bash && 
source setup_acados.sh  && ros2 run mpc mpc_controller_robot2.py
```





## Contributing
 
