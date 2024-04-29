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


## Installation

- Install acados. [Here](src/mpc/README.md) there is instruction on how to install Acados.


## Usage

```bash
source install/setup.bash
python3 src/mpc/scripts/mpc_no_ros.py  
```

```bash
rosdep install --from-paths src -y --ignore-src
```

```bash
colcon build && source install/setup.bash
ros2 launch simulation_numerical numerical_simulation_launch.py 
```


```bash
colcon build && source install/setup.bash && 
source setup_acados.sh  && ros2 run mpc mpc_controller_robot1.py
```



## Contributing



## License

TBD


## Issues

- wheels not showing up in the numerical simulation (low priority)
- Conversion file is currently restasted in each package instead of one place (ie. one package)
- Make the MPC library so that only kinematic section has to changed. 
