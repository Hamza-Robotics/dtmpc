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
colcon build && source install/setup.bash && ros2 launch simulation_numerical numerical_simulation_launch.py 
```
```bash
colcon build && source install/setup.bash && ros2 launch trajectory_generator trajectory.launch.py 
```


## Contributing



## License

TBD


## Issues

- wheels not showing up in the numerical simulation (low priority)
- Conversion file is currently restasted in each package instead of one place (ie. one package)
-
