![A1](/Capture1.png)
# Unitree A1's Simulation to Real using Isaac Lab

[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.2.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-1.2.0-silver)](https://isaac-sim.github.io/IsaacLab)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/20.04/)
[![Windows platform](https://img.shields.io/badge/platform-windows--64-orange.svg)](https://www.microsoft.com/en-us/)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://pre-commit.com/)
[![License](https://img.shields.io/badge/license-MIT-yellow.svg)](https://opensource.org/license/mit)

## Overview
This branch specifically on starting up with your Unitree A1 robot and setting up the basics of deployment. 

**Keywords:** unitree, a1


### Installation
1. There are a few requirements to start working on low-level with the Unitree A1. We need to install the library of intel realsense camera, SDK of the Unitree A1 and ROS-to-Real from Unitree.
2. Following this (https://github.com/ysozkaya/RealSense-Jetson), construct the pyhton bindings as well to use the pyrealsense2. 
3. For the next step of ours, begin by dowloading the v3.3.1 of the Unitree legged SDK https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/v3.3.1.
4. Now, you can use this https://github.com/unitreerobotics/unitree_ros_to_real/releases/tag/v3.2.1 to start working with ROS wrapper to the Unitree SDK, and for further development of the code, we are working to build up on this. 
5. This repository will update the deployment beyond this later. 
```

## Code formatting

There is a pre-commit template to automatically format the code.
To install pre-commit:

```bash
pip install pre-commit
```

Then you can run pre-commit with:

```bash
pre-commit run --all-files
```
