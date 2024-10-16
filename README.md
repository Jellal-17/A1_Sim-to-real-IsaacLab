![A1](/Capture1.png)

# Unitree A1 Simulation to Real Deployment using Isaac Lab

[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.2.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-1.2.0-silver)](https://isaac-sim.github.io/IsaacLab)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/20.04/)
[![Windows platform](https://img.shields.io/badge/platform-windows--64-orange.svg)](https://www.microsoft.com/en-us/)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://pre-commit.com/)
[![License](https://img.shields.io/badge/license-MIT-yellow.svg)](https://opensource.org/license/mit)

## Overview

This branch focuses on the setup and deployment of the Unitree A1 robot. It covers the basics of hardware and software installation, particularly working with Intel RealSense cameras and the Unitree SDK, along with ROS-to-Real integration for seamless operation.

**Keywords:** unitree, a1, simulation-to-real, robotics, Isaac Sim

## Installation

To start working with the Unitree A1 at a low level, follow these steps:

1. **Install the Intel RealSense Camera Library:**
   - You need to set up the Intel RealSense camera library and SDK. Follow the instructions at [this GitHub repo](https://github.com/ysozkaya/RealSense-Jetson) to build Python bindings and use `pyrealsense2`.
     
2. **Install Unitree A1 SDK:**
   - Download and install the Unitree legged SDK v3.3.1 from [this link](https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/v3.3.1).

3. **ROS Integration for Unitree A1:**
   - To integrate ROS with the Unitree A1 SDK, download the ROS wrapper from [this link](https://github.com/unitreerobotics/unitree_ros_to_real/releases/tag/v3.2.1).

4. **Further Development:**
   - The repository will continue to be updated, and we are actively developing beyond this initial setup to enhance real-world deployment.

## Code Formatting

This repository uses **pre-commit** to ensure consistent code formatting. To install and use pre-commit, follow these steps:

To Install pre-commit: 

```bash
pip install pre-commit
```

Then you can run pre-commit with:

```bash
pre-commit run --all-files
```
## Future Work
We will continue updating this repository with new developments related to the Unitree A1's simulation-to-real workflow using Isaac Lab. Stay tuned for further updates.
