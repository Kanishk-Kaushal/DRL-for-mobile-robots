# DRL-robot-navigation


Deep Reinforcement Learning for mobile robot navigation in ROS2 Gazebo simulator. Using Twin Delayed Deep Deterministic Policy Gradient (TD3) neural network, a robot learns to navigate to a random goal point in a simulated environment while avoiding obstacles. Obstacles are detected by laser readings and a goal is given to the robot in polar coordinates. Trained in ROS2 Gazebo simulator with PyTorch.  Tested with ROS2 Foxy on Ubuntu 20.04 with python 3.8.10 and pytorch 1.10.

Training example (Notice how robot fails to reach goal and collides almost every time):
<p align="center">
    <img width=100% src="https://github.com/Kanishk-Kaushal/DRL-for-mobile-robots/blob/main/training.gif">
</p>


Testing example (Notice how robot reaches goal  state almost every time):
<p align="center">
    <img width=100% src="https://github.com/Kanishk-Kaushal/DRL-for-mobile-robots/blob/main/testing.gif">
</p>

**Project Report**


Some more information about the implementation is available [here](https://github.com/Kanishk-Kaushal/DRL-for-mobile-robots/blob/main/DS5220_DRL_for_Mobile_Robots_002306180.pdf)

## Installation
Main dependencies: 

* [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html)
* [PyTorch](https://pytorch.org/get-started/locally/)
* [Tensorboard](https://github.com/tensorflow/tensorboard)
* [Gazebo](https://classic.gazebosim.org/download)

Clone the repository:
```shell
$ cd ~
### Clone this repo
$ git clone https://github.com/Kanishk-Kaushal/DRL-for-mobile-robots
```
The network can be run with a standard 2D laser, but this implementation uses a simulated [3D Velodyne sensor](https://github.com/lmark1/velodyne_simulator)

Compile the workspace:
```shell
$ cd ~/DRL-for-mobile-robots
### Compile
$ colcon build
```

Open a terminal and set up sources:
```shell
$ cd
$ source /opt/ros/foxy/setup.bash
$ source DRL-for-mobile-robots/install/setup.bash
```

Run the training:
```shell
$ ros2 launch td3 training_simulation.launch.py
```

To check the training process on tensorboard:
```shell
1. Navigate to src/td3/scripts/train_velodyne.py
2. Open this file in Visual Studio
3. Click on launch Tensorboard application
```

To kill the training process:
```shell
$ [Ctrl+C]
```

Once training is completed (Took me ~7500 episodes), test the model:
```shell
$ ros2 launch td3 test_simulation.launch.py
```

Gazebo environment:
<p align="center">
    <img width=80% src="https://github.com/Kanishk-Kaushal/DRL-for-mobile-robots/blob/main/gz.png">
</p>

Rviz:
<p align="center">
    <img width=80% src="https://github.com/Kanishk-Kaushal/DRL-for-mobile-robots/blob/main/rviz.png">
</p>
