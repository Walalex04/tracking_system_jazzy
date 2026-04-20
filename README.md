
# Tracking Robotic: A system based on camera for QUPA robots detection using opencv

![ROS 2](https://img.shields.io/badge/ros2-jazzy-blue?logo=ros)
![Maintainer](https://img.shields.io/badge/Maintainer-Walter%20Gonzabay-green)
![Maintainer](https://img.shields.io/badge/Maintainer-Diego%20Ponton-green)

**Tracking Robotic** is detection system of robots using arucos; the system was based on ___ and modifed for the use with QUPA. It was implemented on **ROS-JAZZY** 


## 🛠 System resources

For the correct executing, ensure you have the following resources

* **OS:** Ubuntu 24.04 LTS (Noble Numbat).
* **ROS 2:** Jazzy Jalisco (Desktop Install).
* **Principal Libraries:** 
    ```bash
    sudo apt install python3-numpy
                     libboost-python-dev
                     ros-jazzy-image-transport
                     ros-jazzy-cv-bridge
                     ros-jazzy-camera-info-manager
                     ros-jazzy-image-pipeline
                     
    ```
    The first two libreries are used for **openCV** and **cv_brdige**, see the principal documentation for more detaill [CV_Bride](https://index.ros.org/p/cv_bridge/#jazzy)


## Instalation and Compilation

1. **Create and clone the repository**
```bash
mkdir -p ~/catkin_ws
cd ~/catkin_ws
git clone https://github.com/Walalex04/tracking-system-noetic.git src
```




2. **install dependences** 

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```



3. **build and execute** 

```bash
cd ~/catkin_ws
colcon build --symlink-install
source devel/setup.bash
roslaunch track_sys track-sys_py.launch
```

## First Steps

Before to execute the software is important ensure about the existence of some file configuration and write the correct parameters for the execution


1. **Aruco generation**

```bash
roslaunch track_sys creation_marker_py.launch
```


2. **Camera Calibration**

You must set the parameter for the width and height of the camera image, if you do not, the usual values are 640 x 480
