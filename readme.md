# HMrC-SLAM: Enhanced Multi-Robot Collaborative Mapping through Hybrid Object and Scene Recognition

This repository contains the code and resources for our research article **"Enhanced Multi-Robot Collaborative Mapping through Hybrid Object and Scene Recognition"**, published in *The Visual Computer* journal. The study introduces a novel approach to improving collaborative mapping in multi-robot systems by combining object and scene recognition techniques, enabling more accurate and efficient environmental mapping. HMrC-SLAM has been tested using the ZED2 camera in real environments, and the recorded datasets are open-source and available for download at (https://doi.org/10.5281/zenodo.14712839).

## Citation

If you find this project helpful in your research, please consider citing our article:

**Title**: Enhanced Multi-Robot Collaborative Mapping through Hybrid Object and Scene Recognition  
**Journal**: *The Visual Computer*  

If you use this code, please cite it as:

**Zhensheng Zhou. (2025).**  
**strongman-zzs/hmrc_slam-master: hmrc_slam (v1.0.0).**  
**Zenodo. [https://doi.org/10.5281/zenodo.14714407](https://doi.org/10.5281/zenodo.14714407)**

Alternatively, you can use the DOI badge for citation:

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.14714407.svg)](https://doi.org/10.5281/zenodo.14714407)

# 1 Environment Setup
HMrC-SLAM is implemented on Ubuntu 18.04 with ROS Melodic. The system has been tested using the ZED2 stereo camera, and a dataset has been recorded for usage. Below are instructions for environment setup, installation, and usage of HMrC-SLAM.

## 1.1 Create a Catkin Workspace
```
mkdir -p ~/hmrcslam_ws/src
cd ~/hmrcslam_ws
source /opt/ros/noetic/setup.bash
catkin init
catkin config --extend /opt/ros/noetic
```

## 1.2 Clone the Source Repository
```
cd ~/hmrcslam_ws/src
git clone https://github.com/VIS4ROB-lab/hmrc_slam.git
```

# 2. Installation and Compilation of HMrC-SLAM
## 2.1 Compile DBoW2
```
cd ~/hmrcslam_ws/src/hmrc_slam/hmrc_slam/thirdparty/DBoW2/
mkdir build
cd build
cmake ..
make -j8
```

## 2.2 Compile g2o
```
cd ~/hmrcslam_ws/src/hmrc_slam/hmrc_slam/thirdparty/g2o
mkdir build
cd build
cmake --cmake-args -DG2O_U14=0 ..
make -j8
```

## 2.3 Unzip the Vocabulary File
```
cd ~/hmrcslam_ws/src/hmrc_slam/hmrc_slam/conf
unzip ORBvoc.txt.zip
```

## 2.4 Build the Code
```
cd ~/hmrcslam_ws/
catkin build hmrcslam --cmake-args -DG2O_U14=0 -DCMAKE_BUILD_TYPE=Release
source ~/hmrcslam_ws/devel/setup.bash
```

# 3. Usage Instructions
## 3.1 Start ROS Core
```
source ~/hmrcslam_ws/devel/setup.bash
roscore
```

## 3.2 Dataset Examples
HMrC-SLAM has been tested using the ZED2 camera in real environments, and the recorded datasets are open-source and available for download at (https://doi.org/10.5281/zenodo.14712839).

## Start the Server Launch File: ##
```
roslaunch hmrcslam Server.launch
```

## Start the Client Launch File: ##
```
roslaunch hmrcslam Client0_zed.launch
roslaunch hmrcslam Client1_zed.launch
```

## Start RVIZ: ##
```
cd ~/hmrcslam_ws/src/hmrc_slam/hmrc_slam
rviz -d conf/rviz/hmrcslam.rviz
```

## Play the Rosbag Files: ##
```
rosbag play zed_indoor_1.bag
rosbag play zed_indoor_2.bag
```
