# Udacity PCL Tutorial


## Dependencies:
* ROS Kinetic or Melodic

## How to use:
I will assume you are new to ROS.


```
mkdir -p udacity_ws/src
cd udacity_ws/src
git clone https://github.com/xmfcx/udacity-pcl-tut.git
cd ..
catkin_make
```
This should build the udacity-pcl-tut node in your workspace.

Then you can run ```roscore``` in a terminal. And in another terminal you can run:
```
cd udacity_ws
source devel/setup.bash
rosrun udacity-pcl-tut udacity-pcl-tut_exec
```
to run the executable.

To visualize it, in another terminal run ```rviz``` and as a configuration file choose ```udacity_ws/src/udacity-pcl-tut/rviz_stuff/udacity.rviz```


## Dataset

To feed application with a point cloud stream, you can download a bag file from https://github.com/udacity/self-driving-car/tree/master/datasets

or download these: https://drive.google.com/open?id=1fVjIAJaga9Ulf50WHIp9tVTfLWDxY6f-

and do:
```
rosbag play filename.bag
```
to play it.
