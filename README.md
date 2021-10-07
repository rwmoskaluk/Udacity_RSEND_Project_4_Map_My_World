# Udacity_RSEND_Project_4_Map_My_World
Udacity RSEND Project 4 Map My World

[image1]: ./pictures/rtabmap_viz.png "Rtabmap visualization"
[image2]: ./pictures/rtabmap_databaseviewer.png "Rtabmap visualization database viewer"


### Project Overview
The scope of this project was to learn more about the mapping package rtabmap in ROS.  This package allows for distinct features in an environment to be mapped which can then later be used for localization.  The robot was driven around using the Teleop package and data was then collected about the environment.

# How to run

Clone this repo

```
cd catkin_ws
catkin_make
source devel/setup.bash
```

Launch the world in terminal
```
source devel/setup.bash
roslaunch my_robot my_world.launch
```

Launch the mapping process in another terminal
```
source devel/setup.bash
roslaunch mapping mappling.launch
```

Launch Teleop package in another terminal
```
cd catkin_ws/src
git clone https://github.com/ros-teleop/teleop_twist_keyboard
cd ..
catkin_make
source devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

# Visualizing the Environment
To visualize the environment make sure to activate `Rtabmapviz GUI` as shown below.  This screen will show how many loop closures for the environment and give a visual representation of how the environment is mapped.

To turn the `Rtabmapviz GUI` on and off that can be done in the `mapping.launch` file

![alt text][image1]


After mapping has been done to a satisfactory level, one can view the saved data `rtabmap.db` which is stored in `/home/<hostname>/.ros/` where `<hostname>` denotes your computer's name

![alt text][image2]


# Example Running


<img src="pictures/rviz_driving.gif?raw=true" width="720px">
<img src="pictures/rtabmap_visualization.gif?raw=true" width="720px">


# Localization

After using `rtabmap` to generate a map of the environment the results can be fed back into the system for better localization of the environment.  The launch file is modified from the `mapping.launch` to utilize this information

Make sure the `mapping.launch` process is closed from above
```
roslaunch mapping localization.launch
```



# Notes:

One key thing is to realize that `rtabmap` is utilizing a generic base_footprint for the frame_ids.  Changing the URDF or changing in the launch file is required to get the proper data from the topics.


Otherwise this error will occur where the data is not piped to the proper end topics during remapping.

```
[ WARN] [1633285689.744699814, 435.400000000]: /rtabmap/rtabmapviz: Did not receive data since
 5 seconds! Make sure the input topics are published ("$ rostopic hz my_topic") and the timest
amps in their header are set. If topics are coming from different computers, make sure the clo
cks of the computers are synchronized ("ntpdate"). If topics are not published at the same rat
e, you could increase "queue_size" parameter (current=10).
/rtabmap/rtabmapviz subscribed to (approx sync):
   /rtabmap/odom \
   /camera/rgb/image_raw \
   /camera/depth/image_raw \
   /camera/rgb/camera_info \
   /scan
```
