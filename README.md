# Grid Mapper
A small cpp file that implements occupancy grid mapping of a robot in Stage via ROS using Opencv.
# Prerequisites
Needs opencv 2.x and some boost packages installed. Also requires existing .world file and .inc file to load via Stage.
# Notes
Tested on the previous .world file given in homework 2.
Currently acts on robot 1 and uses the laser data from its scanner in order to plot the occupancy grid.

```
rosrun stage_ros stageros 2017-02-11-00-31-57.world
rosrun grid_mapper grid_mapper [width] [height]
```

Recommended width and height is around 200x200.

Press the 'x' key in order to make a screenshot of the grip map.
