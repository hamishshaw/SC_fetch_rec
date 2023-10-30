# Testing Object Detection with Fetch
This package creates a gazebo environment where the object detection pipeline can be tested\
This package has a few dependencies that must be installed, pcl 1.13, pcl_conversions, moveit, fetch_gazebo\
the find_object_3d package in this github must also be in your catkin workspace
## To run this package install as follows
```console
ln -s ~/yourpath/fetch_world_can ~/yourcatkinws
cd ~/yourcatkinws
catkin_make
```
## to run succesfully follow these steps in seperate tabs in console
```console
roslaunch fetch_can world.launch
```
wait until finished and in a new tab
```console
rosrun fetch_can start_position
```
wait until finished, and in the same tab
``` console
rosrun fetch_can can_grab
```
in a new tab
```console
rosrun find_object_3d find_object_3d
```
This will do the following\
1. start a gazebo environment with the fetch and some objects
2. move the fetch to the testing position
3. estimate the position of the can and pick it up

To play around with the objects on the table load the world .sdf into gazebo
```console
cd ~/yourpath/fetch_world_can/world
gazebo fetch_can_test.sdf
```
you can then change objects on the table and see that the object detection works well \
Save the new world as the same name so that it is correctly loaded
