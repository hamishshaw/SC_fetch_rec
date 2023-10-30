# This package provides an object detection method that searches for coke cans
This package has the capability to search for any object that the appropriate training data is generated.\
Using partial views training data can be created for any object with a 3d cad file 
## how to use
to first compile link this folder into your catkin workspace or copy.
```console
ln -s ~/yourpath/find_object_3d ~/yourcatkinws
cd ~/yourcatkinws
catkin_make
```
This package publishes some pointclouds of clustered objects and the estimated location of the coke can as a pose message \
this package is setup to subscribe to the fetch's pointcloud data and can be changed if needed \
It is run with
```console
rosrun find_object_3d find_object_3d
```
