# SC_fetch_rec

This project provides utility to enable the fetch robot to pick up a can of coke.

Currently the euc_cluster folder contains a package the will take a snapshot of a depth image and calculate of the clustered objects within the scene using the PCL libraries features.

# TODO 
object recognition - check each cluster and see whether it is a coke can

moveit - have moveit calculate an arm movement to reach the detected object and pick it up

