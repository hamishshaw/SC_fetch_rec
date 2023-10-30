# SC_fetch_rec

This project provides utility to enable the fetch robot to pick up a can of coke.

The project has 3 folders which provide functionality. See each folder for further info on how to run and what they do
## Overall Use
This project provides packages that can detect a coke can using PCL's vfh descriptor to analyse pointcloud data \ 
The PCL library is the sole provider of detection in this project and no RGB data is used\
The pipeline works as follows 
1. vfh descriptors are created that allows pointclouds to be compared quickly, giving a likness value
2. a testing data set is created that is used in the object detection system that the fetch received data from
3. a pointcloud is first clustered so that each cluster can be compared to the vfh descriptors
4. a cluster is then selected as the most similar cluster and the centre of the cluster is calculated
5. this location is sent to the fetch robot and the end affector is moved to this location using moveit
## Contributions
33.3% - Hamish Shaw 12947709
33.3% - Madeleine Croker 14259067
33.3% Issam Ur Rahman Ammar Syed 13923560
