# This is the Partial Views
This code utilises the PCL library to create a dataset of vfh signatures from a cad file in .ply format\

## How to run 
to first build and run make a build directory within the partialviews folder \
```console
mkdir build\
cd build\
```

once in build folder invoke cmake and make \
```console
cmake ..\
make\
```
Once these commands have been completed the project should be succesfully if you have the proper dependecies installed i.e. pcl 11.3\
## Running
the uploaded folder allready has the data folder filled, however the can subfolder can be deleted if you want to see how it all works\
The first step is to run the view generator. This create 42 views from different angles from a .ply file and also vfh.pcd files
This can be ran with the following command from the partialviews folder
```console
./build/partialViews can_scaled.ply
```
this will populate the current folder with a bunch of .pcd files, move these to a new folder in the data folder\
The next to run is the training executable which will create the data used when detecting objects\
it is run as follows \
```console
./build/train /data
```
finally results can be viewed by testing one of the vfh against the generated data with the following \
```console
./build/nearest_neighbors -k 16 -thresh 50 data/can/can_view_0_vfh.pcd
```
