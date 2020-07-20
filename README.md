# processing velodyne scans
In order to run:
1. install ROS (http://wiki.ros.org/melodic/Installation)
1. compile change detector using cmake (you may need to install some libraries)  
    * `cd change_det`  
    * `cmake CMakeLists.txt`  
    * `make`
1. make sure you have `velodyne_lifelong` installed  
    * clone https://github.com/gestom/3L4AV to your catkin workspace source directory  
    * some changes need to be made to the source files of the `velodyne_lifelong` to make this program work; velodyne_lifelong folder from this repo contains the modified versions of the files, to make the changes overwrite the source with these (I am sorry for this clumsy solution, will try to make this step easier in the future)  
    * compile using `catkin_make`  
1. the `main.sh` script is able to download and process rosbags from Google drive using a text file with files' ids

if you have the data anywhere else than on Google drive, you need to modify the `main.sh` accordingly  

you will need (a lot of) free disk space for the downloaded bag and the bag with the pointcloud data, which the downloaded bag transforms into  
