#!/usr/bin/env bash
function gdrive_download () {
  CONFIRM=$(wget --quiet --tries=0 --read-timeout=20 --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate "https://docs.google.com/uc?export=download&id=$1" -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')
  wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$CONFIRM&id=$1" -O $2
  rm -rf /tmp/cookies.txt
}

#used paths
BAGFILENAME=$(pwd)"/tmp/velodyne.bag"
PCLBAGFILENAME=$(pwd)"/tmp/pcl.bag"
LINKSFILENAME=$(pwd)"/links.txt"
CHANGESFILENAME=$(pwd)"/tmp/changes.txt"

#preparation
source ~/catkin_ws/devel/setup.bash
export OMP_CANCELLATION=true
mkdir -p output
mkdir -p tmp
rm -rf tmp/*

#number of bag being processed
curbag=1

while read line; do
  line=($line)
  curbagname=${line[0]}
  curbagpath=$curbagname
  curbagname=$(basename $curbagname)
  echo processing bag $curbagname
  if [ -d $(pwd)/output/$curbagname ]; then
    echo bag already processed, moving on
    ((curbag++))
    continue
  fi
  #download the bag
  echo started downloading $curbagname
  gdrive_download ${line[1]} $BAGFILENAME < /dev/null &> /dev/null
  echo download finished
  #convert it to pointcloud
  echo started converting velodyne scans to point cloud to $PCLBAGFILENAME
  #roslaunch velodyne/velodyne2pointcloud.launch bag:=$BAGFILENAME output_bag:=$PCLBAGFILENAME < /dev/null &> /dev/null
  echo converting finished
  #run the change detection
  echo started change detection
  #change_det/octree_change_detection $PCLBAGFILENAME < /dev/null 2> /dev/null > $CHANGESFILENAME
  echo change detection finished
  #run people detector on intervals and save the data
  echo starting people detector
  curint=1
  OUTPUTPATH=$(pwd)/output/$curbagname
  mkdir -p $OUTPUTPATH
  while read interval; do
    arr=($interval)
    start=${arr[0]}
    end=${arr[1]}
    length=$((end-start))
    roslaunch velodyne/velodyne_interval.launch bag:=$BAGFILENAME start:=$start length:=$length output_bag:=$OUTPUTPATH/$curint < /dev/null &> /dev/null
    #python people_msgs2csv/people_msgs2csv.py $OUTPUTPATH/$curint.bag $OUTPUTPATH/$curint.csv
    ((curint++))
  done < $CHANGESFILENAME
  echo detecting finished
  #clean up (is it neccesary though?)
  rm $PCLBAGFILENAME
  rm $BAGFILENAME
  ((curbag++))
  break

done < $LINKSFILENAME
rm -rf tmp/*
echo all rosbags processed
