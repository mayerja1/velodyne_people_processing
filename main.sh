#!/usr/bin/env bash
function gdrive_download () {
  CONFIRM=$(wget --quiet --tries=0 --read-timeout=20 --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate "https://docs.google.com/uc?export=download&id=$1" -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')
  wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$CONFIRM&id=$1" -O $2
  rm -rf /tmp/cookies.txt
}

BAGFILENAME=$(pwd)"/tmp/velodyne.bag"
PCLBAGFILENAME=$(pwd)"/tmp/pcl.bag"
LINKSFILENAME="links.txt"
CHANGESFILENAME="/tmp/changes.txt"
curbag=1
#preparation
source ~/catkin_ws/devel/setup.bash
mkdir -p output
rm -rf tmp
mkdir tmp

while read line; do
  line=($line)
  curbagname=${line[0]}
  echo processing bag no. $curbag
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
  roslaunch velodyne/velodyne2pointcloud.launch bag:=$BAGFILENAME output_bag:=$PCLBAGFILENAME < /dev/null &> /dev/null
  echo converting finished
  #run the change detection
  echo started change detection
  change_dec/octree_change_detection $PCLBAGFILENAME < /dev/null 2> /dev/null > $CHANGESFILENAME
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
    python people_msgs2csv/people_msgs2csv.py $OUTPUTPATH/$curint.bag $OUTPUTPATH/$curint.csv
    ((curint++))
  done < $CHANGESFILENAME
  echo detecting finished
  #clean up
  rm $PCLBAGFILENAME
  rm $BAGFILENAME
  #break #for testing purposes
  ((curbag++))
done < $LINKSFILENAME
echo all rosbags processed
