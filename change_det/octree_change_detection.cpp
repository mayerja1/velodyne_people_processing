#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/io/pcd_io.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include <iostream>
#include <vector>
#include <ctime>
#include <cmath>
#include <string>
#include <algorithm>
#include <functional>

#include "peak_detector.h"


using PointType = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointType>;

/*
This program reads pointcloud2 messages from a rosbag and outputs intervals where it thinks
something is happening (ideally someone is walking there)
*/

// TODO: make these just pairs

// state of the pointcloud (time and number of changes)
typedef struct {
  float time;
  int changes;
} state;

typedef struct {
  float start;
  float end;
} interval;

// just prints a vector
template <class T>
void print_vector (int start, int end, std::vector<T>& v) {
  for (size_t i = start; i < end; i++) {
    std::cout << v[i] << ',';
  }
  std::cout << '\n';
}

// creates intervals from points in time using given parameters
void print_intervals (std::vector<float>& times) {
  const float max_gap = 1.0f;
  const float padding = 5.0f;
  const float min_interval_length = 4.0f;
  if (times.empty()) return;
  float length = 0.0f;
  float first_time = times[0];
  float last_time = times[0];

  std::vector<interval> intervals;
  for (unsigned int i = 1; i < times.size(); i++) {
    float time = times[i];
    if (time - last_time > max_gap || i == times.size() - 1) {
      if (length >= min_interval_length) {
        interval it;
        it.start = std::max(0.0f, first_time - padding);
        it.end = last_time + padding;
        if (intervals.empty() || it.end > intervals.back().end) {
          intervals.push_back(it);
        } else {
          intervals.back().end = it.end;
        }
      }
      length = 0.0f;
      first_time = time;
      last_time = time;
    } else {
      length += time - last_time;
      last_time = time;
    }
  }
  if (intervals.empty()) return;
  // merge intervals that are close together
  const int max_gap_between_intervals = 10;
  for (auto it = intervals.begin() + 1; it != intervals.end(); it++) {
    if ((*it).start - ((*(it - 1)).end < max_gap_between_intervals)) {
      (*(it - 1)).end = (*it).end;
      it = intervals.erase(it);
      if (it == intervals.end()) break;
    }
  }
  // output
  for (const auto& i : intervals) {
    std::cout << round(i.start) << ' ' << round(i.end) << '\n';
  }
}

// decides where in time is something happening
std::vector<float> find_changes (std::vector<state>& states) {
  std::vector<float> ret;
  std::vector<double> changes;
  // sort because vector might not be sorted due to paralelization
  std::sort(states.begin(), states.end(), [](state& s1, state& s2) {return s1.time < s2.time;});
  for (const state& s: states) {
    changes.push_back(s.changes);
    //std::cout << s.time << ' ' << s.changes << '\n';
  }
  std::vector<int> signals = smoothedZScore(changes);
  assert(signals.size() != 0);
  for (unsigned int i = 0; i < states.size(); i++) {
    if (signals[i] == 1) {
      ret.push_back(states[i].time);
    }
  }
  return ret;
}

// process .dat file with changes and print intervals used to test interval making and peak detection
// mainly used for testing purposes
void process_dat (std::string filename) {
  using namespace std;
  string STRING;
	ifstream infile;
	infile.open (filename, std::ifstream::in);
  string line;
  float a;
  int b;
  std::vector<state> states;
  while (infile >> a >> b) {
      state s;
      s.time = a;
      s.changes = b;
      states.push_back(s);
  }
	infile.close();
  auto changes = find_changes(states);
  print_intervals(changes);
}

int main (int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "enter rosbag name as first paramater" << '\n';
    return 1;
  }
  if (!omp_get_cancellation()) {
    std::cout << "set OMP_CANCELLATION by typing 'export OMP_CANCELLATION=true' and run again" << '\n';
    return 2;
  }
  const int batch_size = 3000;
  //process_dat(argv[1]);
  //return 0;
  // Octree resolution - side length of octree voxels
  const float resolution = 0.5f;
  const int min_points_per_leaf = 0;

  // the rosbag with pointcloud2
  rosbag::Bag bag;
  bag.open(argv[1], rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne_points")); // the topic where the cloud msgs will be found
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // iterator of messages
  auto it = view.begin();
  // first message
  auto msg = (*it).instantiate<sensor_msgs::PointCloud2>();
  auto start_time = msg->header.stamp;

  // reference pointcloud either from file or the firs msg
  Cloud::Ptr ref (new Cloud);
  if (pcl::io::loadPCDFile<PointType> ("reference.pcd", *ref) == -1) {
    PCL_ERROR ("Couldn't read file reference.pcd, using first cloud as a reference \n");
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*ref);
  }

  unsigned int processed_cnt = 0;
  auto states = new std::vector<state>();

  while (it != view.end()) {
    #pragma omp parallel for
    for (int i = 0; i < batch_size; i++) {
      // Instantiate octree-based point cloud change detection class
      pcl::octree::OctreePointCloudChangeDetector<PointType> octree (resolution);

      // Add points from ref to octree
      octree.setInputCloud (ref);
      octree.addPointsFromInputCloud ();

      // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
      octree.switchBuffers ();
      sensor_msgs::PointCloud2::Ptr msg;
      ros::Time msg_time;
      bool exit = false;
      // work with the message iterator in a critical section
      #pragma omp critical
      {
        if (it == view.end()) {
          exit = true;
        } else {
          msg = (*it).instantiate<sensor_msgs::PointCloud2>();
          msg_time = msg->header.stamp;
          it++;
        }
      }
      #pragma omp cancellation point for
      if (exit) {
        #pragma omp cancel for
      }

      // inform about progress
      if (++processed_cnt % batch_size == 0) {
        std::cerr << processed_cnt << "msgs processed time: " << msg_time - start_time << std::endl;
      }

      // cloudB - current cloud which we are comparing with reference
      pcl::PCLPointCloud2 pcl_pc2;
      pcl_conversions::toPCL(*msg, pcl_pc2);
      Cloud::Ptr cloudB(new Cloud);
      pcl::fromPCLPointCloud2(pcl_pc2,*cloudB);

      //pcl::io::savePCDFile("reference.pcd", cloudB);
      //return 1;

      // Add points from cloudB to octree
      octree.setInputCloud (cloudB);
      octree.addPointsFromInputCloud ();

      std::vector<int> newPointIdxVector;
      // Get vector of point indices from octree voxels which did not exist in previous buffer
      octree.getPointIndicesFromNewVoxels (newPointIdxVector, min_points_per_leaf);
      // work with shared vector in a critical section
      #pragma omp critical
      {
        state s;
        s.time = (msg_time - start_time).toSec();
        s.changes = newPointIdxVector.size();
        states->push_back(s);
      }
    }
  }
  bag.close();
  // vector of times in the bag where change was detected in seconds
  std::vector<float> change_times = find_changes(*states);
  print_intervals(change_times);

  return 0;
}
