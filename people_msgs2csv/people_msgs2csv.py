#!/usr/bin/env python
import rosbag
import matplotlib.pyplot as plt
import numpy as np
import csv
from bayes_people_tracker.msg import PeopleTracker

'''
script that reads bags that contain PeopeTracker messages published on
/people_tracker/positions topic and writes the output in a csv format.
also filters out undesired detections
'''

# get distances of given point to all points in given list
def get_distances(point, point_list):
    x, y = point
    for p in point_list:
        x2, y2 = p
        yield np.linalg.norm((x - x2, y - y2))

def get_min_dist(point, point_list):
    return min(get_distances(point, point_list))

# load detections from a file in form of tuple of points
def load_detections(name):
    people = set()
    f = open(name, 'r')
    false_detections = tuple(f)
    f.close()
    false_detections = map(lambda x: x.strip().split(), false_detections)
    false_detections = map(lambda x: map(float, x), false_detections)
    false_detections = map(tuple, false_detections)
    false_detections = tuple(false_detections)
    return false_detections

# return matrix of rotation around z-axis in 3d
def rot_matrix(degrees):
    angle = np.radians(degrees)
    return np.array([np.cos(angle), -np.sin(angle), 0, np.sin(angle), np.cos(angle), 0, 0, 0, 1]).reshape((3, 3))

# decide if point is in area that is interesting for us
def valid_point(p):
    x, y, z = p
    # this is where the borders are defined, if you want to add or modify them, do it here
    room = (-9.25, 0.1, 3.0, 12.8)
    #elevator = (-4.7, 16.0, -6.8, 12.8)
    column1 = (-6.9, 8.7, -5.9, 8.2)
    column2 = (-2.0, 8.7, -1.5, 8.2)
    counter = (-1.5, 1.0, 1.0, 0.1)
    areas = (room, column1, column2, counter)
    # inside the room and outside the obstacles
    valid_result = (True,) + (len(areas) - 1)*(False,)
    result = tuple(map(lambda a: inside_rectangle((x, y), a), areas))
    return result == valid_result

def false_detection(p, false_detections):
    return get_min_dist(p[:-1], false_detections) < 0.5

# decide if point lies inside a rectangle given as [x1,y1,x2,y2] (rectangle must be oriented so that sides are parallel to axes)
def inside_rectangle(p, rec):
    xs = sorted([rec[0], rec[2]])
    ys = sorted([rec[1], rec[3]])
    x, y = p
    return xs[0] < x < xs[1] and ys[0] < y < ys[1]

# performs function on every element in li, func doesn't return anything
def foreach(li, func):
    for x in li:
        func(x)

# process a single bag
def process_bag(name, writer, R, false_detections):
    with rosbag.Bag(name) as bag:
        for _, msg, t in bag.read_messages(topics=['/people_tracker/positions']):
            for id, pose, vel in zip(msg.uuids, msg.poses, msg.velocities):
                pos = (pose.position.x, pose.position.y, pose.position.z)
                veloc = (vel.position.x, vel.position.y, vel.position.z)
                speed = np.linalg.norm(veloc)
                # rotate the position and velocity
                pos = np.matmul(R, pos)
                veloc = np.matmul(R, veloc)
                # compute phi
                signum = 1 if veloc[1] >= 0.0 else -1
                phi = signum * np.arccos(veloc[0] / speed)
                if speed > 3.0 or not valid_point(pos): continue
                writer.writerow([t, id, pos[0], pos[1], pos[2], veloc[0], veloc[1], veloc[2], speed, phi])

# the main function: reads names of bags to process from standard input, filters the data and outputs it in one .csv file
def main():
    #false_detections = load_detections('false_detections.txt')
    R = rot_matrix(0.0)
    bag_list = raw_input().split()
    out_name = ((bag_list[0].split('/'))[1])[:10] + '.csv'
    with open(out_name, 'w') as csv_out:
        writer = csv.writer(csv_out)
        writer.writerow(['t', 'id', 'pos_x', 'pos_y', 'pos_z', 'vel_x', 'vel_y', 'vel_z', 'speed', 'phi'])
        foreach(bag_list, lambda x: process_bag(x, writer, R, false_detections))

if __name__ == '__main__':
    main()
