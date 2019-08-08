import rosbag
import matplotlib.pyplot as plt
import numpy as np
from bayes_people_tracker.msg import PeopleTracker
from sys import argv

'''
simple script that reads bags that contain PeopeTracker messages published on
/people_tracker/positions topic. also filters out obviously wrong detections
'''

HIGHEST_HUMAN_SPEED  = 2.5
LOWEST_HUMAN_SPEED   = 0.5

# get a line of csv file from a list of values
def list2csv(li):
    li = map(str, li)
    ret = ''
    for x in li:
        ret += x + ','
    return ret[:-1] + '\n'

if __name__ == '__main__':
    if len(argv) < 3:
        print('usage: python {0} INPUT OUTPUT'.format(argv[0]))
        exit(1)
    speeds = []
    with open(argv[2], 'w') as file, rosbag.Bag(argv[1]) as bag:
        file.write('t,id,pos_x,pos_y,pos_z,orient_x,orient_y,orient_z,orient_w,vel_x,vel_y,vel_z\n')
        for _, msg, t in bag.read_messages(topics=['/people_tracker/positions']):
            pplcnt = 0
            for id, pose, vel in zip(msg.uuids, msg.poses, msg.velocities):
                speed = np.linalg.norm([vel.position.x, vel.position.y, vel.position.z])
                # skip things not moving or moving insanely fast
                if not LOWEST_HUMAN_SPEED < speed < HIGHEST_HUMAN_SPEED: continue
                ids.add(id)
                file.write(list2csv([t, id, pose.position.x, pose.position.y, pose.position.z, \
                                            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, \
                                            vel.position.x, vel.position.y, vel.position.z]))
                speeds.append(speed)
    #plt.hist(speeds, normed=True, bins=300)
    #plt.show()
