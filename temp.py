# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
import pandas as pd
import numpy as np
# This is the same obstacle data from the previous lesson.
filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
print(data)


from motion_planning import MotionPlanning


from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
import argparse


# TODO: read lat0, lon0 from colliders into floating point values
with open('colliders.csv', newline='') as f:
   reader = csv.reader(f)
   row1 = next(reader)
 
print(row1[1])
lat0 = float(row1[0].split(' ')[1])
lon0 = -float(row1[1].split('-')[1])
print(lon0)

row1[1].split(' ')[2]




if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    


drone.plan_path()


points= np.array([(316, 445), (316, 446), (317, 446), (317, 447), (318, 447), (318, 448), (319, 448), (319, 449), (320, 449), (320, 450), (321, 450), (321, 451), (322, 451), (322, 452), (323, 452), (323, 453), (324, 453), (324, 454), (325, 454), (325, 455), (326, 455)])
plt.scatter(points[:,0], points[:,1])
plt.show()


