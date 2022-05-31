#####################################
# Generate NS2 mobility trace for simulations
import os, sys
import numpy as np
import argparse

#---------------------------
# 0. Command Line Arguments
parser = argparse.ArgumentParser()
parser.add_argument('-n', '--numVeh', type=int, default=50, help='Number of vehicles')
parser.add_argument('-l', '--numLane', type=int, default=2, help='Number of lanes per direction')
parser.add_argument('-w', '--laneWidth', type=float, default=3.5, help='Width of each lane (in meters)')
parser.add_argument('-d', '--numDirection', type=int, default=2, choices=[1, 2, 4], help='Number of directions with vehicles at the junction')
args = parser.parse_args()

#---------------------------
# 1. Parameters
n = args.numVeh
w = args.laneWidth
l = args.numLane
numVehPerDirection = n // args.numDirection
vpd = [ numVehPerDirection for i in range(args.numDirection) ]
vpd[-1] += n - args.numDirection * numVehPerDirection

#---------------------------
# 1. Generate vehicle locations
locs = []
# Iterate over lane and slot
lane = 0
slot = 0
direction = 0
minX = 0
minY = 0
while(len(locs) < n):
    loc = [0, 0, 1.5]
    val1 = (0.5 + lane) * w
    val2 = (l + 0.5 + slot) * w

    # Based on direction, apply value
    if direction == 0:
        loc[0] = val1
        loc[1] = -val2
    elif direction == 1:
        loc[0] = -val1
        loc[1] = val2
    elif direction == 2:
        loc[0] = val2
        loc[1] = val1
    elif direction == 3:
        loc[0] = -val2
        loc[1] = -val1
    locs.append( loc )
    minX = min(loc[0], minX)
    minY = min(loc[1], minY)

    # Advance slot and lane
    direction += 1
    if direction >= args.numDirection:
        direction -= args.numDirection
        lane += 1
        if lane >= l:
            lane -= l
            slot += 1

for i, _ in enumerate(locs):
    locs[i][0] -= minX
    locs[i][1] -= minY

#---------------------------
# 2. Save to NS2 Mobility file
def saveNs2Mobility(locs, fn='ns2mobility.tcl'):
    with open(fn, 'w') as f:
        for i, loc in enumerate(locs):
            f.write( '$node_(' + str(i) + ') set X_ ' + str(loc[0]) + '\n' )
            f.write( '$node_(' + str(i) + ') set Y_ ' + str(loc[1]) + '\n' )
            f.write( '$node_(' + str(i) + ') set Z_ ' + str(loc[2]) + '\n' )

saveNs2Mobility(locs)































