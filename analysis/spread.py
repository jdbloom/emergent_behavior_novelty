import numpy as np


data = np.loadtxt(open('../data.dat', 'rb'), delimiter=',')

total_robots = 0

for tic in data:
    time = int(tic[0])
    robot = int(tic[1])
    if time == 1:
        total_robots = robot

total_robots += 1


positions = [[] for i in range(total_robots)]

for tic in data:
    robot = int(tic[1])

    pose = (tic[2], tic[3], tic[4])

    positions[robot].append(pose)

