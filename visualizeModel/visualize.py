

import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

#constant settings
xmin = -10
xmax = 10
ymin = -10
ymax = 10
dt = 0.005
timescale = 4

#load the file
if len(sys.argv) is not 2:
	print 'must priovide a file to visualize!'
	exit(-1)

fname = sys.argv[1]
file = open(fname, 'r')

header = file.readline().strip().split(';')[1:] #ignore time
numpeople = len(header)

poslist = []

for line in file:
	split = line.strip().split(';')[1:];
	entry = np.zeros((numpeople, 2))

	for i,pos in enumerate(split):
		pos = pos.replace('(','')
		pos = pos.replace(')','')
		entry[i] = (np.fromstring(pos, dtype=float,sep=','))

	poslist.append(entry)

numframes = len(poslist) / timescale

print 'making figure'
fig, ax = plt.subplots()
ax.set_xlim(xmin,xmax)
ax.set_ylim(ymin,ymax)
people, = ax.plot([],[], 'bo')


def animate(i):

	# print 'step: ', i, 'time: ', i*dt*timescale
	state = poslist[i * timescale];
	people.set_data(state[:,0], state[:,1])
	return people

print 'making animation'
ani = animation.FuncAnimation(fig, animate, frames=numframes, interval=1000*dt/timescale, blit=False)

print 'showing plot'
plt.show()

