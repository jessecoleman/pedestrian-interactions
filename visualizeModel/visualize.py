

import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

#constant settings
xmin = -10
xmax = 10
ymin = -10
ymax = 10
dt = 0.05
timescale = 1

#load the file
if len(sys.argv) is not 2:
	print('must priovide a file to visualize!')
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

print('making figure')
fig, ax = plt.subplots()
ax.set_xlim(xmin,xmax)
ax.set_ylim(ymin,ymax)
people, = ax.plot([],[], 'bo')

#fire
patch = plt.Circle((0,0),0.01, fill = False, ec='r');
ax.add_patch(patch)

#walls
plt.plot([-4,4],[8,8],'k-')
plt.plot([-4,4],[-8,-8],'k-')
plt.plot([-4,-4],[8,-8],'k-')
plt.plot([4,4],[8,-8],'k-')
plt.plot([4,4],[2,-2],'w-',linewidth=2)
# plt.plot([-4,-4],[2,-2],'w-',linewidth=2)

radiusGrowth = 0.3

def animate(i):

	print 'step: ', i, 'time: ', i*dt*timescale
	state = poslist[i * timescale];
	people.set_data(state[:,0], state[:,1])
	patch.set_radius(i * dt * timescale * radiusGrowth)
	return people

print('making animation')
ani = animation.FuncAnimation(fig, animate, frames=int(numframes), interval=1000*dt//timescale, blit=False)

print('showing plot')
plt.show()

# print 'saving file'
# Writer = animation.writers['ffmpeg']
# writer = Writer(fps=15, bitrate=1800)
# ani.save(fname+'.mp4', writer=writer)

print('done')
