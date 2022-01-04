import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

file = open("soft_sim.dat")
data = file.readlines()
file.close()
line = data[0].split(",")

width = int(line[0])
height = int(line[1])
frames = int(line[2])

size = width * height
xlim = (-0.5 * width, 1.5 * width)
ylim = (0, 1.5 * height)
 
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on = False, xlim = xlim, ylim = ylim)
ax.set_xticks([])
ax.set_yticks([])
scatter, = ax.plot([], [], 'o', markersize = 10, c = 'r')

def init():
    return []

def animate(t):
    line = data[t+1].split(",")
    
    x = [float(line[i]) for i in range(0, 2*size, 2)]
    y = [float(line[i]) for i in range(1, 2*size, 2)]
    
    scatter.set_data(x, y)
    return scatter,


if __name__ == "__main__":
    ani = animation.FuncAnimation(fig, animate, frames = frames, init_func = init, interval=500, blit=True)

    ani.save('video.mp4', fps = 24)
