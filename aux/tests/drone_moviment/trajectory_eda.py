import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


xs = np.linspace(-50, 50, 5)
zs = np.linspace(0, 50, 5)
grid = np.meshgrid(xs, xs, zs)


xtarget = np.e * 10
ytarget = np.pi * 10
ztarget = 0.0


xuav = 0.0
yuav = 0.0
zuav = 10.0



vecx = xtarget - xuav
vecy = ytarget - yuav
vecz = ztarget - zuav

g = 9.81
acceleration_max =  g
const = np.sqrt(acceleration_max/(abs(xuav - xtarget)))


'''
    s(t) = s0 + v.dt
'''
times = np.linspace(0, 20, 20)

def cinematica(s0, sf, constant, t):

    '''
        s(t) = sf(1 - e^{-ct}) + s0 * e^{-ct}
        v(t) = s'
        a(t) = s"
    '''
    
    pos_t = sf*(1 - np.exp(-constant*t)) + s0 * np.exp(-constant*t)

    vel_t = (sf - s0) * constant * np.exp(-constant*t)

    a_t = (-sf + s0) * constant**2 * np.exp(-constant*t)
    
    return pos_t, vel_t, a_t



xt_uav, vxt_uav, axt_uav = cinematica(s0 = xuav, sf = xtarget, constant = const, t = times)
yt_uav, vyt_uav, ayt_uav = cinematica(s0 = yuav, sf = ytarget, constant = const, t = times)
zt_uav, vzt_uav, azt_uav = cinematica(s0 = zuav, sf = ztarget, constant = const, t = times)


fig = plt.figure(figsize=(20,15))

ax = fig.add_subplot(111, projection='3d')
grid_pts = ax.scatter(grid[0], grid[1], grid[2], c='black', alpha=0.4, s = 1)
target_pos = ax.scatter(xtarget, ytarget, ztarget, c='orangered', s = 50, marker='s', label = 'Fire')


uav_pos = ax.scatter(xt_uav, yt_uav, zt_uav, c=times, s = 25, marker = 'x', label = 'UAV', cmap = "rainbow")
uav_pos0 = ax.scatter(xt_uav[0], yt_uav[0], zt_uav[0], s = 25, label = 'UAV s(t = 0)')


ax.set_title('Grid')
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_zlabel('z [m]')

# fig.colorbar(pts ,shrink=0.5, aspect=8)
plt.legend()
plt.show()


# fig2d = plt.figure(figsize=(20,15))

# plt.plot(vxt_uav, times)