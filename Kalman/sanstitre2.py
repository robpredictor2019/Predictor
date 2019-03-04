# -*- coding: utf-8 -*-
"""
Created on Thu Feb 21 13:48:38 2019

@author: olivi
"""

import numpy as np
from roblib_modified import *


G = np.array([[1,2,3], [4,5,6], [7,8,9]])

lG = np.dstack((G, G+10))
lG = np.dstack((lG, G+20))
lG = np.dstack((lG, G+30))
print(lG, lG.shape)
print(lG[0,0])

#X = np.array([0,0,1])
#print(X[:2])


def draw_ellipse(c,rx, ry,ax,col):
    #draw_disk(array([[1],[2]]),0.5,ax,"blue")
    e = Ellipse(xy=c, width=2*rx, height=2*ry, angle=0)
    ax.add_artist(e)
    e.set_clip_box(ax.bbox)
    e.set_alpha(0.1)
    e.set_edgecolor(col)


lX = [np.array([0, 0, 1]), np.array([2, 2, 1])]
lGup = np.dstack((np.array([[1,0,0], [0,3,0], [0,0,1]]), np.array([[4,0,0], [0,2,0], [0,0,2]])))
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal')
ax.set_xlim(-2,5)
ax.set_ylim(-2,5)
# draw_tank(self.lX[i],col='darkblue',r=0.1)
draw_ellipse(lX[0][:2].T,lGup[0,0,0], lGup[1,1,0],ax,"blue")
draw_ellipse(lX[1][:2].T,lGup[0,0,1], lGup[1,1,1],ax,"red")
# plt.xmin=-2
# plt.xmax=5
# plt.ymin=-2
# plt.ymax=5
plt.show()
