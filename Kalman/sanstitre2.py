# -*- coding: utf-8 -*-
"""
Created on Thu Feb 21 13:48:38 2019

@author: olivi
"""

import numpy as np


G = np.array([[1,2,3], [4,5,6], [7,8,9]])

lG = np.dstack((G, G+10))
lG = np.dstack((lG, G+20))
lG = np.dstack((lG, G+30))
print(lG, lG.shape)
print(lG[0,0])

#X = np.array([0,0,1])
#print(X[:2])