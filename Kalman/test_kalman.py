# -*- coding: utf-8 -*-
"""
Created on Thu Feb 21 10:04:52 2019

@author: olivi
"""

from roblib_modified import *

float_formatter = lambda x: "%.6f" % x
np.set_printoptions(formatter={'float_kind':float_formatter})
plt.close()

class kalman_localisation:
    
    def __init__(self, X, theta, Γα, u):
        self.X = X
        self.theta = theta
        self.Γα = Γα
        self.u = u
        self.Gup = 1.*np.eye(3)
        self.A = np.array([[0., 0., cos(theta)], [0., 0., sin(theta)], [0., 0., -1.]])
        
        # GPS
        self.C1 = np.array([[1., 0., 0.], [0., 1., 0.]])
        self.Γβ1 = 0.48*np.eye(2)
        # DVL
        self.C2 = np.array([0., 0., 1.])
        self.Γβ2 = 1.*np.eye(1)
        # GPS + DVL
        self.C3 = np.eye(3)
        self.Γβ3 = np.array([[0.48, 0., 0.], [0., 0.48, 0.], [0., 0., 1.]])
        
#        self.flg_predict = 0
#        self.flg_GPS = 1
#        self.flg_DVL = 0
        
        self.dX = np.array([0., 0., 0.])
        self.range = 500
        self.dt = 0.01
        
    def set_mission(self, lm):
        self.lm = lm
    
    def apply_kalman_predict(self):
        return kalman_predict(self.X, self.Gup, self.u, self.Γα, self.A)

    def apply_kalman_GPS(self):
        y = self.X[:2]
        return kalman(self.X, self.Gup, self.u, y, self.Γα, self.Γβ1, self.A, self.C1)

    def apply_kalman_DVL(self):
        y = self.X[2]
        return kalman(self.X, self.Gup, self.u, y, self.Γα, self.Γβ2, self.A, self.C2)
    
    def apply_kalman_full(self):
        y = self.X
        return kalman(self.X, self.Gup, self.u, y, self.Γα, self.Γβ3, self.A, self.C3)
    
    def loop(self):
        self.lX = []
        for i in range(len(self.lm)-1):
            pt, r = np.array(self.lm[i][0]), self.lm[i][1]
            pt_1, r_1 = np.array(self.lm[i+1][0]), self.lm[i+1][1]
            V = pt_1 - pt
            for i in range(self.range):
                theta = arctan2(V[1], V[0])
                self.A = np.array([[0., 0., cos(theta)], [0., 0., sin(theta)], [0., 0., -1.]])
                d = self.dist_min_amers(pt, r, pt_1, r_1)
#                if self.flg_predict:
#                    print("predict\n")
#                    self.dX, self.Gup = self.apply_kalman_predict()
#                elif self.flg_GPS:
#                    print("GPS\n")
#                    self.dX, self.Gup = self.apply_kalman_GPS()
#                elif self.flg_DVL:
#                    print("DVL\n")
#                    self.dX, self.Gup = self.apply_kalman_DVL()
#                else :
#                    print("No situation flag selected\n")
#                print(self.dX)
                if d>0:         # UAV proche d'un point, remonte à la surface, passage en GPS + DVL
#                    print("full\n")
                    self.dX, self.Gup = self.apply_kalman_full()
                else:         # UAV en haute mer, plonge, passage en DVL pur
#                    print("DVL\n")
                    self.dX, self.Gup = self.apply_kalman_DVL()
                for i in range(3):
                    self.X[i] += self.dt*self.dX[i]
                self.lX.append(np.array([self.X[0], self.X[1], theta]))
                try :
#                    print("try")
                    self.lGup = np.dstack((self.lGup, np.sqrt(self.Gup)))
                except AttributeError :
#                    print("except")
                    self.lGup = np.sqrt(self.Gup)
        print(self.lGup[0,0])
        print(self.lGup[1,1])
        print(self.lGup[2,2])
        self.plot_results()
        
    def dist_min_amers(self, pt, r, pt_1, r_1):
#        print("r", r, "l", norm(self.X[:2]-pt), "r_1", r_1, "l1", norm(self.X[:2]-pt_1), "C1", r - norm(self.X[:2]-pt), "C2", r_1 - norm(self.X[:2]-pt_1) )
#        print(type(r - norm(self.X[:2]-pt)))
#        print(type(r_1 - norm(self.X[:2]-pt_1)))
#        
#        print(max(r - norm(self.X[:2]-pt),r_1 - norm(self.X[:2]-pt_1)))
        return max(r - norm(self.X[:2]-pt),r_1 - norm(self.X[:2]-pt_1))
    
    def plot_results(self):
        plt.figure()
        for i in range(len(self.lX)):
            draw_tank(self.lX[i],col='darkblue',r=0.1)
        plt.show()
        for i in range(3):
            plt.figure()
            plt.plot(self.lGup[i,i])


if __name__ == "__main__":
    X = np.array([0., 0., 1.])
#    theta = np.pi/4. # np.arctan2(X[1],X[0])
    Γα = 1.*np.eye(3)
    u = np.array([0., 0., 1.])
    
    kl = kalman_localisation(X, theta, Γα, u)
    lm = [[[0.,0.], 1.], [[5.,0.], 1.], [[5.,5.], 1.], [[0.,5.], 1.]]
    kl.set_mission(lm)
    kl.loop()
