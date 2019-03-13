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

    def __init__(self, X, Γα, u, r, dt):
        self.X = X
        self.Γα = Γα
        self.u = u
        self.Gup = 0.*np.eye(3)

        err_GPS = 0.48
        err_DVL = 0.3

        # GPS
        self.C1 = np.array([[1., 0., 0.], [0., 1., 0.]])
        self.Γβ1 = (err_GPS**2)*np.eye(2)
        # DVL
        self.C2 = np.array([0., 0., 1.])
        self.Γβ2 = (err_DVL**2)*np.eye(1)
        # GPS + DVL
        self.C3 = np.eye(3)
        self.Γβ3 = np.array([[(err_GPS**2), 0., 0.], [0., (err_GPS)**2, 0.], [0., 0., (err_DVL)**2]])

#        self.flg_predict = 0
#        self.flg_GPS = 1
#        self.flg_DVL = 0

        self.dX = np.array([0., 0., 0.])
        self.r = r
        self.dt = dt
        self.epsilon = 2*dt

    def set_mission(self, lm):
        self.lm = lm
        self.len_lm = len(lm)
        sum_dist = 0
        for i in range(self.len_lm):
            sum_dist += np.linalg.norm(np.array(self.lm[(i+1)%self.len_lm][0])-np.array(self.lm[i][0]))
        self.r = int(sum_dist/dt)

    def apply_kalman_predict(self):
        return kalman_predict(self.X, self.Gup, self.u*self.dt, self.Γα*self.dt, self.A)

    def apply_kalman_GPS(self):
        y = self.X[:2]
        return kalman(self.X, self.Gup, self.u*self.dt, y, self.Γα*self.dt, self.Γβ1, self.A, self.C1)

    def apply_kalman_DVL(self):
        y = self.X[2]
        return kalman(self.X, self.Gup, self.u*self.dt, y, self.Γα*self.dt, self.Γβ2, self.A, self.C2)

    def apply_kalman_full(self):
        y = self.X
        return kalman(self.X, self.Gup, self.u*self.dt, y, self.Γα*self.dt, self.Γβ3, self.A, self.C3)

    def get_V_pt(self, i):
        pt, r = np.array(self.lm[i][0]), self.lm[i][1]
        pt_1, r_1 = np.array(self.lm[(i+1)%self.len_lm][0]), self.lm[(i+1)%self.len_lm][1]
        V = pt_1 - pt
        return V, pt, r, pt_1, r_1

    def loop(self):
        self.lX = []
        V, pt, r, pt_1, r_1 = self.get_V_pt(0)
        theta = sawtooth(arctan2(V[1], V[0]))
        self.A = np.array([[1., 0., self.dt*cos(theta)],
                           [0., 1., self.dt*sin(theta)],
                           [0., 0., 1.-self.dt]])
        j = 0
        flag = 0
        i_old = 0
        for i in range(self.r):
            if flag == 1 :    # Nouvelle île détectée
                flag =0
                j += 1
                i_old = i
                if j == self.len_lm:
                    break
                V, pt, r, pt_1, r_1 = self.get_V_pt(j)
                theta = arctan2(V[1], V[0])
                self.A = np.array([[1., 0., self.dt*cos(theta)],
                                   [0., 1., self.dt*sin(theta)],
                                   [0., 0., 1.-self.dt]])
            d = self.dist_min_amers(pt, r, pt_1, r_1)
            d =  (i-i_old) - np.linalg.norm(pt_1-pt)/self.dt
            if d>=0:         # UAV proche d'un point, remonte à la surface, passage en GPS
#               print("full\n")
                self.X, self.Gup = self.apply_kalman_GPS()
                flag =1
            else:         # UAV en haute mer, plonge, passage en kalman predictif
#               print("DVL\n")
                self.X, self.Gup = self.apply_kalman_predict()
            self.lX.append(np.array([self.X[0], self.X[1], theta]))
            try :
#               print("try")
                self.lGup = np.dstack((self.lGup, np.sqrt(abs(self.Gup))))
            except AttributeError :
#               print("except")
                self.lGup = np.sqrt(abs(self.Gup))
        self.plot_results()

    def dist_min_amers(self, pt, r, pt_1, r_1):
        return max(r - norm(self.X[:2]-pt),r_1 - norm(self.X[:2]-pt_1))

    def plot_results(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, aspect='equal')
        xmin,xmax,ymin,ymax = -1,1,-1,1
        for ile in self.lm:
            r = ile[1]
            xmin = min(xmin,ile[0][0]-r)
            xmax = max(xmax,ile[0][0]+r)
            ymin = min(ymin,ile[0][1]-r)
            ymax = max(ymax,ile[0][1]+r)
        ax.set_xlim(xmin,xmax)
        ax.set_ylim(ymin,ymax)
        for i in range(len(self.lX)):
            # draw_tank(self.lX[i],col='darkblue',r=0.1)
            if i%20 == 0 :
                    draw_ellipse(self.lX[i][:2].T,self.lGup[0,0,i], self.lGup[1,1,i],ax,"blue")
            # draw_ellipse(self.lX[i].T,self.lGup[0,0], self.lGup[1,1],ax,"blue")
        nlX = np.array(self.lX)
        plt.plot(nlX[:,0],nlX[:,1],'k')
        # print(self.lX[:][0][:][0])
        for ile in self.lm:
            draw_ellipse(ile[0],ile[1],ile[1],ax,"red","red")
        for i in range(3):
            plt.figure()
            plt.plot(self.lGup[i,i])


if __name__ == "__main__":
    X = np.array([0., 0., 1.])
    Γα = 0.01*np.eye(3)
    u = np.array([0., 0., 1.])
    r = 25000
    dt = 0.01

    # ax=init_figure(-1,6,-1,6)

    kl = kalman_localisation(X, Γα, u, r, dt)
    lm = [[[0.,0.], 1.], [[8.,1.], 1.], [[5.,5.], 1.]]
    print(kl.r)
    kl.set_mission(lm)
    print(kl.r)
    kl.loop()
    plt.show()
