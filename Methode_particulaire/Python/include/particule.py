# -*- coding: utf-8 -*-
"""
Created on Tue Apr 17 19:13:33 2018

@author: alexandre
"""

from roblib import *
import time
import math
from PyUnityVibes.UnityFigure import UnityFigure

def kalman_predict(xup,Gup,u,Γα,A):
    Γ1 = A @ Gup @ A.T + Γα
    x1 = A @ xup + u 
    return(x1,Γ1)    

def kalman_correc(x0,Γ0,y,Γβ,C):
    S = C @ Γ0 @ C.T + Γβ 
    if det(S) != 0 :
        K = Γ0 @ C.T @ inv(S)
    else : 
        K = zeros((3,2))
    ytilde = y - C @ x0        
    Gup = (eye(len(x0))-K @ C) @ Γ0 
    xup = x0+ K@ytilde    
    return(xup,Gup) 
    
def kalman(x0,Γ0,u,y,Γα,Γβ,A,C):
    xup,Gup = kalman_correc(x0,Γ0,y,Γβ,C)
    x1,Γ1=kalman_predict(xup,Gup,u,Γα,A)
    return(x1,Γ1)

############################################
class Particule:
    """
    Particule class
    """
    def __init__(self,X,U,cov, figure):
        """
        Constructor

        Parameters
        ----------
        X: state vector
            X[0]:x coordinate
            X[1]:y coordinate
            X[2]:v speed

        U: input vector
            U[0]:u speed input
            U[1]:theta heading input

        cov: matrix 3*3
            covariance matrix
        """

        self.Xchap = X
        self.X = X
        self.U = U
        self.cov = cov
        self.theta = 0
        self.auv = figure.create(UnityFigure.OBJECT_3D_SUBMARINE, 0, 0, 0, dimX=5, dimY=5, dimZ=5, color=UnityFigure.COLOR_YELLOW)
        self.auv.updateRotation(0,math.degrees(self.U[1,0]),0)
        time.sleep(0.1)


    def __str__(self):
        """
        Allows to print the Particule object
        """
        X = self.X.flatten()
        U = self.U.flatten()
        return 'X: \n x coordinate > {}\n y coordinate > {}\n speed > {}\n\n U: \n speed input > {}\n theta heading input > {}\n\n cov:\n {}'.format(self.X[0], self.X[1], self.X[2], self.U[0], self.U[1], self.cov)

    def __repr__(self):
        X = self.X.flatten()
        U = self.U.flatten()
        return "Vecteur etat : [{},{},{}]".format(self.X[0],self.X[1],self.X[2]) + "\n Matrice de covariance : {}".format(self.cov) + "\n Vecteur d'entree : [{},{}]".format(self.U[0],self.U[1])  # {:.2f} notation pour n'afficher que deux chiffres après la virgule

    def noise(self, variance):
        return np.random.normal(0,variance**2)

    def sign(self, a):
        if a > 0:
            return 1
        else:
            return -1

    def display(self,col):
        """
        Allows to display the Particule object through matplotlib
        """
        X = self.X.flatten()
        U = self.U.flatten()
        draw_arrow(X[0],X[1],U[1],0.1,col)

    def appendFrame(self,anim): #PyUnityVibes
        #print("U : ", math.degrees(self.U[1,0]), "th: ", math.degrees(self.theta))
        anim.appendFrame(self.auv, x=self.X[1,0], y=0.0, z=self.X[0,0], rx=0, ry=math.degrees(self.theta), rz=0)

    def controle(self,t,theta_target):
        """
        Control equation of the AUV
        """
        k = 1
        #print(">>>", theta_target, self.theta)
        self.U[1,0] = k*(theta_target-self.theta)

    def f(self):
        """
        State equation of the AUV

        alpha : bruit gaussien sur x,y et v
        """
        #print("u : ", self.U.flatten())
        theta = self.U[1,0]
        u = self.U[0,0]

        sigma_x, sigma_y,sigma_v = 0.1,0.1,0.15
        G_alpha = np.diag([sigma_x**2,sigma_y**2,sigma_v**2])

        alpha = np.zeros((3,1))
        alpha[0,0] = np.random.normal(0,sigma_x**2)
        alpha[1,0] = np.random.normal(0,sigma_y**2)
        alpha[2,0] = np.random.normal(0,sigma_v**2)

        A  = array([[0,0,cos(theta)],[0,0,sin(theta)],[0,0,-1]])
        return A.dot(self.X) + array([[0],[0],[u]]) + alpha


    def step(self,t,dt):
        if t == 60:
            C = array([[1,0,0],[0,1,0]])
            G_beta = diag([[0.45**2],[0.45**2]])
        else :
            C = zeros((2,3))
            G_beta = zeros((2,2))

        if t<60:
            theta_target = 0
        else:
            theta_target = np.pi

        sigma_x, sigma_y,sigma_v = 0.1,0.1,0.15
        G_alpha = np.diag([sigma_x**2,sigma_y**2,sigma_v**2])

        #print("X avant : [{},{},{}]".format(self.X[0,0], self.X[1,0], self.X[2,0]))
        self.X = self.X + dt*self.f()
        #print("X apres : [{},{},{}]".format(self.X[0,0], self.X[1,0], self.X[2,0]))
        
        U = self.U.flatten()
        
        A  = array([[1,0,cos(self.theta)],[0,1,sin(self.theta)],[0,0,-1]])
        self.Xchap,self.cov = kalman(self.X,self.cov,array([[0],[0],[U[0]]]),G_beta ,G_alpha,G_beta,A,C)
        self.controle(t, theta_target)







if __name__ == "__main__":

    X = array([[1],[10],[2]])
    Xchap = X
    U = array([[3],[pi/4]])
    cov = eye(3)
    theta = 0
    part = Particule(X,Xchap,U,cov,theta)
    print(part)
    figure()
    part2 = part
    part2.step(10,0.1)
    print(part2)
    part2.display("red")
    part2.step(12,0.1)
    part2.display("green")