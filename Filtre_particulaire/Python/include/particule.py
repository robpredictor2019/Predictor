# -*- coding: utf-8 -*-
"""
Created on Tue Apr 17 19:13:33 2018

@author: alexandre
"""

from roblib import *
from PyUnityVibes.UnityFigure import UnityFigure

############################################
class Particule:
    """
    Particule class
    """
    def __init__(self,X,U,cov):
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

        self.X = X
        self.U = U
        self.cov = cov
        self.auv = figure.create(UnityFigure.OBJECT_3D_SUBMARINE, 0, 0, 0, dimX=5, dimY=5, dimZ=5)



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


    def display_plt(self,col):
        """
        Allows to display the Particule object through matplotlib
        """
        X = self.X.flatten()
        U = self.U.flatten()
        draw_arrow(X[0],X[1],U[1],0.1,col)

    def appendFrame(self,anim): #PyUnityVibes
        anim.appendFrame(self.auv, x=self.X[0,0], y=0.0, z=self.X[1,0], rx=0, ry=0, rz=self.u[1,0])

    def controle(self):
        """
        Control equation of the AUV
        """
        return 0, 0

    def f(self):
        """
        State equation of the AUV

        alpha : bruit gaussien sur x,y et v
        """
        alpha = np.zeros((3,1))
        alpha[0] = np.random.randn(1,1)*self.cov[0,0]
        alpha[1] = np.random.randn(1,1)*self.cov[1,1]
        alpha[2] = np.random.randn(1,1)*self.cov[2,2]

        U = self.U.flatten()
        theta = U[1]
        A  = array([[0,0,cos(theta)],[0,0,sin(theta)],[0,0,-1]])
        return A.dot(self.X) + array([[0],[0],[U[0]]]) + alpha

if __name__ == "__main__":

    X = array([[1],[4],[2]])
    U = array([[3],[pi/4]])
    cov = eye(3)
    part = Particule(X,U,cov)
    figure()
    part.display_plt("blue")
    part2 = part
    part2.X = part.f()
    part2.display("red")
    print(part)
