# -*- coding: utf-8 -*-
"""
Created on Thu Feb 21 10:19:09 2019

@author: Lucie
"""

import roblib
import numpy as np




class Particule:
    def __init__(self):
        self.x = np.zeros((3,1)) # vecteur d'état de l'AUV : avec x , y et v sa vitesse
        self.Mco = np.eye(3) # Matrice de covariance de l'AUV

    def __repr__(self):
        return "Vecteur etat :[{},{},{}]".format(self.x[0],self.x[1],self.x[2]) + "\n Matrice de covariance : {}".format(self.Mco)  # {:.2f} notation pour n'afficher que deux chiffres après la virgule

    def __str__(self):
        return "Vecteur etat :[{},{},{}]".format(self.x[0],self.x[1],self.x[2]) + "\n Matrice de covariance : {}".format(self.Mco)

    def controle(self):
        return 0, 0

    def f(x,theta,u):
        """ equation d'etat de l'AUV"""
        alpha = np.zeros((3,1))
        alpha[0] = np.random.randn(1,1)*x.Mco[0,0]
        alpha[1] = np.random.randn(1,1)*x.Mco[1,1]
        alpha[2] = np.random.randn(1,1)*x.Mco[2,2]

        x = x.flatten()
        A  = np.array([[0,0,np.cos(theta)],[0,0,np.sin(theta)],[0,0,-1]])
        return A @ x + np.array([[0,0,u]]).T + alpha

if __name__ == "__main__":
    p = Particule()
    print(p)
