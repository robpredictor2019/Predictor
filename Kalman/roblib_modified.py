import numpy as np
from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round
from numpy.linalg import inv, norm
import matplotlib.pyplot as plt
from matplotlib.pyplot import *

def kalman_predict(xup,Gup,u,Γα,A):
    Γ1 = A @ Gup @ A.T + Γα
    x1 = A @ xup + u    
    return(x1,Γ1)    

def kalman_correc(x0,Γ0,y,Γβ,C):
    S = C @ Γ0 @ C.T + Γβ        
    print("Γ0",Γ0,"\nC", C,"\nS", S)
    if S.shape == (1,1):
        K = (1./S)*Γ0 @ C.T 
        ytilde = y - C @ x0        
        Gup = (eye(len(x0))-K @ C) @ Γ0 
        xup = x0 + K*ytilde
    else :
        K = Γ0 @ C.T @ inv(S)
        ytilde = y - C @ x0        
        Gup = (eye(len(x0))-K @ C) @ Γ0 
        xup = x0 + K@ytilde
    return(xup,Gup) 
    
def kalman(x0,Γ0,u,y,Γα,Γβ,A,C):
    xup,Gup = kalman_correc(x0,Γ0,y,Γβ,C)
    x1,Γ1=kalman_predict(xup,Gup,u,Γα,A)
    print("Gup", Γ1)
    return(x1,Γ1)
    
def draw_tank(x,col='darkblue',r=1):
    x=x.flatten()
    M = r*array([[1,-1,0,0,-1,-1,0,0,-1,1,0,0,3,3,0], [-2,-2,-2,-1,-1,1,1,2,2,2,2,1,0.5,-0.5,-1]])
    M=move_motif(M,x[0],x[1],x[2])
    plot2D(M,col,2)
    
def move_motif(M,x,y,θ):
    M1=ones((1,len(M[1,:])))
    M2=vstack((M, M1))
    R = array([[cos(θ),-sin(θ),x], [sin(θ),cos(θ),y]])
    return(R @ M2)
    
def plot2D(M,col='black',w=1):
    plot(M[0, :], M[1, :], col, linewidth = w)  