from roblib import *    

def draw_field(xmin,xmax,ymin,ymax,d):
	r = 10
	Mx    = arange(xmin,xmax,da)
	My    = arange(ymin,ymax,da)
	X1,X2 = meshgrid(Mx,My)
	VX= -X1**3/(r**2) - X1*X2**2/(r**2) + X1 - X2
	VY= -X2**3/(r**2) - X2*X1**2/(r**2) + X1 + X2
	R=sqrt(VX**2+VY**2)
	quiver(Mx,My,VX/R,VY/R)
	return()

def draw_field1(xmin,xmax,ymin,ymax):
    Mx    = arange(xmin,xmax,da)
    My    = arange(ymin,ymax,da)
    X1,X2 = meshgrid(Mx,My)
    # VX=   X1*0.001
    # VY= X2*0.001
    r = 20
    VX= -X1**3/(r**2) - X1*X2**2/(r**2) + X1 - X2    
    VY= -X2**3/(r**2) - X2*X1**2/(r**2) + X1 + X2
    R0=sqrt(VX**2+VY**2)
    VX=VX/R0
    VY=VY/R0    
    for i in range(len(TC1)):
        c1=TC1[i]
        c2=TC2[i]
        r=Tr[i]
        u1=Tu1[i]
        u2=Tu2[i]  
        dX1=X1-c1
        dX2=X2-c2
        T=p1*tanh(dX1*u2-dX2*u1)
        VXi=u1-T*u2
        VYi=u2+T*u1 
        A=exp(-(dX1/r)**2-(dX2/r)**2)
        VX=VX+A*VXi
        VY=VY+A*VYi
        draw_disk(array([[c1],[c2]]),r,ax,"blue")        
    R=sqrt(VX**2+VY**2)
    quiver(Mx,My,VX/R,VY/R)
    return()


def draw_field2(xmin,xmax,ymin,ymax):
    Mx    = arange(xmin,xmax,da)
    My    = arange(ymin,ymax,da)
    X1,X2 = meshgrid(Mx,My)
    VX= -X2
    VY= X1
    R0=sqrt(X1**2+X2**2)
    VX=VX/R0
    VY=VY/R0    
    for i in range(len(TC1)):
        c1=TC1[i]
        c2=TC2[i]
        r=Tr[i]
        u1=Tu1[i]
        u2=Tu2[i]  
        dX1=X1-c1
        dX2=X2-c2
        T=p1*tanh(dX1*u2-dX2*u1)
        VXi=u1-T*u2
        VYi=u2+T*u1 
        A=exp(-(dX1/r)**2-(dX2/r)**2)
        VX=VX+A*VXi
        VY=VY+A*VYi
        draw_disk(array([[c1],[c2]]),r,ax,"blue")        
    R=sqrt(VX**2+VY**2)
    quiver(Mx,My,VX/R,VY/R)
    return()


def draw_field3(xmin,xmax,ymin,ymax):
    Mx    = arange(xmin,xmax,da)
    My    = arange(ymin,ymax,da)
    X1,X2 = meshgrid(Mx,My)
    VX= X1*0.001
    VY= X2*0.001
    R0=sqrt(X1**2+X2**2)
    VX=VX/R0
    VY=VY/R0    
    for i in range(len(TC1)):
        c1=TC1[i]
        c2=TC2[i]
        r=Tr[i]
        u1=Tu1[i]
        u2=Tu2[i]  
        dX1=X1-c1
        dX2=X2-c2
        T=p1*tanh(dX1*u2-dX2*u1)
        VXi=u1-T*u2
        VYi=u2+T*u1 
        A=exp(-(dX1/r)**2-(dX2/r)**2)
        VX=VX+A*VXi
        VY=VY+A*VYi
        draw_disk(array([[c1],[c2]]),r,ax,"blue")        
    R=sqrt(VX**2+VY**2)
    quiver(Mx,My,VX/R,VY/R)
    return()


ech=30
da=ech*0.07
p1=1.5 #focus

ax=init_figure(-ech,ech,-ech,ech)

### EXEMPLE 1
TC1 = [20,0,-20]
TC2 = [-10,20,-10]
Tr  = [5,5,5]
Tu1 = [1,-1,1]
Tu2 = [1,0,-1]
draw_field1(-ech,ech,-ech,ech)
plt.show()

## EXEMPLE 2
# TC1 = [10,0,-10]
# TC2 = [0,10,0]
# Tr  = [5,5,5]
# Tu1 = [0,-1,0]
# Tu2 = [1,0,-1]
# draw_field2(-ech,ech,-ech,ech)
# plt.show()

### EXEMPLE 3
# TC1 = [13,8,-5,-15,-15,-0]
# TC2 = [-0,12,10, 0,-15,-15]
# Tr  = [5,2,4,3,4,4]
# Tu1 = list((np.roll(TC1,-1) -TC1)/10)
# Tu2 = list((np.roll(TC2,-1) -TC2)/10)
# draw_field3(-ech,ech,-ech,ech)
# plt.show()
