from roblib import *    
import random
# from focus import draw_field1, draw_field2, draw_field3

def init_figure(background,size):
    fig = figure(0, figsize = (20,10))
    ax = fig.add_subplot(111, aspect='equal')
    ax.background = plt.imread(background)
    ax.size = size 
    ax.imshow(ax.background,extent=ax.size)
    ax.xmin=ax.size[0]
    ax.xmax=ax.size[1]
    ax.ymin=ax.size[2]
    ax.ymax=ax.size[3]
    clear(ax)
    return ax

def clear(ax):
    pause(0.001)
    cla()
    ax.imshow(ax.background,extent=ax.size)  
    ax.set_xlim(ax.xmin,ax.xmax)
    ax.set_ylim(ax.ymin,ax.ymax)

   
def f(x,u):
    theta=x[2,0]
    return array([[cos(theta)], [sin(theta)],[u]])


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
        draw_disk(array([[c1],[c2]]),r,ax,"red")        
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
        draw_disk(array([[c1],[c2]]),r,ax,"red")        
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
        draw_disk(array([[c1],[c2]]),r,ax,"red")        
    R=sqrt(VX**2+VY**2)
    quiver(Mx,My,VX/R,VY/R)
    return()

def field1(x):
    x=x.flatten()
    x1=x[0]
    x2=x[1]
    r = 20
    vx = -x1**3/(r**2) - x1*x2**2/(r**2) + x1 - x2 
    vy = -x2**3/(r**2) - x2*x1**2/(r**2) + x1 + x2 
    r0=sqrt(vx**2+vy**2)
    vx=vx/r0
    vy=vy/r0    
    for i in range(len(TC1)):
        c1=TC1[i]
        c2=TC2[i]
        r=Tr[i]
        u1=Tu1[i]
        u2=Tu2[i]  
        dx1=x1-c1
        dx2=x2-c2
        T=p1*tanh(dx1*u2-dx2*u1)
        vxi=u1-T*u2
        vyi=u2+T*u1 
        a=exp(-(dx1/r)**2-(dx2/r)**2)
        vx=vx+a*vxi
        vy=vy+a*vyi
    return array([[vx],[vy]])



def field2(x):
    x=x.flatten()
    x1=x[0]
    x2=x[1]
    # vx= -x2
    # vy= x1 
    vx = -x2 
    vy = x1  
    r0=sqrt(vx**2+vy**2)
    vx=vx/r0
    vy=vy/r0    
    for i in range(len(TC1)):
        c1=TC1[i]
        c2=TC2[i]
        r=Tr[i]
        u1=Tu1[i]
        u2=Tu2[i]  
        dx1=x1-c1
        dx2=x2-c2
        T=p1*tanh(dx1*u2-dx2*u1)
        vxi=u1-T*u2
        vyi=u2+T*u1 
        a=exp(-(dx1/r)**2-(dx2/r)**2)
        vx=vx+a*vxi
        vy=vy+a*vyi
    return array([[vx],[vy]])



def field3(x):
    x=x.flatten()
    x1=x[0]
    x2=x[1]
    vx= -x2
    vy= x1 
    r0=sqrt(vx**2+vy**2)
    vx=vx/r0
    vy=vy/r0    
    for i in range(len(TC1)):
        c1=TC1[i]
        c2=TC2[i]
        r=Tr[i]
        u1=Tu1[i]
        u2=Tu2[i]  
        dx1=x1-c1
        dx2=x2-c2
        T=p1*tanh(dx1*u2-dx2*u1)
        vxi=u1-T*u2
        vyi=u2+T*u1 
        a=exp(-(dx1/r)**2-(dx2/r)**2)
        vx=vx+a*vxi
        vy=vy+a*vyi
    return array([[vx],[vy]])

ech = 30
da=ech*0.07

# ### EXEMPLE 1
TC1 = [20,0,-20]
TC2 = [-10,20,-10]
Tr  = [5,5,5]
Tu1 = [1,-1,1]
Tu2 = [1,0,-1]
field = field1
draw_field = draw_field1
ax=init_figure("zone_field1.png",[-ech,ech,-ech,ech])

## EXEMPLE 2
# TC1 = [23,0,-23]
# TC2 = [0,23,0]
# Tr  = [5,5,5]
# Tu1 = [0,-1,0]
# Tu2 = [1,0,-1]
# field = field2
# draw_field = draw_field2
# ax=init_figure("zone_field2.png",[-ech,ech,-ech,ech])

### EXEMPLE 2
# TC1 = [13,8,-5,-15,-15,-0]
# TC2 = [-0,12,10, 0,-15,-15]
# Tr  = [5,2,4,3,4,4]
# Tu1 = list((np.roll(TC1,-1) -TC1)/10)
# Tu2 = list((np.roll(TC2,-1) -TC2)/10)
# field = field3
# draw_field = draw_field3
# ax=init_figure("zone_field3.png",[-ech,ech,-ech,ech])


p1=1.5 #focus
x = array([[22],[0],[1]])
dt   = 0.5
for t in arange(0,1000,dt):
    clear(ax)
    v2  = field(x)
    v2  = v2.flatten()
    w    = angle(v2)
    u   =  sawtooth(w-x[2])
    x    = x +dt*f(x,u)
    draw_field(-ech,ech,-ech,ech)
    draw_disk(x[0:2],0.5,ax,"white",α=0.9)
    plt.title(str(t))