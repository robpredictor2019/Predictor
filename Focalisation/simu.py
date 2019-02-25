from roblib import *    


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


def field2(x):
    x=x.flatten()
    x1=x[0]
    x2=x[1]
    vx= -x2
    vy= x1 
    r0=sqrt(x1**2+x2**2)
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



ech = 17
da=ech*0.07
TC1 = [10,0,-10]
TC2 = [0,10, 0]
Tr  = [5,5,5]
Tu1 = [0,-1,0]
Tu2 = [1,0,-1]
p1=1.5 #focus

ax=init_figure("zone_pyinvariant.png",[-ech,ech,-ech,ech])
draw_disk(x[0:2],0.5,ax,"white",α=0.9)

x=array([[0],[-7],[1]])
dt   = 0.5
for t in arange(0,1000,dt):
    clear(ax)
    v2  = field2(x)
    v2  = v2.flatten()
    w    = angle(v2)
    u   =  sawtooth(w-x[2])
    x    = x +dt*f(x,u)
    # draw_field2(-ech,ech,-ech,ech)
    # draw_tank(x,'blue',1,2)
    draw_disk(x[0:2],0.5,ax,"white",α=0.9)