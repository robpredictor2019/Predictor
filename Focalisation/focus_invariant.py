from pyibex import *
from pyinvariant import *

from roblib import *  
import numpy as np



bouee =True
def field1():
	R = 20
	S1 = "(-x[0]^3/(" + str(R**2) + ") - x[0]*x[1]^2/(" + str(R**2) + ") + x[0] - x[1])/(sqrt((-x[0]^3/(" + str(R**2) + ") - x[0]*x[1]^2/(" + str(R**2) + ") + x[0] - x[1])^2 + (-x[1]^3/(" + str(R**2) + ") - x[1]*x[0]^2/(" + str(R**2) + ") + x[0] + x[1])^2))"
	S2 = "(-x[1]^3/(" + str(R**2) + ") - x[1]*x[0]^2/(" + str(R**2) + ") + x[0] + x[1])/(sqrt((-x[0]^3/(" + str(R**2) + ") - x[0]*x[1]^2/(" + str(R**2) + ") + x[0] - x[1])^2 + (-x[1]^3/(" + str(R**2) + ") - x[1]*x[0]^2/(" + str(R**2) + ") + x[0] + x[1])^2))"
	if bouee:   
		for i in range(len(TC1)):
			c1=TC1[i]
			c2=TC2[i]
			r=Tr[i]
			u1=Tu1[i]
			u2=Tu2[i]
			DX1 = "(x[0]-"+str(c1)+")"
			DX2 = "(x[1]-"+str(c2)+")"
			TT = "(" + str(p1) + "*tanh(" + DX1 +"*"+str(u2) + "-" + DX2 +"*" + str(u1) + "))"
			VXI = "(" + str(u1) +"-" + TT +"*"+str(u2) +")"
			VYI=  "(" + str(u2) +"-" + TT +"*"+str(u1)+")"
			A = "exp(-(" + DX1 +"/"+str(r) +")^2"+ "-(" + DX2 +"/"+str(r) +")^2)"
			S1 = S1 +"+"+ A + "*" +VXI
			S2 = S2 + "+" + A + "*" +VYI
	S =  "(" + S1 +"," +S2 +")" 
	return S

def field2():
	S1 = "(-x[1])/sqrt(x[0]^2 + x[1]^2)"
	S2 = "(x[0])/sqrt(x[0]^2 + x[1]^2)"	
	if bouee:   
		for i in range(len(TC1)):
			c1=TC1[i]
			c2=TC2[i]
			r=Tr[i]
			u1=Tu1[i]
			u2=Tu2[i]
			DX1 = "(x[0]-"+str(c1)+")"
			DX2 = "(x[1]-"+str(c2)+")"
			TT = "(" + str(p1) + "*tanh(" + DX1 +"*"+str(u2) + "-" + DX2 +"*" + str(u1) + "))"
			VXI = "(" + str(u1) +"-" + TT +"*"+str(u2) +")"
			VYI=  "(" + str(u2) +"-" + TT +"*"+str(u1)+")"
			A = "exp(-(" + DX1 +"/"+str(r) +")^2"+ "-(" + DX2 +"/"+str(r) +")^2)"
			S1 = S1 +"+"+ A + "*" +VXI
			S2 = S2 + "+" + A + "*" +VYI
	S =  "(" + S1 +"," +S2 +")" 
	return S

def field3():
	S1 = "(x[0]*0.001)/sqrt(x[0]^2 + x[1]^2)"
	S2 = "(x[1]*0.001)/sqrt(x[0]^2 + x[1]^2)"	
	if bouee:   
		for i in range(len(TC1)):
			c1=TC1[i]
			c2=TC2[i]
			r=Tr[i]
			u1=Tu1[i]
			u2=Tu2[i]
			DX1 = "(x[0]-"+str(c1)+")"
			DX2 = "(x[1]-"+str(c2)+")"
			TT = "(" + str(p1) + "*tanh(" + DX1 +"*"+str(u2) + "-" + DX2 +"*" + str(u1) + "))"
			VXI = "(" + str(u1) +"-" + TT +"*"+str(u2) +")"
			VYI=  "(" + str(u2) +"-" + TT +"*"+str(u1)+")"
			A = "exp(-(" + DX1 +"/"+str(r) +")^2"+ "-(" + DX2 +"/"+str(r) +")^2)"
			S1 = S1 +"+"+ A + "*" +VXI
			S2 = S2 + "+" + A + "*" +VYI
	S =  "(" + S1 +"," +S2 +")" 
	return S

ech=30
da=ech*0.07
p1=1.5 #focus

### EXEMPLE 1
TC1 = [20,0,-20]
TC2 = [-10,20,-10]
Tr  = [5,5,5]
Tu1 = [1,-1,1]
Tu2 = [1,0,-1]
f = Function("x[2]",field1())
f_sep = Function("x[2]", "(x[0])^2+(x[1])^2-(1.0)^2")

# ### EXEMPLE 2
# TC1 = [23,0,-23]
# TC2 = [0,23,0]
# Tr  = [5,5,5]
# Tu1 = [0,-1,0]
# Tu2 = [1,0,-1]
# f = Function("x[2]",field2())
# f_sep = Function("x[2]", "(x[0])^2+(x[1])^2-(15.0)^2")

### EXEMPLE 3
# TC1 = [13,8,-5,-15,-15,-0]
# TC2 = [-0,12,10, 0,-15,-15]
# Tr  = [5,2,4,3,4,4]
# Tu1 = list((np.roll(TC1,-1) -TC1)/10)
# Tu2 = list((np.roll(TC2,-1) -TC2)/10)
# f = Function("x[2]",field3())
# f_sep = Function("x[2]", "(x[0])^2+(x[1])^2-(8.0)^2")


# Define the search space
space = IntervalVector([[-ech, ech],[-ech,ech]])

# Create the grpah structure
smartSubPaving = SmartSubPaving(space)

# Create the Domain
dom = Domain(smartSubPaving, FULL_DOOR)
dom.set_border_path_in(False)
dom.set_border_path_out(False)

s = SepFwdBwd(f_sep, GEQ) # possible options : LT, LEQ, EQ, GEQ, GT
dom.set_sep(s); # Reduce the domain with a separator


# Create the Dynamics
dyn = DynamicsFunction(f, FWD_BWD)

# Create the Maze associated with the Domain and the dynamics
maze = Maze(dom, dyn)

# Contract the system
for i in range(15):
	print(i)
	smartSubPaving.bisect()
	maze.contract()

# Visualization
visu = VibesMaze("Invariant_Ile", maze)
visu.setProperties(0,0,512,512)
visu.show()


for i in range(len(TC1)):
	visu.drawCircle(TC1[i], TC2[i], Tr[i], "black[red]");
