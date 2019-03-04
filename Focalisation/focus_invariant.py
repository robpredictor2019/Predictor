from pyinvariant import *
import time
import numpy as np
from pprint import pprint
# from sympy.mpmath import *
from sympy import Symbol

bouee =True
c = 1
def field2():
	S1 = ""
	S2 = ""
	S1 = S1 + "(-x[1]+"+str(c)+"*x[0]/abs(x[0]))/sqrt(x[0]^2 + x[1]^2)"
	S2 = S2 + "(x[0] +"+str(c)+"*x[1]/abs(x[1]))/sqrt(x[0]^2 + x[1]^2)"
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



ech=17
da=ech*0.07
TC1 = [10,0,-10,0]
TC2 = [0,10, 0,-10]
Tr  = [5,5,5,5]
Tu1 = [0,-1,0,1]
Tu2 = [1,0,-1,0]
p1=1.5 #focus

# S = field2()

space = IntervalVector([[-ech,ech],[-ech,ech]])

# Create the Subpaving structure
subPaving = SmartSubPaving(space)

# Create the Domain
dom = Domain(subPaving, FULL_DOOR)
dom.set_border_path_in(False)
dom.set_border_path_out(False)


# Create the Dynamics
# f = Function("x[2]", "(-x[1],x[0])") 
# f = Function("x[2]", "(-(x[1]-5)/((x[1]-5)^2+1),(x[0]-5)/((x[0]-5)^2+1))")
# f = Function("x[2]", "(-(x[0]-5)/((x[0]-5)^2+1),-(x[1]-5)/((x[1]-5)^2+1))") 
# f = Function("x[2]", "(-x[1]/sqrt(x[0]^2 + x[1]^2)+exp(-((x[0]-10)/5)^2-((x[1]-0)/5)^2)*(0-(1.5*tanh((x[0]-10)*1-(x[1]-0)*0))*1),x[0]/sqrt(x[0]^2 + x[1]^2)+exp(-((x[0]-10)/5)^2-((x[1]-0)/5)^2)*(1-(1.5*tanh((x[0]-10)*1-(x[1]-0)*0))*0))")

# f = Function("x[2]", "(-x[1]/sqrt(x[0]^2 + x[1]^2)+exp(-((x[0]-10)/5)^2-((x[1]-0)/5)^2)*(0-(1.5*tanh((x[0]-10)*1-(x[1]-0)*0))*1)+exp(-((x[0]-0)/5)^2-((x[1]-10)/5)^2)*(-1-(1.5*tanh((x[0]-0)*0-(x[1]-10)*-1))*0)+exp(-((x[0]--10)/5)^2-((x[1]-0)/5)^2)*(0-(1.5*tanh((x[0]--10)*-1-(x[1]-0)*0))*-1), x[0]/sqrt(x[0]^2 + x[1]^2)+exp(-((x[0]-10)/5)^2-((x[1]-0)/5)^2)*(1-(1.5*tanh((x[0]-10)*1-(x[1]-0)*0))*0)+exp(-((x[0]-0)/5)^2-((x[1]-10)/5)^2)*(0-(1.5*tanh((x[0]-0)*0-(x[1]-10)*-1))*-1)+exp(-((x[0]--10)/5)^2-((x[1]-0)/5)^2)*(-1-(1.5*tanh((x[0]--10)*-1-(x[1]-0)*0))*0))") 

# f = Function("x[2]", "(-x[1]/sqrt(x[0]^2 + x[1]^2)+exp(-((x[0]-10)/5)^2-((x[1]-0)/5)^2)*(0-(1.5*tanh((x[0]-10)*1-(x[1]-0)*0))*1)+exp(-((x[0]-0)/5)^2-((x[1]-10)/5)^2)*(-1-(1.5*tanh((x[0]-0)*0-(x[1]-10)*-1))*0)+exp(-((x[0]--10)/5)^2-((x[1]-0)/5)^2)*(0-(1.5*tanh((x[0]--10)*-1-(x[1]-0)*0))*-1),x[0]/sqrt(x[0]^2 + x[1]^2)+exp(-((x[0]-10)/5)^2-((x[1]-0)/5)^2)*(1-(1.5*tanh((x[0]-10)*1-(x[1]-0)*0))*0)+exp(-((x[0]-0)/5)^2-((x[1]-10)/5)^2)*(0-(1.5*tanh((x[0]-0)*0-(x[1]-10)*-1))*-1)+exp(-((x[0]--10)/5)^2-((x[1]-0)/5)^2)*(-1-(1.5*tanh((x[0]--10)*-1-(x[1]-0)*0))*0))") 
f = Function("x[2]", field2()) 

dyn = DynamicsFunction(f, FWD_BWD)

# Create the Maze associated with the Domain and the dynamics
maze = Maze(dom, dyn)

# Contract the system
for i in range(12): # Number of bisections
	print(i)
	subPaving.bisect()
	maze.contract()
# pprint(dir(maze))
# Visualization
visu = VibesMaze(" Invariant", maze)
visu.setProperties(0,0,512,512)
visu.show()

# TC1 = [10,0,-10]
# TC2 = [0,10, 0]
# Tr  = [5,5,5]
# for i in range(len(TC1)):
# 	visu.drawCircle(TC1[i], TC2[i], Tr[i], "black[red]");
