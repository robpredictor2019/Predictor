import time
import sys

import numpy as np

from PyUnityVibes.UnityFigure import UnityFigure

### Constante ###
N  = 30
dt = 0.1

if len(sys.argv) < 2:
	print("Merci de rentrer le chemin d'accès au fichier à traiter")
	sys.exit(1)
else:
	filename = sys.argv[1]

file = open(filename, "r")
heading = file.readline()
data = file.readlines()


N, T, dt = heading.split(";")
N = int(N[2:])    # on lit "N=100"
T = int(T[2:])    # ------ "T=1200"
dt= float(dt[3:]) # ------ "dt=0.1"

parts = np.zeros((N, int(T/dt), 6))

for line in data:
	data_line = line.split(";")
	ID, t, x, y, z, rx, ry, rz = [float(el) for el in data_line]
	ID = int(ID)
	parts[ID, int(T/dt)-1] = x, y, z, rx, ry , rz

### Initialisation de Unity ###
figure = UnityFigure(UnityFigure.FIGURE_3D, UnityFigure.SCENE_EMPTY)
time.sleep(1)
anim = figure.createAnimation(dt)
time.sleep(1)


### Creation des AUVs ###
AUVs = []
for ind_auv in range(N):
	AUVs.append(figure.create(UnityFigure.OBJECT_3D_SUBMARINE, 0, -0.4, 0, dimX=5, dimY=5, dimZ=5))
	anim.addObject(AUVs[ind_auv])
	time.sleep(0.1)
time.sleep(1)

### Calcul des frames successives ###
for t in np.arange(0, T, dt):
	for ind_auv, auv in enumerate(AUVs):
		x, y, z, rx, ry, rz = parts[ind_auv, int(t/dt)]
		anim.appendFrame(auv, x=x, y=y, z=z, rx=rx, ry=ry, rz=rz)
time.sleep(1)


### Display ###
figure.animate(anim)

sys.exit(0)
