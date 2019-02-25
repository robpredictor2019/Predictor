from  particule import Particule
import roblib
import time
import numpy as np
import sys

from PyUnityVibes.UnityFigure import UnityFigure

class mission:
	def __init__(self, num):
		#Unity Display
		self.figure = UnityFigure(UnityFigure.FIGURE_3D, UnityFigure.SCENE_EMPTY)
		time.sleep(1)

		self.listParticules = [Particule(np.array([[0],[0],[0]]), np.array([[1],[0]]), np.diag((10**-9,10**-9,10**-9)), self.figure ) for i in range(num)]
		self.t = 0
		self.dt = 0.1
		self.tfinal = 130
		self.num = num

		self.anim = self.figure.createAnimation(self.dt)
		time.sleep(1)
	
		print("Ajout des objets à l'animation")
		for part in self.listParticules:
			self.anim.addObject(part.auv)
			time.sleep(1)

	def __repr__(self):
		return "Programme de la mission: \n Aller en ligne droite, retour a 60s\n Nombre de particules {}".format(self.num)

	def display(self):
		print("Affichage de la mission sur PyUnityVibes")
		self.figure.animate(self.anim)

	def run(self):
		while self.t < self.tfinal :
			
			sys.stdout.write("t = %f \r" % self.t)
			for part in self.listParticules: 
				part.step(self.t, self.dt)
				part.appendFrame(self.anim)
			self.t  += self.dt

		print("\n Done ! ")
		time.sleep(1)
		self.display()


if __name__=='__main__':
	N = 10
	mission = mission(N)
	mission.run()