from  particule import Particule
import roblib
import time

from PyUnityVibes.UnityFigure import UnityFigure

class mission:
	def __init__(self, num):
		self.listParticules = [Particule() for i in range(num)]
		self.t = 0
		self.dt = 1
		self.tfinal = 130
		self.num = num

		#Unity Display
		self.figure = UnityFigure(UnityFigure.FIGURE_3D, UnityFigure.SCENE_EMPTY)
		time.sleep(1)
		self.anim = self.figure.createAnimation(self.dt)
		time.sleep(1)
		
		for part in self.listParticules:
			self.anim.addObject(part.auv)
			time.sleep(0.1)

	def __repr__(self):
		return "Programme de la mission: \n Aller en ligne droite, retour a 60s\n Nombre de particules {}".format(self.num)

	def display(self):
		self.figure.animate(self.anim)

	def run(self):
		while self.t < self.tfinal :
			for part in self.listParticules: 
				part.step(self.t, self.dt)
				part.appendFrame(self.anim)
			self.t  += self.dt

		time.sleep(1)
		self.display()
