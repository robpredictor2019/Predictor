from  particule import Particule
import roblib


class mission:
	def __init__(self, num):
		self.listParticules = [Particule() for i in range(num)]
		self.t = 0
		self.dt = 1
		self.tfinal = 130
		self.num = num

	def __repr__(self):
		return "Programme de la mission: \n Aller en ligne droite, retour a 60s\n Nombre de particules {}".format(self.num)


	def run(self):
		while self.t < self.tfinal :
			for rob in self.listParticules: 
				rob.step(self.t, self.dt)
			self.t  += self.dt

