#! /usr/bin/env python

#import rospy
import math
import numpy as np
import matplotlib.pyplot as plt

#rospy.init_node('controller')

class car:
	"""
	States = [x, y, yaw, v]
	Parameters = [dt, l, cx, cy]
	"""
	def __init__(self, x=0.0, y=0.0, yaw=0.0, v = 5, dt = 0.1, l = 7):
		self.x = x
		self.y = y
		self.yaw = yaw
		self.v = v
		self.dt = dt
		self.l = l
		self.cx, self.cy = self.generate_course()
		self.lf = 1

	def generate_course(self):
		cx = np.arange(0,50,0.1)
		cy = [math.sin(ix)/2*ix for ix in cx]
		return cx, cy

	def apply_control(self, a, delta):
		"""
		Simple kinematic model
		"""
		self.x = self.x + self.v*math.cos(self.yaw)*self.dt
		self.y = self.y + self.v*math.sin(self.yaw)*self.dt
		self.yaw = self.yaw + self.v/self.l*math.tan(delta)*self.dt
		self.v = self.v + a*self.dt


	def calc_target_index(self):

	    # search nearest point index
	    dx = [self.x - icx for icx in self.cx]
	    dy = [self.y - icy for icy in self.cy]
	    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
	    ind = d.index(min(d))
	    L = 0.0
	    k = 0.1
	    Lf = k * self.v + self.lf

	    # search look ahead target point index
	    while Lf > L and (ind + 1) < len(self.cx):
	        dx = self.cx[ind + 1] - self.cx[ind]
	        dy = self.cx[ind + 1] - self.cx[ind]
	        L += math.sqrt(dx ** 2 + dy ** 2)
	        ind += 1
	    return ind

	def control(self):
		ind = self.calc_target_index()
		D = math.sqrt((self.cx[ind]-self.x)**2 + (self.cy[ind]-self.y)**2)
		x = self.cx[ind] - self.x
		delta = math.atan()
	
	def pure_pursuit_control(self, pind):

	    ind = self.calc_target_index()

	    if pind >= ind:
	        ind = pind

	    if ind < len(self.cx):
	        tx = self.cx[ind]
	        ty = self.cy[ind]
	    else:
	        tx = self.cx[-1]
	        ty = self.cy[-1]
	        ind = len(self.cx) - 1

	    alpha = math.atan2(ty - self.y, tx - self.x) - self.yaw

	    if self.v < 0:  # back
	        alpha = math.pi - alpha
	    k = 0.1

	    Lf = k * self.v + self.lf

	    delta = math.atan2(2.0 * self.l * math.sin(alpha) / self.lf, 1.0)

	    return delta, ind
		



def animate():
	a = car()
	sim_time = 1000
	plt.plot(a.cx, a.cy, '-r')
	for i in range(sim_time):
		ind = a.calc_target_index()
		control_signal,_ = a.pure_pursuit_control(ind)
		a.apply_control(0, control_signal)
		plt.plot(a.x,a.y, '.b')
		plt.pause(0.01)
	plt.show()




if __name__ == '__main__':
	animate()


