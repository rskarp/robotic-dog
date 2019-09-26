# Riley Karp
# CS484 - Honors Thesis
# Gait simulation
# 11 April 2019
# Bio simulation: strideSim2
# Waypoint method: strideSim3

import numpy as np                        # for matrix math
import sys                                # for command line arguments
import matplotlib.pyplot as plt				# for visualization
from math import cos, sin, pi				# for trig functions (cos and sin)
import math

def visualizeLeg( t0,ax=None ):
	if( ax == None ):
		# Create a new figure and configure the axes and subplots
		fig = plt.figure()
		ax = fig.add_subplot(111)

	ax.clear()
	ax.set_title("Forward Kinematics Stride Simulation")
	ax.set_xlabel("X (in)")
	ax.set_ylabel("Y (in)")

	ax.set_xlim( [-8, 8] )
	ax.set_ylim( [-8, 8] )
	for i in range( 1, t0.shape[0] ):
		ax.plot( [t0[i-1][0][3], t0[i][0][3]], [t0[i-1][1][3], t0[i][1][3]], 'o-', color='#B0B0B0', linewidth=5)
	print("( %.2f, %.2f )" % (t0[3][0][3], t0[3][1][3]))
	return ax


def transfromationMatrix( DHparams ):
	alpha = DHparams[0]
	a = DHparams[1]
	d = DHparams[2]
	theta = DHparams[3]
	matrix = np.matrix([ [cos(theta), -1*sin(theta), 0, a],
						 [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -1*sin(alpha), -1*d*sin(alpha)],
						 [sin(theta)*sin(alpha), cos(theta)*sin(alpha),cos(alpha),cos(alpha)*d],
						 [0,0,0,1] ])
	return matrix


def updateAxes( theta0, theta1, theta2, axs ):
	# Copy joint angles that were provided as command line arguments
	# into the table of DH parameters. Assumes angles are given in degrees
	theta0 = (float(theta0))*pi/180
	theta1 = (float(theta1))*pi/180
	theta2 = (float(theta2))*pi/180

	# Modified DH parameters [alpha(i-1), a(i-1), d(i), theta(i)]
	DH = [ [0,0,0,theta0],
		   [0,1.5,0,theta1],
		   [0,2.5,-0.75,theta2],
		   [0,2.75,-0.75,0] ]

	# DH = [ [0,0,0,theta0],
	# 	   [0,2,0,theta1],
	# 	   [0,2,-0.75,theta2],
	# 	   [0,2,-0.75,0] ]

	# Compute the forward kinematics of the arm, finding the orientation
	# and position of each joint's frame in the base frame (X0, Y0, Z0).
	t1 = transfromationMatrix( DH[0] ) # hip orientation to base frame
	t2 = t1*transfromationMatrix( DH[1] ) # hip to base frame
	t3 = t2*transfromationMatrix( DH[2] ) # knee to base frame
	t4 = t3*transfromationMatrix( DH[3] ) # ankle to base frame

	# Get t0 matrices
	t0 = np.array([t1,t2,t3,t4])

	# Draw the arm in its current configuration
	ax = visualizeLeg( t0, axs )
	plt.draw()
	return ax


def strideSim():
	hipFunc = getHip()
	kneeFunc = getKnee()
	ankleFunc = getAnkle()
	axis = None
	phase = 0.0
	while(True):
		if(phase>1.0):
			phase = 0.0
		O1 = hipFunc(phase) - 90
		O2 = kneeFunc(phase)
		O3 = ankleFunc(phase)

		print("phase: %.2f" % phase )
		print( "hip: %.2f, knee: %.2f, ankle: %.2f" % (O1,O2,O3))

		axis = updateAxes(O1,O2,O3,axis)
		# if(abs(phase-0.5)<0.05):
		# 	plt.show()

		plt.pause(0.2)
		phase += 0.05

# Bio simulation
def strideSim2():
	hipFunc = getHip()
	kneeFunc = getKnee()
	ankleFunc = getAnkle()
	axis = None
	phase = 0.0

	xs = []
	ys = []
	count = 0
	while(True):
		if count > 2:
			break
		O1 = hipFunc(phase) - 90
		O2 = kneeFunc(phase)
		O3 = ankleFunc(phase)

		print("phase: %.2f" % phase )
		print( "hip: %.2f, knee: %.2f, ankle: %.2f" % (O1,O2,O3))

		curX = 2.75*cos(math.radians(O1+O2+O3)) + 2.5*cos(math.radians(O1+O2)) + 1.5*cos(math.radians(O1))
		curY = 2.75*sin(math.radians(O1+O2+O3)) + 2.5*sin(math.radians(O1+O2)) + 1.5*sin(math.radians(O1))
		print("curX: %f" % curX)
		xs.append(curX)
		ys.append(curY)

		axis = updateAxes(O1,O2,O3,axis)
		plt.pause(0.025)
		if( phase < 0.3 ):
			phase += 0.01
		else:
			phase += 0.03
		if(phase>0.99):
			phase = 0.0
			count += 1

	fig = plt.figure()
	ax = fig.add_subplot(111)
	ax.set_title("Forward Kinematics Stride Simulation")
	ax.set_xlabel("X (in)")
	ax.set_ylabel("Y (in)")
	ax.scatter(xs,ys)
	plt.show()

# Waypoint method
def strideSim3():
	# hipAngles = [-107.53,-115.32, -120.90,-120.89,-122.7,-133.56,-132.03,-126.2,-117.23,-105,-79.74,-88.8,-104.19]
	# kneeAngles = [16.76,15.07,11.30,5.73,0.94,7.32,12.75,16.97,20,21.15,13.79,13.34,14.76]
	# ankleAngles = [31.44,35.70,34.52,23.72,15.29,44.96,53.55,55.09,51,41.14,12,16.81,31]
	hipAngles = [-7.53,25.0,0.0,-33.0,-25.0,-10.0,10.0,15.0]
	kneeAngles = [-36.76,-43.0,-45.0,-48.0,-30.0,-40.0,-35.0,-30.0]
	ankleAngles = [31.44,48,60,66,45,40,30,38]
	axis = None
	idx = 0
	print(len(hipAngles))
	xs = []
	ys = []
	count = 0
	while(True):
		if count > 2:
			break
		O1 = hipAngles[idx] -90
		O2 = kneeAngles[idx]
		O3 = float(ankleAngles[idx])

		curX = 2.75*cos(math.radians(O1+O2+O3)) + 2.5*cos(math.radians(O1+O2)) + 1.5*cos(math.radians(O1))
		curY = 2.75*sin(math.radians(O1+O2+O3)) + 2.5*sin(math.radians(O1+O2)) + 1.5*sin(math.radians(O1))
		if curX > -3.99:
			xs.append(curX)
			ys.append(curY)

		axis = updateAxes(O1,O2,O3,axis)
		plt.pause(0.02)

		idx += 1
		if( idx >= len(hipAngles)):
			idx = 0
			count +=1

	fig = plt.figure()
	ax = fig.add_subplot(111)
	ax.set_title("Waypoint Method Stride Simulation")
	ax.set_xlabel("X (in)")
	ax.set_ylabel("Y (in)")
	ax.scatter(xs,ys)
	print(xs)
	print(ys)
	plt.show()

def strideSim4():
	hipAngles = [-7.53,25.0,0.0,-33.0,-25.0,-10.0,10.0,15.0]
	kneeAngles = [-36.76,-43.0,-45.0,-48.0,-30.0,-40.0,-35.0,-30.0]
	ankleAngles = [31.44,48,60,66,45,40,30,38]
	axis = None
	idx = 1
	print(len(hipAngles))
	xs = []
	ys = []
	count = 0
	O1 = hipAngles[idx-1]-90
	O2 = kneeAngles[idx-1]
	O3 = float(ankleAngles[idx-1])
	while(True):
		if count > 2:
			break


		curX = 2.75*cos(math.radians(O1+O2+O3)) + 2.5*cos(math.radians(O1+O2)) + 1.5*cos(math.radians(O1))
		curY = 2.75*sin(math.radians(O1+O2+O3)) + 2.5*sin(math.radians(O1+O2)) + 1.5*sin(math.radians(O1))
		xs.append(curX)
		ys.append(curY)

		axis = updateAxes(O1,O2,O3,axis)
		plt.pause(0.02)

		O1 += (hipAngles[idx]-90 - O1)/10.0
		O2 += (kneeAngles[idx]-O2)/10.0
		O3 += (ankleAngles[idx]-O3)/10.0
		if( abs(O1-(hipAngles[idx]-90))< 0.1 ):
			idx += 1
			if( idx >= len(hipAngles)):
				idx = 1
				count +=1
			O1 = hipAngles[idx-1]-90
			O2 = kneeAngles[idx-1]
			O3 = float(ankleAngles[idx-1])

	fig = plt.figure()
	ax = fig.add_subplot(111)
	ax.set_title("Waypoint Method Stride Simulation")
	ax.set_xlabel("X (in)")
	ax.set_ylabel("Y (in)")
	ax.scatter(xs,ys)
	plt.show()

def getHip():
	# get list of stride duration %
	ph=[]
	for i in range(0,101,4):
		ph.append(i/100)
	# print(ph)
	phases = np.array(ph)

	# stride: ph[0:75],angles[0:13], stance: ph[75:],angles[13:]

	# hip data from beagle paper
	h_angles = np.array([-11,-9,-5,-3,-2,-1,0,5,8,10,14,16,18,19,16,14,7,0,-9,-12,-13,-14,-14,-13,-14,-11])
	hip = np.poly1d( np.polyfit(phases,h_angles,6) )
	print("hip: ",hip)
	return hip


def plotHip():
	hip = getHip()
	# plot hip function
	xs = np.linspace(0,1,100)
	plt.plot(phases,h_angles,'.',xs,hip(xs),'-')
	plt.xlabel("Stride Duration")
	plt.ylabel("Angle (degrees)")
	plt.title("Hip Angles During Stride")
	plt.ylim(-20,20)
	plt.show()


def getKnee():
	# get list of stride duration %
	ph=[]
	for i in range(0,101,4):
		ph.append(i/100)
	phases = np.array(ph)

	# knee data from beagle paper
	k_angles = np.array([13,9,7,5,3,1,1.5,2,4,6,9,10,10,6,-1,-15,-24,-33,-34,-28,-10,0,19,23,17,14])
	knee = np.poly1d( np.polyfit(phases,k_angles,6) )
	print("knee: ",knee)
	return knee


def plotKnee():
	knee = getKnee()
	# plot knee function
	xs = np.linspace(0,1,100)
	plt.plot(phases,k_angles,'.',xs,knee(xs),'-')
	plt.xlabel("Stride Duration")
	plt.ylabel("Angle (degrees)")
	plt.title("Knee Angles During Stride")
	plt.ylim(-50,30)
	plt.show()


def getAnkle():
	# get list of stride duration %
	ph=[]
	for i in range(0,101,4):
		ph.append(i/100)
	phases = np.array(ph)

	# ankle data from beagle paper
	a_angles = np.array([2,-9,-13,-15,-14.5,-14,-11,-2,2,16,21,27,31,30,22,11,0,-18,-25,-28,-22,-5,9,10,9,1])
	ankle = np.poly1d( np.polyfit(phases,a_angles,6) )
	print("ankle: ",ankle)
	return ankle


def plotAnkle():
	ankle = getAnkle()
	# plot knee function
	xs = np.linspace(0,1,100)
	plt.plot(phases,a_angles,'.',xs,ankle(xs),'-')
	plt.xlabel("Stride Duration")
	plt.ylabel("Angle (degrees)")
	plt.title("Ankle Angles During Stride")
	plt.ylim(-50,50)
	plt.show()

def plots():
	fig, axs = plt.subplots(3,1)

	axs[0].set_title("Hip Angles During Stride")
	axs[0].set_xlabel("Stride Duration")
	axs[0].set_ylabel("Angle (degrees)")
	axs[0].set_ylim( [-20, 20] )

	axs[1].set_title("Knee Angles During Stride")
	axs[1].set_xlabel("Stride Duration")
	axs[1].set_ylabel("Angle (degrees)")
	axs[1].set_ylim( [-50, 30] )

	axs[2].set_title("Ankle Angles During Stride")
	axs[2].set_xlabel("Stride Duration")
	axs[2].set_ylabel("Angle (degrees)")
	axs[2].set_ylim( [-50, 50] )

	plt.tight_layout()

	xs = np.linspace(0,1,100)
	ph=[]
	for i in range(0,101,4):
		ph.append(i/100)
	phases = np.array(ph)

	hip = getHip()
	knee = getKnee()
	ankle = getAnkle()

	print("hip: %.2f , %.2f" % (hip(0.2),hip(0.9)))
	print("knee: %.2f , %.2f" % (knee(0.2),knee(0.9)))
	print("ankle: %.2f , %.2f" % (ankle(0.2),ankle(0.9)))

	h_angles = np.array([-11,-9,-5,-3,-2,-1,0,5,8,10,14,16,18,19,16,14,7,0,-9,-12,-13,-14,-14,-13,-14,-11])
	k_angles = np.array([13,9,7,5,3,1,1.5,2,4,6,9,10,10,6,-1,-15,-24,-33,-34,-28,-10,0,19,23,17,14])
	a_angles = np.array([2,-9,-13,-15,-14.5,-14,-11,-2,2,16,21,27,31,30,22,11,0,-18,-25,-28,-22,-5,9,10,9,1])

	axs[0].plot(phases,h_angles,'.',xs,hip(xs),'-')
	axs[1].plot(phases,k_angles,'.',xs,knee(xs),'-')
	axs[2].plot(phases,a_angles,'.',xs,ankle(xs),'-')

	plt.show()


if __name__ == "__main__":
	# plotHip()
	# plotKnee()
	# plotAnkle()
	strideSim3()
	# plots()

	'''
		thursday:
		un-normalize angles. longer stance,shorter swing phase.
		Try splitting equations - one for stance, one for swing
		monday: one leg gait. full body.
		If swing is weird: use x = cos(t),y=sin(t) and inverse kinematics to get angles to make curved swing path

		monday 4/15:
		- dif polynomials for stance/swing (debug using simulation at low speed)
		- smooth between 0.2 and 0.9 eg sliding mean or sinusoid
		- other non-polynomail fits (sum of gaussians or sinusoids)
		- SCASM (state machine with flexion/extension for each joint). where speed is a function of phase

		Thurs 4/18: Comparing Joint Control Strategies
		-jacobian method for IK. too much for
		-biological data from beagles paper
			- put foot on ground at phase 0
			- draw line for ground
			- split polynomial into 2 pieces
			- SCASM
		-waypoint method. get angles from simulation. move through list (sliding average) in arduino
	'''
