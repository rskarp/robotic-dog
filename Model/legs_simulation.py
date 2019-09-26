# Riley Karp
# CS484 - Honors Thesis
# Inverse kinematic model of 4 dog legs
# 28 February 2019

import numpy as np                        # for matrix math
import sys                                # for command line arguments
from math import cos, sin, pi, sqrt, degrees, radians			# for trig functions (cos and sin)
import matplotlib.pyplot as plt				# for visualization
from mpl_toolkits.mplot3d import Axes3D 	# for 3D plotting

'''Draw leg with matplotlib'''
def visualizeArm( t0, z0=[], ax=None ):
	'''Draw a stick figure of the the arm in its current configuration.

	t0 is a numpy ndarray of shape (N, 5, 4), where N is the number of
	degrees of freedom. t0[i][:][:] is the transformation
	matrix describing the position and orientation of frame i in frame 0.

	Similarly, z0 is a numpy ndarray of shape (N, 5, 4), where N is the
	number of degrees of freedom. z0[i][:][:] is the transformation
	matrix describing the position and orientation of the end of the Zi
	axis in frame 0.

	All angles must be in radians and all distances must be in inches.'''

	# If no existing axis was given as a parameter, then create a new figure
	if ax == None:
		# Create a new figure and configure the axes for 3D plotting
		fig = plt.figure()
		ax = fig.add_subplot(111, aspect='equal', projection='3d')
		ax.view_init(-90,90) # show XY plane

	# Clear any existing axes
	ax.clear()

	# Label the figure and axes (including units)
	ax.set_title("Dog Leg Inverse Kinematics Simulation")
	ax.set_xlabel("X0 (cm)")
	ax.set_ylabel("Y0 (cm)")
	ax.set_zlabel("Z0 (cm)")

	# Fix axis limits so that they're the same size regardless of
	# the configuration of the arm
	ax.set_xlim( [-15, 8] )
	ax.set_ylim( [-8, 8] )
	ax.set_zlim( [-10, 10] )

	# Draw the links of the 4 legs
	for k in range(2): # front/back
		for j in range(2): # left/right
			for i in range( 1, t0.shape[0] ):
				ax.plot( [t0[i-1][0][3] -10*k, t0[i][0][3] -10*k], [t0[i-1][1][3], t0[i][1][3]], [t0[i-1][2][3] +j*5, t0[i][2][3] +j*5], 'o-', color='#B0B0B0', linewidth=5)

			if z0 is not None:
				# Draw the Z axis for each joint, if they were provided
				for i in range( t0.shape[0] ):
					ax.plot( [t0[i][0][3] -10*k, z0[i][0][3] -10*k], [t0[i][1][3], z0[i][1][3]], [t0[i][2][3] +j*5, z0[i][2][3] +j*5], 'o-', markersize=6, linewidth=1, label="z{0}".format(i+1))

	ax.legend(loc='center left')

	return ax


'''Calculate and return the jacobian matrix from the given joint angles'''
def getJacobian( O1, O2, O3 ):
	J = np.matrix([ [-2.75*sin(O1+O2+O3) - 2.5*sin(O1+O2) - 1.5*sin(O1),
						-2.75*sin(O1+O2+O3) - 2.5*sin(O1+O2),
						-2.75*sin(O1+O2+O3)],
					[2.75*cos(O1+O2+O3) + 2.5*cos(O1+O2) + 1.5*cos(O1),
						2.75*cos(O1+O2+O3) + 2.5*cos(O1+O2),
						2.75*cos(O1+O2+O3)]])
	return J


'''Returns the transformation matrix based on the given row of DH parameters for a joint.
DHparams is a list in the format: [alpha, a, d, theta]'''
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


'''Redraw leg in new position based on new joint angles'''
def updateAxes( theta1, theta2, theta3, ax ):
	# Update frame
	DH = [ [0,0,0,theta1],
		   [0,1.5,0,theta2],
		   [0,2.5,-0.75,theta3],
		   [0,2.75,-0.75,0] ]

	# Compute the forward kinematics of the arm, finding the orientation
	# and position of each joint's frame in the base frame (X0, Y0, Z0).
	t1 = transfromationMatrix( DH[0] ) # hip to base frame
	t2 = t1*transfromationMatrix( DH[1] ) # knee to base frame
	t3 = t2*transfromationMatrix( DH[2] ) # ankle to base frame
	t4 = t3*transfromationMatrix( DH[3] ) # toe to base frame

	# Get t0 and z0 matrices
	matrices = [t1,t2,t3,t4]
	t0 = np.array(matrices)
	z = np.matrix([ [1,0,0,0],
					 [0,1,0,0],
					 [0,0,1,5],
					 [0,0,0,1] ])
	z0 = np.array([z*t1,z*t2,z*t3,z*t4])

	curax = visualizeArm( t0, z0, ax )
	plt.draw()

	return curax


'''Input: 3 initial angles(in degrees) and target end effector location'''
def main( argv ):
	# Initialize joint angles and target & initial positions
	O1 = (float(argv[1]))*pi/180
	O2 = (float(argv[2]))*pi/180
	O3 = (float(argv[3]))*pi/180
	# endX = float(argv[4])
	# endY = float(argv[5])
	ax = None # initialize axes

	curX = 2.75*cos(O1+O2+O3) + 2.5*cos(O1+O2) + 1.5*cos(O1) # from transformation matrix
	curY = 2.75*sin(O1+O2+O3) + 2.5*sin(O1+O2) + 1.5*sin(O1) # from transformation matrix

	print "Initial (x,y): (%.2f,%.2f)\n" % (curX,curY)

	for pos in [(0,-6.75),(2,-5),(2.5,-5.5),(0,-6.75),(-1,-5.75)]:
		endX = pos[0]
		endY = pos[1]
		print "Target (x,y): (%.2f,%.2f)\n" % (endX,endY)
		minDist = 10000000000000
		curDist = sqrt( (curX-endX)**2 + (curY-endY)**2 )
		while( curDist > 0.25 ):
			# Get Jacobian
			J = getJacobian(O1,O2,O3)

			# End effector velocity
			vx = (endX - curX)*0.05
			vy = (endY - curY)*0.05

			# Invert Jacobian
			Jinv = np.linalg.pinv(J)

			# Get angles using Inverse Kinematics: dt = Jinv*V
			dt = Jinv*(np.matrix([vx,vy]).T)

			dt1 = dt.tolist()[0][0]
			dt2 = dt.tolist()[1][0]
			dt3 = dt.tolist()[2][0]

			print "dt before check: %.2f, %.2f, %.2f" % (degrees(dt1),degrees(dt2),degrees(dt3))

			# Make sure angle doesn't change by more than 5 degrees
			angleThresh = radians(5)
			if( dt1 > angleThresh ):
				dt1 = angleThresh
			if( dt1 < -angleThresh ):
				dt1 = -angleThresh
			if( dt2 > angleThresh ):
				dt2 = angleThresh
			if( dt2 < -angleThresh ):
				dt2 = -angleThresh
			if( dt3 > angleThresh ):
				dt3 = angleThresh
			if( dt3 < -angleThresh ):
				dt3 = -angleThresh

			print "vx: %.2f, vy: %.2f" % (vx,vy)
			print "dt after check: %.2f, %.2f, %.2f" % (degrees(dt1),degrees(dt2),degrees(dt3))

			# Increment Angles
			O1 += dt1
			O2 += dt2
			O3 += dt3

			# Update current position
			curX = 2.75*cos(O1+O2+O3) + 2.5*cos(O1+O2) + 1.5*cos(O1)
			curY = 2.75*sin(O1+O2+O3) + 2.5*sin(O1+O2) + 1.5*sin(O1)

			curDist = sqrt( (curX-endX)**2 + (curY-endY)**2 )

			print "new (x,y): (%.2f,%.2f)" % (curX,curY)
			print "distance to target location: %.3f\n" % curDist

			if( curDist <= minDist ):
				minDist = curDist
			else:
				break

			# Draw new axes
			ax = updateAxes(O1,O2,O3,ax)

			plt.pause(0.05)
	plt.show() # keep the display window open after end position is reached


if __name__=="__main__":
	main( sys.argv )

	# alpha = pi for left side of body
	'''
	email tim
	Gait: hip move flat-ish/stable forward, foot trajectory, or forces
	Polygon of stability has to contain COM. triangle when moving one leg, rectangle w all 4 on ground
	'''
