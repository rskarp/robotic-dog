# Riley Karp
# CS484 - Honors Thesis
# Inverse kinematic model of dog leg
# 21 February 2019
# IK stride simulation

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
	if( ax == None ):
		# Create a new figure and configure the axes and subplots
		fig = plt.figure()
		ax = fig.add_subplot(111)

	ax.clear()
	ax.set_title("Inverse Kinematics Stride Simulation")
	ax.set_xlabel("X (in)")
	ax.set_ylabel("Y (in)")

	ax.set_xlim( [-8, 8] )
	ax.set_ylim( [-8, 8] )
	for i in range( 1, t0.shape[0] ):
		ax.plot( [t0[i-1][0][3], t0[i][0][3]], [t0[i-1][1][3], t0[i][1][3]], 'o-', color='#B0B0B0', linewidth=5)

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

def risingY( x ):
	return 0.75*x - 4.25

def fallingY( x ):
	return -0.75*x - 4.25

def groundY( x ):
	return -5.75

'''Input: 3 initial angles(in degrees) and target end effector location'''
def main( argv ):
	# Initialize numSteps
	numSteps = float(argv[1])

	# Initialize joint angles
	O1 = -pi/2
	O2 = 0
	O3 = 0
	ax = None # initialize axes

	# initialize initial positions
	initX = 2.75*cos(O1+O2+O3) + 2.5*cos(O1+O2) + 1.5*cos(O1) # from transformation matrix
	initY = 2.75*sin(O1+O2+O3) + 2.5*sin(O1+O2) + 1.5*sin(O1) # from transformation matrix

	curX = initX
	curY = initY
	endX = curX
	endY = curY
	curStep = 0
	print( "Initial (x,y): (%.2f,%.2f)\n" % (curX,curY) )

	ground = True
	rising = False
	falling = False
	alpha = 1
	# xs=[]
	# ys=[]
	# count = 0
	while( curStep < numSteps ):
		# print("COUNT" + str(count))
		# if(count > 15 ):
		# 	break
		# Update target location based on part of stride
		if( rising ):
			# count += 1
			endX += 0.25
			endY = risingY(endX)
			# if( endX < -1 ): # first half of rise
			# 	endY = ( alpha*risingY(endX) + (1-alpha)*groundY(endX) )/2
			# 	alpha = 1 - abs(0.5*endX + 0.5) # alpha increases 0.5 to 1
			# else: # second half of rise
			# 	endY = ( alpha*risingY(endX) + (1-alpha)*fallingY(-endX) )/2
				# alpha = -0.5*endX + 0.5 # alpha decreases 1 to 0.5
		if(falling):
			endX += 0.25
			endY = fallingY(endX)
			# if( endX < 1 ): # first half of fall
			# 	endY = ( alpha*fallingY(endX) + (1-alpha)*risingY(endX) )/2
			# 	alpha = 1 - 0.5*abs(endX-1) # alpha increases 0.5 to 1
			# else: # second half of fall
			# 	endY = ( alpha*fallingY(endX) + (1-alpha)*groudY(endX) )/2
			# 	alpha = -0.5*endX + 1.5 # alpha decreases 1 to 0.5
		if(ground):
			endX -= 0.25
			endY = groundY(endX)
			# if( endX > 0 ): # first half of ground
			# 	endY = ( alpha*groundY(endX) + (1-alpha)*fallingY(endX) )/2
			# 	alpha = 1 - 0.5*endX/2 # alpha increases 0.5 to 1
			# else: # second half of ground
			# 	endY = ( alpha*groundY(endX) + (1-alpha)*risingY(endX) )/2
			# 	alpha = 0.5*(endX+2)/2 # alpha decreases 1 to 0.5

		# Update part of stride
		if( rising and (endX > 0)): # end rising, go to falling
			endX = 0
			endY = -4.25
			rising = False
			ground = False
			falling = True
		if( falling and (endX > 1.75)): # end falling, go to ground
			endX = 1.75
			endY = -5.75
			rising = False
			ground = True
			falling = False
		if( ground and (endX < -1.5)): # end ground, go to rising
			endX = -1.5
			endY = -5.75
			rising = True
			ground = False
			falling = False

		# update step count
		stepDist = sqrt( (curX-initX)**2 + (curY-initY)**2 )
		if( stepDist < 0.01 ):
			curStep += 1

		improving = True
		minDist = 10000000000000
		curDist = sqrt( (curX-endX)**2 + (curY-endY)**2 )
		while( (curDist > 0.4) and improving ):
			print "Target (x,y): (%.2f,%.2f)" % (endX,endY)
			print "Rising: %r , Falling: %r , Ground: %r" % (rising,falling,ground)
			# Get Jacobian
			J = getJacobian(O1,O2,O3)

			# End effector velocity
			vx = (endX - curX)*0.1
			vy = (endY - curY)*0.1

			# Invert Jacobian
			Jinv = np.linalg.pinv(J)

			# Get angles using Inverse Kinematics: dt = Jinv*V
			dt = Jinv*(np.matrix([vx,vy]).T)

			dt1 = dt.tolist()[0][0]
			dt2 = dt.tolist()[1][0]
			dt3 = dt.tolist()[2][0]

			# print "dt before check: %.2f, %.2f, %.2f" % (degrees(dt1),degrees(dt2),degrees(dt3))

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

			# print "vx: %.2f, vy: %.2f" % (vx,vy)
			# print "dt after check: %.2f, %.2f, %.2f" % (degrees(dt1),degrees(dt2),degrees(dt3))

			# Increment Angles
			O1 += dt1
			O2 += dt2
			O3 += dt3

			# Update current position
			curX = 2.75*cos(O1+O2+O3) + 2.5*cos(O1+O2) + 1.5*cos(O1)
			curY = 2.75*sin(O1+O2+O3) + 2.5*sin(O1+O2) + 1.5*sin(O1)

			curDist = sqrt( (curX-endX)**2 + (curY-endY)**2 )

			# xs.append(curX)
			# ys.append(curY)
			print "new (x,y): (%.2f,%.2f)" % (curX,curY)
			print "distance to target location: %.3f\n" % curDist

			if( curDist <= minDist ):
				minDist = curDist
			else:
				improving = False

			# Draw new axes
			ax = updateAxes(O1,O2,O3,ax)

			plt.pause(0.01)
	plt.show() # keep the display window open after end position is reached

	# fig = plt.figure()
	# ax = fig.add_subplot(111)
	# ax.set_title("Inverse Kinematics Stride Simulation")
	# ax.set_xlabel("X (in)")
	# ax.set_ylabel("Y (in)")
	# ax.scatter(xs[50:],ys[50:])
	# plt.show()


if __name__=="__main__":
	main( sys.argv )
