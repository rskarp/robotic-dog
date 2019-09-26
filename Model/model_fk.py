'''
Based on:
Lab 1: Forward Kinematics
CS442 Spring 2019
Caitrin Eaton

Edited by:
Riley Karp
CS484 - Dog Leg Model: Forward Kinematics
17 February 2019
'''

import sys									# for command line arguments
from math import cos, sin, pi				# for trig functions (cos and sin)
import numpy as np							# for matrix math
import matplotlib.pyplot as plt				# for visualization
from mpl_toolkits.mplot3d import Axes3D 	# for 3D plotting

def visualizeArm( t0, z0=[], ax=None ):
	'''Draw a stick figure of the the arm in its current configuration.

	t0 is a numpy ndarray of shape (N, 4, 4), where N is the number of
	degrees of freedom. t0[i][:][:] is the transformation
	matrix describing the position and orientation of frame i in frame 0.

	Similarly, z0 is a numpy ndarray of shape (N, 4, 4), where N is the
	number of degrees of freedom. z0[i][:][:] is the transformation
	matrix describing the position and orientation of the end of the Zi
	axis in frame 0.

	All angles must be in radians and all distances must be in cm.'''

	# If no existing axis was given as a parameter, then create a new figure
	if ax == None:
		# Create a new figure and configure the axes for 3D plotting
		fig = plt.figure()
		ax = fig.add_subplot(111, aspect='equal', projection='3d')

		# Label the figure and axes (including units)
		ax.set_title("Dog Leg forward kinematics")
		ax.set_xlabel("X0 (in)")
		ax.set_ylabel("Y0 (in)")
		ax.set_zlabel("Z0 (in)")

		# Fix axis limits so that they're the same size regardless of
		# the configuration of the arm
		ax.set_xlim( [-8, 8] )
		ax.set_ylim( [-8, 8] )
		ax.set_zlim( [-8, 8] )

	# Draw the links of the arm
	for i in range( 1, t0.shape[0] ):
		ax.plot( [t0[i-1][0][3], t0[i][0][3]], [t0[i-1][1][3], t0[i][1][3]], [t0[i-1][2][3], t0[i][2][3]], 'o-', color='#B0B0B0', linewidth=5)

	if z0 is not None:
		# Draw the Z axis for each joint, if they were provided
		for i in range( t0.shape[0] ):
			ax.plot( [t0[i][0][3], z0[i][0][3]], [t0[i][1][3], z0[i][1][3]], [t0[i][2][3], z0[i][2][3]], 'o-', markersize=6, linewidth=1, label="z{0}".format(i+1))

	ax.legend(loc='center left')

	return (fig, ax)


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


def main( argv ):

	# Copy joint angles that were provided as command line arguments
	# into the table of DH parameters. Assumes angles are given in degrees
	theta0 = (float(argv[1]))*pi/180
	theta1 = (float(argv[2]))*pi/180
	theta2 = (float(argv[3]))*pi/180

	# The WAM arm's modified DH parameters [alpha(i-1), a(i-1), d(i), theta(i)]
	DH = [ [0,0,0,theta0],
		   [0,1.5,0,theta1],
		   [0,2.5,-0.75,theta2],
		   [0,2.75,-0.75,0] ]

	# Compute the forward kinematics of the arm, finding the orientation
	# and position of each joint's frame in the base frame (X0, Y0, Z0).
	t1 = transfromationMatrix( DH[0] ) # hip orientation to base frame
	t2 = t1*transfromationMatrix( DH[1] ) # hip to base frame
	t3 = t2*transfromationMatrix( DH[2] ) # knee to base frame
	t4 = t3*transfromationMatrix( DH[3] ) # ankle to base frame
	# t5 = t4*transfromationMatrix( DH[4] ) # toe to base frame

	# Print the joint coordinates (in the base frame) to the Terminal
	matrices = [t1,t2,t3,t4]
	# for i in range(len(matrices)):
	# 	coords = matrices[i][0:3,3].tolist()
	# 	print "(x,y,z) of joint %d: ( %.2f , %.2f , %.2f ) cm\n" % (i, coords[0][0], coords[1][0], coords[2][0])

	# Get t0 and z0 matrices
	t0 = np.array(matrices)
	z = np.matrix([ [1,0,0,0],
					 [0,1,0,0],
					 [0,0,1,2],
					 [0,0,0,1] ])
	z0 = np.array([z*t1,z*t2,z*t3,z*t4])

	# Draw the arm in its current configuration
	(fig, ax) = visualizeArm( t0, z0 )
	plt.show()


if __name__=="__main__":
	main( sys.argv )
