# Riley Karp
# 17 February 2019

import numpy as np                        # for matrix math
import sys                                # for command line arguments
from math import cos, sin, pi				# for trig functions (cos and sin)

''' Input: 3 angles(in degrees) and target end effector velocities
Output: Jacobian, determinant, thetas, actual end velocities'''
def main( argv ):
	# Initialize joint angles and target forces
	O1 = (float(argv[1]) - 90)*pi/180
	O2 = (float(argv[2]))*pi/180
	O3 = (float(argv[3]))*pi/180
	Vx = float(argv[4])
	Vy = float(argv[5])
	# Vz = float(argv[6])

	# Get Jacobian
	J = np.matrix([ [-2.75*sin(O1+O2+O3) - 2.5*sin(O1+O2) - 1.5*sin(O1),
						-2.75*sin(O1+O2+O3) - 2.5*sin(O1+O2),
						-2.75*sin(O1+O2+O3)],
					[2.75*cos(O1+O2+O3) + 2.5*cos(O1+O2) + 1.5*cos(O1),
						2.75*cos(O1+O2+O3) + 2.5*cos(O1+O2),
						2.75*cos(O1+O2+O3)]])

	# Invert Jacobian
	# det = np.linalg.det(J)
	Jinv = np.linalg.pinv(J)
	# Jt = J.T
	# print "Jpinv:\n", Jinv.round(2)
	# print "Jt:\n", Jt.round(2)

	# Get torques using Inverse Kinematics: p = Jinv*V
	p = Jinv*(np.matrix([Vx,Vy]).T)

	# Get forces using Forward Kinematics: V = J*p
	V = J*p

	# Print 3x3 Jacobian, determinant, angles, end effector velocities
	print "3x3 Jacobian: \n" ,  J.round(2)
	# print "Determinant of Jacobian: %.2f" % det
	print "Joint angles: ( %.2f , %.2f , %.2f )" % (p[0,0], p[1,0], p[2,0])
	print "End effector velocities: ( %.2f , %.2f )" % (V[0,0], V[1,0])


if __name__=="__main__":
	main( sys.argv )
