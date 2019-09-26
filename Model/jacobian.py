# Riley Karp
# Inverse kinematics
# 17 February 2019

import sys									# for command line arguments
from math import cos, sin, pi					# for trig functions (cos and sin)
import numpy as np							# for matrix math
from sympy import diff, symbols
import sympy as sym							# for Jacobian in inverse kinematics

'''Returns the transformation matrix based on the given row of DH parameters for a joint.
DHparams is a list in the format: [alpha, a, d, theta]'''
def transfromationMatrix( DHparams ):
	alpha = DHparams[0]
	a = DHparams[1]
	d = DHparams[2]
	theta = DHparams[3]
	matrix = sym.Matrix([ [sym.cos(theta), -1*sym.sin(theta), 0, a],
						 [sym.sin(theta)*sym.cos(alpha), sym.cos(theta)*sym.cos(alpha), -1*sym.sin(alpha), -1*d*sym.sin(alpha)],
						 [sym.sin(theta)*sym.sin(alpha), sym.cos(theta)*sym.sin(alpha),sym.cos(alpha),sym.cos(alpha)*d],
						 [0,0,0,1] ])
	return matrix

def getJacobian():
	x1, x2, x3= symbols('x1 x2 x3')
	# DH = [ [0,0,0,-pi/2],
	# 	   [0,0,0,x1],
	# 	   [0,1.5,0,x2],
	# 	   [0,2.5,-0.75,x3],
	# 	   [0,2.75,-0.75,0] ]

	DH = [ [0,0,0,x1],
		   [0,1.5,0,x2],
		   [0,2.5,-0.75,x3],
		   [0,2.75,-0.75,0] ]

	# Get transformation matrix from end effector to base frame
	t1 = transfromationMatrix( DH[0] ) # hip orientation to base frame
	t2 = t1*transfromationMatrix( DH[1] ) # hip to base frame
	t3 = t2*transfromationMatrix( DH[2] ) # knee to base frame
	t4 = t3*transfromationMatrix( DH[3] ) # ankle to base frame
	# t5 = t4*transfromationMatrix( DH[4] ) # toe to base frame

	simp = sym.simplify(t4)

	# print simp

	# Get Jacobian
	pos = simp[:-1,-1]
	J = pos.jacobian(sym.Matrix([x1,x2,x3]))
	return J


if __name__=="__main__":
	J = getJacobian()
	print J[:-1,:]
	jinv = J[:-1,:].pinv()
	print sym.simplify(jinv)
