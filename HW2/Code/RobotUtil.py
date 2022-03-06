import numpy as np 
import math

def rpyxyz2H(rpy,xyz):
	Ht=[[1,0,0,xyz[0]],
	    [0,1,0,xyz[1]],
            [0,0,1,xyz[2]],
            [0,0,0,1]]

	Hx=[[1,0,0,0],
	    [0,math.cos(rpy[0]),-math.sin(rpy[0]),0],
            [0,math.sin(rpy[0]),math.cos(rpy[0]),0],
            [0,0,0,1]]

	Hy=[[math.cos(rpy[1]),0,math.sin(rpy[1]),0],
            [0,1,0,0],
            [-math.sin(rpy[1]),0,math.cos(rpy[1]),0],
            [0,0,0,1]]

	Hz=[[math.cos(rpy[2]),-math.sin(rpy[2]),0,0],
            [math.sin(rpy[2]),math.cos(rpy[2]),0,0],
            [0,0,1,0],
            [0,0,0,1]]

	H=np.matmul(np.matmul(np.matmul(Ht,Hz),Hy),Hx)

	return H

def R2axisang(R):
	ang = math.acos(( R[0,0] + R[1,1] + R[2,2] - 1)/2)
	Z = np.linalg.norm([R[2,1] - R[1,2], R[0,2] - R[2,0], R[1,0] - R[0,1]])
	if Z==0:
		return[1,0,0], 0.
	x = (R[2,1] - R[1,2])/Z
	y = (R[0,2] - R[2,0])/Z
	z = (R[1,0] - R[0,1])/Z 	

	return[x, y, z], ang


def BlockDesc2Points(H, Dim):
	center = H[0:3,3]
	axes=[ H[0:3,0],H[0:3,1],H[0:3,2]]	

	corners=[
		center,
		center+(axes[0]*Dim[0]/2.)+(axes[1]*Dim[1]/2.)+(axes[2]*Dim[2]/2.),
		center+(axes[0]*Dim[0]/2.)+(axes[1]*Dim[1]/2.)-(axes[2]*Dim[2]/2.),
		center+(axes[0]*Dim[0]/2.)-(axes[1]*Dim[1]/2.)+(axes[2]*Dim[2]/2.),
		center+(axes[0]*Dim[0]/2.)-(axes[1]*Dim[1]/2.)-(axes[2]*Dim[2]/2.),
		center-(axes[0]*Dim[0]/2.)+(axes[1]*Dim[1]/2.)+(axes[2]*Dim[2]/2.),
		center-(axes[0]*Dim[0]/2.)+(axes[1]*Dim[1]/2.)-(axes[2]*Dim[2]/2.),
		center-(axes[0]*Dim[0]/2.)-(axes[1]*Dim[1]/2.)+(axes[2]*Dim[2]/2.),
		center-(axes[0]*Dim[0]/2.)-(axes[1]*Dim[1]/2.)-(axes[2]*Dim[2]/2.)
		]	
	# returns corners of BB and axes
	return corners, axes



def CheckPointOverlap(pointsA, pointsB, axis):	
	# TODO: check if sets of points projected on axis are overlapping
	# Project points
	projPointsA = np.matmul(axis, np.transpose(pointsA))
	projPointsB = np.matmul(axis, np.transpose(pointsB))
	
	# Check overlap
	maxA = np.max(projPointsA)
	minA = np.min(projPointsA)
	maxB = np.max(projPointsB)
	minB = np.min(projPointsB)

	if minB <= maxA <= maxB:
		return True
	if minB <= minA <= maxB:
		return True
	if minA <= maxB <= maxA:
		return True
	if minA <= minB <= maxA:
		return True

	return False


def CheckBoxBoxCollision(pointsA, axesA, pointsB, axesB):	
	# check collision between two cuboids
	# distance between two centers > sum of the two radius
	if np.linalg.norm(pointsA[0] - pointsB[0]) > (np.linalg.norm(pointsA[0] - pointsA[1]) + np.linalg.norm(pointsB[0] - pointsB[1])):
		return False

	# SAT cuboid-cuboid collision check
	# Surface normal check
	for i in range(3):
		if not CheckPointOverlap(pointsA, pointsB, axesA[i]):
			return False
		
	for j in range(3):
		if not CheckPointOverlap(pointsA, pointsB, axesB[j]):
			return False

	# Edge check
	for i in range(3):
		for j in range(3):
			if not CheckPointOverlap(pointsA, pointsB, np.cross(axesA[i], axesB[j])):
				return False
	
	return True
	


if __name__ == "__main__":
	ref_corners, ref_axes = BlockDesc2Points(rpyxyz2H((0, 0, 0), (0, 0, 0)), (3, 1, 2))

	test_origin = [(0, 1, 0), (1.5, -1.5, 0), (0, 0, -1), (3, 0, 0), (-1, 0, -2), (1.8, 0.5, 1.5), (0, -1.2, 0.4), (-0.8, 0, -0.5)]

	test_orient = [(0, 0, 0), (1, 0, 1.5), (0, 0, 0), (0, 0, 0), (0.5, 0, 0.4), (-0.2, 0.5, 0), (0, 0.785, 0.785), (0, 0, 0.2)]

	test_dims = [(0.8, 0.8, 0.8), (1, 3, 3), (2, 3, 1), (3, 1, 1), (2, 0.7, 2), (1, 3, 1), (1, 1, 1), (1, 0.5, 0.5)]

	for i in range(len(test_origin)):
		test_corners, text_axes = BlockDesc2Points(rpyxyz2H(test_orient[i], test_origin[i]), test_dims[i])
		print(CheckBoxBoxCollision(ref_corners, ref_axes, test_corners, text_axes))