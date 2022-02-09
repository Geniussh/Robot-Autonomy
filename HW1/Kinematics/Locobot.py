import numpy as np
import math
import time

import RobotUtil as rt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Locobot:

	def __init__(self):
		# Robot descriptor taken from URDF file (rpy xyz for each rigid link transform) - NOTE: don't change
		self.Rdesc=[
			[0, 0, 0, 0.08, 0, 0.159], # From robot base to joint1
			[0, 0, 0, 0, 0, 0.04125],
			[0, 0, 0, 0.05, 0, 0.2],
			[0, 0, 0, 0.2002, 0, 0],
			[0, 0, 0, 0.063, 0.0001, 0],
			[0, 0, 0, 0.106525, 0, 0.0050143] # From joint5 to end-effector center
			]
        
		#Define the axis of rotation for each joint 
		self.axis=[
            		[0, 0, 1],
            		[0, 1, 0],
            		[0, 1, 0],
            		[0, 1, 0],
           		[-1, 0, 0],
			[0, 1, 0]
        		]
		self.axis = np.array(self.axis)

		#Set base coordinate frame as identity - NOTE: don't change
		self.Tbase= [[1,0,0,0],
			[0,1,0,0],
			[0,0,1,0],
			[0,0,0,1]]
		
		#Initialize matrices - NOTE: don't change this part
		self.Tlink=[] #Transforms for each link (const)
		self.Tjoint=[] #Transforms for each joint (init eye)
		self.Tcurr=[] #Coordinate frame of current (init eye)
		for i in range(len(self.Rdesc)):
			self.Tlink.append(rt.rpyxyz2H(self.Rdesc[i][0:3],self.Rdesc[i][3:6]))
			self.Tcurr.append([[1,0,0,0],[0,1,0,0],[0,0,1,0.],[0,0,0,1]])
			self.Tjoint.append([[1,0,0,0],[0,1,0,0],[0,0,1,0.],[0,0,0,1]])

		self.Tlinkzero=rt.rpyxyz2H(self.Rdesc[0][0:3],self.Rdesc[0][3:6])
		self.Tlink[0]=np.matmul(self.Tbase,self.Tlink[0])

		# initialize Jacobian matrix
		self.J=np.zeros((6,5))
		
		self.q=[0.,0.,0.,0.,0.,0.]
		self.ForwardKin([0.,0.,0.,0.,0.])

	def ForwardKin(self,ang):
		'''
		inputs: joint angles
		outputs: joint transforms for each joint, Jacobian matrix
		'''
		self.q[0:-1] = ang		
		# TODO: Compute current joint and end effector coordinate frames (self.Tjoint). Remember that not all joints rotate about the z axis!
		for i in range(len(self.q)):
			self.Tjoint[i] = rt.rpyxyz2H(self.axis[i] * self.q[i], [0, 0, 0])
			if i == 0:
				self.Tcurr[i] = self.Tlink[i] @ self.Tjoint[i]
			else:
				self.Tcurr[i] = self.Tcurr[i-1] @ self.Tlink[i] @ self.Tjoint[i]
				
		# TODO: Compute Jacobian matrix	
		for i in range(len(self.q) - 1):
			# all 5 joints are rotational joints
			p = self.Tcurr[-1][0:3, 3] - self.Tcurr[i][0:3, 3]
			axis = np.argwhere(self.axis[i])[0][0]
			a = self.Tcurr[i][0:3, axis]
			self.J[0:3, i] = np.cross(a, p)
			self.J[3:7, i] = a
		
		return self.Tcurr, self.J


	def IterInvKin(self, ang, TGoal, x_eps=1e-3, r_eps=1e-3):
		'''
		inputs: starting joint angles (ang), target end effector pose (TGoal)

		outputs: computed joint angles to achieve desired end effector pose, 
		Error in your IK solution compared to the desired target
		'''	
		# Damped Least Squares Parameters
		W = np.eye(len(self.q) - 1)
		Winv = np.linalg.inv(W)
		C = 1e6 * np.eye(6)
		Cinv = np.linalg.inv(C)

		self.ForwardKin(ang)
		
		Err=[0,0,0,0,0,0] # error in position and orientation, initialized to 0
		for s in range(20000):			
			#TODO: Compute rotation error
			rErrR = TGoal[0:3, 0:3] @ self.Tcurr[-1][0:3, 0:3].T
			rErrAxis, rErrAng = rt.R2axisang(rErrR)
			if rErrAng > 0.1: rErrAng = 0.1 * np.pi / 180
			if rErrAng < -0.1: rErrAng = -0.1 * np.pi / 180
			rErr = [rErrAxis[0] * rErrAng, rErrAxis[1] * rErrAng, rErrAxis[2] * rErrAng]
						
			#TODO: Compute position error
			xErr = TGoal[0:3, 3] - self.Tcurr[-1][0:3, 3]
			if np.linalg.norm(xErr) > 0.01:
				xErr = xErr * 0.01 / np.linalg.norm(xErr)
			
			#TODO: Update joint angles 
			Err[0:3] = xErr
			Err[3:6] = rErr
			
			# Print for debug
			if s % 250 == 0 or s == 9999:
				print("Iteration %d: Position Error %.5f Orientation Error %.5ff" % (s, np.linalg.norm(xErr), np.linalg.norm(rErr)))

			# if norm of the error is below thresholds, then exit 
			if np.linalg.norm(Err[0:3]) <= x_eps and np.linalg.norm(Err[3:6]) <= r_eps:
				break			
	
			self.q[0:-1] = self.q[0:-1] + Winv @ self.J.T @ np.linalg.inv(self.J @ Winv @ self.J.T + Cinv) @ Err  # Damped LS Approach
			# self.q[0:-1] = self.q[0:-1] + 0.1 * self.J.T @ Err  # Jacobian Transpose Approach
			
			self.ForwardKin(self.q[0:-1]) # Recompute forward kinematics for new angles

		return self.q[0:-1], Err


	def PlotSkeleton(self,ang):
		'''
		this code plots the output of you FK
		'''
		#Compute forward kinematics for ang
		self.ForwardKin(ang)

		#Create figure
		fig =plt.figure()
		ax = fig.add_subplot(111,projection='3d')

		#Draw links along coordinate frames 
		for i in range(len(self.Tcurr)):
			ax.scatter(self.Tcurr[i][0,3], self.Tcurr[i][1,3], self.Tcurr[i][2,3], c='k', marker='.')
			if i is 0:
				ax.plot([0,self.Tcurr[i][0,3]], [0,self.Tcurr[i][1,3]], [0,self.Tcurr[i][2,3]], c='b')
			else:
				ax.plot([self.Tcurr[i-1][0,3],self.Tcurr[i][0,3]], [self.Tcurr[i-1][1,3],self.Tcurr[i][1,3]], [self.Tcurr[i-1][2,3],self.Tcurr[i][2,3]], c='k')

		#Format axes and display
		# ax.set_aspect('equal')
		ax.set(xlim=(-1., 1.), ylim=(-1., 1.), zlim=(0,1.))
		ax.set_xlabel('X-axis')
		ax.set_ylabel('Y-axis')
		ax.set_zlabel('Z-axis')

		plt.show()


