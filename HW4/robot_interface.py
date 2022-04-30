import vrep
import time
import numpy as np

def RolloutPolicy(Param):
	robot=vBot()
	robot.connect()
	robot.move(Param[0:2])
	robot.resetProjectile()
	robot.move(Param[2:4])
	robot.wait()
	position=robot.getProjectilePos()

	robot.destroy()
	return position

def RewardFunction(projPos):
	return 2.0-np.linalg.norm(projPos[0]-(-2.));

class vBot:

	#def __init__(self):


	def connect(self):
		vrep.simxFinish(-1) # just in case, close all opened connections
		self.clientID=vrep.simxStart('127.0.0.1', 19997,True,True,5000,5) # Connect to V-REP
		if self.clientID==-1:
			print ('Failed connecting to remote API server')
			exit()

		#Start simulation and enter sync mode
		vrep.simxSynchronous(self.clientID,True)
		vrep.simxStartSimulation(self.clientID,vrep.simx_opmode_oneshot)

		# Create connections to joints and sensors
		TossRobotJointNames=["joint_1","joint_2"]
		ObjectNames=["Projectile","Launcher"]

		self.JointHandles=[vrep.simxGetObjectHandle(self.clientID,Name,vrep.simx_opmode_blocking)[1] for Name in TossRobotJointNames]
		self.ObjectHandles=[vrep.simxGetObjectHandle(self.clientID,Name,vrep.simx_opmode_blocking)[1] for Name in ObjectNames]


		#Start Streaming buffers
		JointPosition=[vrep.simxGetJointPosition(self.clientID, JointHandle,vrep.simx_opmode_streaming)[1] for JointHandle in self.JointHandles];
		ObjectPosition=[vrep.simxGetObjectPosition(self.clientID, ObjectHandle,-1,vrep.simx_opmode_streaming)[1] for ObjectHandle in self.ObjectHandles];


	def destroy(self):
		vrep.simxStopSimulation(self.clientID,vrep.simx_opmode_blocking)
		vrep.simxFinish(self.clientID)


	def getJointPos(self):
		CurJointPosition=[vrep.simxGetJointPosition(self.clientID, JointHandle,vrep.simx_opmode_buffer)[1] for JointHandle in self.JointHandles];
		return CurJointPosition


	def move(self, DesJointPosition):
		CurJointPosition=self.getJointPos()
		vrep.simxPauseCommunication(self.clientID,1);
		for j in range(2):
			vrep.simxSetJointTargetPosition(self.clientID,self.JointHandles[j], DesJointPosition[j],vrep.simx_opmode_oneshot)
		vrep.simxPauseCommunication(self.clientID,0);


		for i in range(45):
			vrep.simxSynchronousTrigger(self.clientID);
	def wait(self):
		for i in range(150):
			vrep.simxSynchronousTrigger(self.clientID);

	def resetProjectile(self):
		returnCode, position=vrep.simxGetObjectPosition(self.clientID,self.ObjectHandles[1],-1,vrep.simx_opmode_buffer)
		vrep.simxSetObjectPosition(self.clientID,self.ObjectHandles[0],-1, position,vrep.simx_opmode_oneshot)
		for i in range(45):
			vrep.simxSynchronousTrigger(self.clientID);

	def getProjectilePos(self):
		returnCode, position=vrep.simxGetObjectPosition(self.clientID,self.ObjectHandles[0],-1,vrep.simx_opmode_buffer)
		return position
