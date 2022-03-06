import vrep
import time


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
		CartRobotJointNames=["joint_1","joint_2","joint_3","joint_4","joint_5"]

		self.JointHandles=[vrep.simxGetObjectHandle(self.clientID,Name,vrep.simx_opmode_blocking)[1] for Name in CartRobotJointNames ]

		#Start Streaming buffers
		JointPosition=[vrep.simxGetJointPosition(self.clientID, JointHandle,vrep.simx_opmode_streaming)[1] for JointHandle in self.JointHandles];

	def destroy(self):
		vrep.simxStopSimulation(self.clientID,vrep.simx_opmode_blocking)
		vrep.simxFinish(self.clientID)



	def getJointPos(self):
		CurJointPosition=[vrep.simxGetJointPosition(self.clientID, JointHandle,vrep.simx_opmode_buffer)[1] for JointHandle in self.JointHandles];
		return CurJointPosition
	

	def move(self, DesJointPosition):
		CurJointPosition=self.getJointPos()
		for i in range(15):
			t=min(i,12.)/12.
			vrep.simxPauseCommunication(self.clientID,1);
			for j in range(5):
				vrep.simxSetJointTargetPosition(self.clientID,self.JointHandles[j], (1.0-t)*CurJointPosition[j]+t*DesJointPosition[j],vrep.simx_opmode_oneshot)
			vrep.simxPauseCommunication(self.clientID,0);
			for step in range(3):
				vrep.simxSynchronousTrigger(self.clientID);
			time.sleep(0.02)

			
	       




