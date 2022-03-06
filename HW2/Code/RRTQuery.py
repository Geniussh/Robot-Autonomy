import time
import pickle
import numpy as np

import vrep_interface as vpi
import RobotUtil as rt
import Locobot

def FindNearest(prevPoints,newPoint):
	D = np.array([np.linalg.norm(np.array(point)-np.array(newPoint)) for point in prevPoints])
	return D.argmin()


np.random.seed(0)
deg_to_rad = np.pi/180.

#Initialize robot object
mybot=Locobot.Locobot()

#Create environment obstacles - # these are blocks in the environment/scene (not part of robot) 
pointsObs=[]
axesObs=[]

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[0.275,-0.15,0.]),[0.1,0.1,1.05])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[0.275,0.05,0.425]),[0.1,0.3,0.1])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[0.275,0.25,0.4]),[0.1,0.1,0.15])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[0.425,0.25,0.375]),[0.2,0.1,0.1])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[-0.1,0.0,0.675]),[0.45,0.15,0.1])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[-0.275,0.0,0.]),[0.1,1.0,1.25])
pointsObs.append(envpoints), axesObs.append(envaxes)

# base and mount
envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[0.,0.0,0.05996]),[0.35004,0.3521,0.12276])
pointsObs.append(envpoints), axesObs.append(envaxes)

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],[-0.03768,0.0,0.36142]),[0.12001,0.26,0.5])
pointsObs.append(envpoints), axesObs.append(envaxes)
	

# define start and goal
deg_to_rad = np.pi/180.
qInit = np.array([-80.*deg_to_rad, 0., 0., 0., 0.])
qGoal = np.array([0., 60*deg_to_rad, -75*deg_to_rad, -75*deg_to_rad, 0.])


#TODO - Create RRT to find path to a goal configuration
rrtVertices=[]
rrtEdges=[]

rrtVertices.append(qInit)
rrtEdges.append(0)

thresh=0.25
FoundSolution=False

while len(rrtVertices)<3000 and not FoundSolution:
	qRand = np.array(mybot.SampleRobotConfig())

	# Goal Bias
	if np.random.uniform(0, 1) < 0.05:
		qRand = qGoal
	
	idNear = FindNearest(rrtVertices, qRand)
	qNear = np.array(rrtVertices[idNear])

	# Keep connecting qNear and qRand up to threshold
	while np.linalg.norm(qRand - qNear) > thresh:
		qConnect = np.array(qNear) + thresh * (np.array(qRand) - np.array(qNear)) / np.linalg.norm(qRand - qNear)
		
		if not mybot.DetectCollisionEdge(qConnect, qNear, pointsObs, axesObs):
			rrtVertices.append(qConnect)
			rrtEdges.append(idNear)
			qNear = qConnect
		else:
			break
	
	qConnect = qRand
	if not mybot.DetectCollisionEdge(qConnect, qNear, pointsObs, axesObs):
		rrtVertices.append(qConnect)
		rrtEdges.append(idNear)

	idNear = FindNearest(rrtVertices, qGoal)
	if np.linalg.norm(qGoal - rrtVertices[idNear]) < 0.025:
		rrtVertices.append(qGoal)
		rrtEdges.append(idNear)
		FoundSolution = True
		break

### if a solution was found
		
if FoundSolution:
	# Extract path
	plan=[]
	c=-1 #Assume last added vertex is at goal 
	plan.insert(0, rrtVertices[c])

	while True:
		c=rrtEdges[c]
		plan.insert(0, rrtVertices[c])
		if c==0:
			break

	# TODO - Path shortening
	for i in range(150):
		# Sample vertices
		anchorA = np.random.randint(0, len(plan) - 2)
		anchorB = np.random.randint(anchorA + 1, len(plan) - 1)
		
		# Sample shift along edges
		shiftA = np.random.uniform(0, 1)
		shiftB = np.random.uniform(0, 1)

		# Compute test vertices
		candidateA = (1 - shiftA) * plan[anchorA] + shiftA * plan[anchorA + 1]
		candidateB = (1 - shiftB) * plan[anchorB] + shiftB * plan[anchorB + 1]

		# Shorten
		if not mybot.DetectCollisionEdge(candidateA, candidateB, pointsObs, axesObs):
			while anchorB > anchorA:
				plan.pop(anchorB)
				anchorB = anchorB - 1
			plan.insert(anchorA + 1, candidateB)
			plan.insert(anchorA + 1, candidateA)
		
	robot=vpi.vBot()
	robot.connect()
	
	for q in plan:
		robot.move(q)
		time.sleep(1)

	robot.destroy()

else:
	print("No solution found")



