import Locobot
import numpy as np
import matplotlib.pyplot as plt
import random
import pickle
import RobotUtil as rt
import time

random.seed(13)

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


# Create PRM - generate collision-free vertices
prmVertices = []
prmEdges = []
radius = 2
num_vertices = 1000
start = time.time()

while len(prmVertices) < num_vertices:
	# sample random poses 
	q = mybot.SampleRobotConfig()
	if not mybot.DetectCollision(q, pointsObs, axesObs):
		prmVertices.append(q)  # added collision-free vertex
		prmEdges.append([])  # to add collision-free edge
		# find neighbors within <radius>
		for j in range(len(prmVertices) - 1):
			if np.linalg.norm(np.array(prmVertices[-1]) - np.array(prmVertices[j])) < 2:
				if not mybot.DetectCollisionEdge(prmVertices[-1], prmVertices[j], pointsObs, axesObs):
					prmEdges[-1].append(j)
					prmEdges[j].append(len(prmVertices) - 1)
	
	print(len(prmVertices))

#Save the PRM such that it can be run by PRMQuery.py
f = open("myPRM.p", 'wb')
pickle.dump(prmVertices, f)
pickle.dump(prmEdges, f)
pickle.dump(pointsObs, f)
pickle.dump(axesObs, f)
f.close

print("\n",len(prmVertices),": ", time.time()-start)





