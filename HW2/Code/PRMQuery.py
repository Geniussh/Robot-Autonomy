import numpy as np
import time
import pickle
import matplotlib.pyplot as plt
import random

import Locobot
import RobotUtil as rt
import vrep_interface as vpi


# NOTE: Please set a random seed for your random joint generator!
random.seed(13)

#Initialize robot object
mybot = Locobot.Locobot()

# open road map
f = open("myPRM.p", 'rb')
prmVertices = pickle.load(f)
prmEdges = pickle.load(f)
pointsObs = pickle.load(f)
axesObs = pickle.load(f)
f.close

# define start and goal
deg_to_rad = np.pi/180.
qInit=[-80.*deg_to_rad, 0., 0., 0., 0.] 
qGoal=[0., 60*deg_to_rad, -75*deg_to_rad, -75*deg_to_rad, 0.] 

#Find neighbors to initial and goal nodes
neighInit=[]
neighGoal=[]
heuristic=[]
parent=[]
for i in range(len(prmVertices)):
	if np.linalg.norm(np.array(prmVertices[i])-np.array(qInit))<2.:
		if not mybot.DetectCollisionEdge(prmVertices[i], qInit, pointsObs, axesObs):
			neighInit.append(i)

	if np.linalg.norm(np.array(prmVertices[i])-np.array(qGoal))<2.:
		if not mybot.DetectCollisionEdge(prmVertices[i], qGoal, pointsObs, axesObs):
			neighGoal.append(i)

	heuristic.append(np.linalg.norm(np.array(prmVertices[i])-np.array(qGoal)))
	parent.append([])


activenodes= neighInit
bestscore=0  # the distance between a node and the goal

# Starting from neighbors of qInit, find a path from any node in neighInit
# to any node in neighGoal. For each iteration, we iterate all the unvisted
# neighbors of all the nodes in the current expansion set, activenodes, and
# then choose one node that has the least distance to qGoal and add it to
# the expansion set. Stop if the expansion set contain any neighbor of qGoal
while bestscore<1000 and not any([g in activenodes for g in neighGoal]):
	bestscore=1000
	for i in range(len(activenodes)):
		for j in range(len(prmEdges[activenodes[i]])):  # iterate each edge of the current node activenodes[i]
			if prmEdges[activenodes[i]][j] not in activenodes:
				if heuristic[prmEdges[activenodes[i]][j]]<bestscore:
					bestscore=heuristic[prmEdges[activenodes[i]][j]]
					bestcandi=prmEdges[activenodes[i]][j]
					bestparent=activenodes[i]

	if bestscore<1000:
		activenodes.append(bestcandi)
		parent[bestcandi]= bestparent


if any([g in activenodes for g in neighGoal]):
	print("Found a path")
else:
	print("Failed to find a plan")

plan= [activenodes[-1]]
prevstep= parent[plan[0]]
while prevstep:
	plan.insert(0, prevstep)
	prevstep = parent[plan[0]]
print("Plan: ", plan)

MyPlan=[]
MyPlan.append(qInit)
for i in range(len(plan)):
	MyPlan.append(prmVertices[plan[i]])
MyPlan.append(qGoal)


# V-REP interface
robot=vpi.vBot()
robot.connect()

for q in MyPlan:
	robot.move(q)

robot.destroy()