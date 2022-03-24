import numpy as np

def CheckCondition(state,condition):
	if (np.sum(np.multiply(state, condition))-np.sum(np.multiply(condition, condition)))==0:
		return True
	else:
		return False


def CheckVisited(state,vertices):
	for i in range(len(vertices)):
		if np.linalg.norm(np.subtract(state,vertices[i]))==0:
			return True, i
	return False, None


def ComputeNextState(state, effect):
	newstate=np.add(state, effect)
	return newstate


def Heuristic(state, GoalIndicesOneStep, GoalIndicesTwoStep): 
	score=0

	for idx in GoalIndicesOneStep:
		# if the robot is not in kitchen or if the strawberry is not chopped
		# robot teleoperates to kitchen, or use psionics to cut strawberry
		if state[idx[0]][idx[1]]==-1:
			score+=1

	for idx in GoalIndicesTwoStep:
		# if the strawberry is not in kitechen or lemon not in garden
		# plus, if the robot does not have strawberry/lemon on itself
		# it needs two steps: teleoperate object to itself and teleoperate 
		# object to the goal room
		if state[idx[0]][idx[1]]==-1 and state[idx[0]][-1]==-1:
			score+=2

		# else if robot already has the object on itself,
		# it only needs to teleoperate object to the goal room
		elif state[idx[0]][idx[1]]==-1 and state[idx[0]][-1]==1:
			score+=1	
	
	return score


Predicates=['InHallway', 'InKitchen', 'InOffice', 'InLivingRoom', 'InGarden','InPantry','Chopped','OnRobot']

Objects=['Robot','Strawberry','Lemon', 'Paper', 'Knife'] 

nrPredicates=len(Predicates)
nrObjects=len(Objects)

ActionPre=[]
ActionEff=[]
ActionDesc=[]

###Move to hallway
for i in range(1,5,1):
	Precond=np.zeros([nrObjects, nrPredicates])
	Precond[0][0]=-1 #Robot not in hallway
	Precond[0][i]=1  #Robot in i-th room

	Effect=np.zeros([nrObjects, nrPredicates])
	Effect[0][0]=2.  #Robot in the hallway
	Effect[0][i]=-2. #Robot not in the i-th room

	ActionPre.append(Precond)
	ActionEff.append(Effect)
	ActionDesc.append("Move to InHallway from "+Predicates[i])

###Move to room
for i in range(1,5,1):
	Precond=np.zeros([nrObjects, nrPredicates])
	Precond[0][0]=1  #Robot in the hallway
	Precond[0][i]=-1 #Robot not in the ith room

	Effect=np.zeros([nrObjects, nrPredicates])
	Effect[0][0]=-2. #Robot not in the hallway
	Effect[0][i]=2.  #Robot in the ith room

	ActionPre.append(Precond)
	ActionEff.append(Effect)
	ActionDesc.append("Move to "+Predicates[i]+" from InHallway")


###Move to Pantry 
Precond=np.zeros([nrObjects, nrPredicates])
Precond[0][1]=1  #Robot in the kitchen
Precond[0][5]=-1 #Robot not in the pantry

Effect=np.zeros([nrObjects, nrPredicates])
Effect[0][1]=-2 #Robot not in the kitchen
Effect[0][5]=2 #Robot in the pantry

ActionPre.append(Precond)
ActionEff.append(Effect)
ActionDesc.append("Move to Pantry from Kitchen")


###Move from Pantry 
Precond=np.zeros([nrObjects, nrPredicates])
Precond[0][1]=-1 #Robot not in the kitchen
Precond[0][5]=1 #Robot in the pantry

Effect=np.zeros([nrObjects, nrPredicates])
Effect[0][1]=2 #Robot in the kitchen
Effect[0][5]=-2 #Robot not in the pantry

ActionPre.append(Precond)
ActionEff.append(Effect)
ActionDesc.append("Move to Kitchen from Pantry")


###Cut fruit in kitchen
for j in [1,2]:
	Precond=np.zeros([nrObjects, nrPredicates])
	Precond[0][1]=1 #Robot in the kitchen
	Precond[j][1]=1 #Fruit in the kitchen
	Precond[4][1]=1 #Knife in the kitchen
	Precond[j][-2]=-1 #Fruit not chopped


	Effect=np.zeros([nrObjects, nrPredicates])
	Effect[j][-2]=2 #Fruit chopped

	ActionPre.append(Precond)
	ActionEff.append(Effect)
	ActionDesc.append("Cut "+Objects[j]+" in the kitchen")


###Pickup object
for i in range(1,6,1):
	for j in range(1,5,1):
		Precond=np.zeros([nrObjects, nrPredicates])
		Precond[0][i]=1 #Robot in ith room
		Precond[j][i]=1 #Object j in ith room
		Precond[j][-1]=-1 #Object j not on robot

		Effect=np.zeros([nrObjects, nrPredicates])
		Effect[j][i]=-2 #Object j not in ith room
		Effect[j][-1]=2 # Object j on robot

		ActionPre.append(Precond)
		ActionEff.append(Effect)
		ActionDesc.append("Pick up "+Objects[j]+" from "+Predicates[i])
	

###Place object
for i in range(1,6,1):
	for j in range(1,5,1):
		Precond=np.zeros([nrObjects, nrPredicates])
		Precond[0][i]=1 #Robot in ith room
		Precond[j][i]=-1 #Object j not in ith room
		Precond[j][-1]=1 #Object j on robot

		Effect=np.zeros([nrObjects, nrPredicates])
		Effect[j][i]=2.  #Object j in ith room
		Effect[j][-1]=-2 #Object j not on robot

		ActionPre.append(Precond)
		ActionEff.append(Effect)
		ActionDesc.append("Place "+Objects[j]+" at "+Predicates[i])



InitialState=-1*np.ones([nrObjects, nrPredicates])
InitialState[0][0]=1 # Robot is in the hallway
InitialState[1][4]=1 # Strawberry is in the garden
InitialState[2][5]=1 # Lemon is in the pantry
InitialState[3][2]=1 # Paper is in the office
InitialState[4][2]=1 # Knife is in the office

GoalState=np.zeros([nrObjects, nrPredicates])
GoalState[0][1]=1 # Robot is in the kitchen
GoalState[1][1]=1 # Strawberry is in the kitchen
GoalState[2][4]=1 # Lemon is in the Garden
GoalState[1][6]=1 # Strawberry is chopped

GoalIndicesOneStep=[[0,1],[1,6]]
GoalIndicesTwoStep=[[1,1],[2,4]]

np.random.seed(13)


# Search for Solution
vertices=[]  # store all nodes visited so far
parent=[]
action=[]

cost2come=[]
cost2go=[]

Queue=[]  # store indices of vertices/nodes
Queue.append(0)
vertices.append(InitialState)
parent.append(0)
action.append(-1)
cost2come.append(0)  # cost corresponding to each index of the node
cost2go.append(Heuristic(InitialState, GoalIndicesOneStep, GoalIndicesTwoStep))

FoundPath=False
while len(vertices)<6000:
	#TODO perform search
	# Get highest priority
	P = np.array([cost2come[q] + cost2go[q] for q in Queue])
	id = P.argmin()  # the index that gives the highest priority node

	x = Queue[id]
	del Queue[id]

	cur = vertices[x]
	cost_so_far = cost2come[x]

	# Check Goal
	if CheckCondition(cur, GoalState):
		FoundPath = True
		break;

	# Check each action (if precondition matches the current node, then if visited)
	for i in range(len(ActionPre)):
		if CheckCondition(cur, ActionPre[i]):
			next = ComputeNextState(cur, ActionEff[i])
			hasVisited, next_index = CheckVisited(next, vertices)
			if hasVisited:  # visited before, already has a cost
				if cost2come[next_index] > cost_so_far + 1:  # each step (edge weight) is one in our case
					cost2come[next_index] = cost_so_far + 1
					parent[next_index] = x
				
			else:  # not visited
				vertices.append(next)
				action.append(i)
				parent.append(x)
				cost2come.append(cost_so_far + 1) 
				cost2go.append(Heuristic(next, GoalIndicesOneStep, GoalIndicesTwoStep))
				Queue.append(len(vertices) - 1)  # again, Queue stores the index

# Print Plan
print("\n FoundPath: ", FoundPath, " ", len(vertices))

Plan=[]
if FoundPath:
	while not x==0:
		Plan.insert(0,action[x])
		x=parent[x]
		
for i in range(len(Plan)):
	print(ActionDesc[Plan[i]])
			




