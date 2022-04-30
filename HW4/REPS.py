import numpy as np
import time
import matplotlib.pyplot as plt
import random
from scipy import optimize
from plotter import plot_rewards
import robot_interface as ri
np.random.seed(13)
PlotRewards=[]

#Function for computing eta parameters
def computeEta(returns,epsilon):
	#Make more numerically stable by removing max return
	R = returns - np.max(returns)

	#Define dual function to be optimized
	def dual_function(eta):
		#TODO: Compute dual function for optimization (will use R)
		f = eta * np.log(np.mean(np.exp(R / eta))) + np.max(returns) + eta * epsilon
		return f

	#Perform optimization of dual function
	eta = optimize.minimize(dual_function, 1,bounds=[(0.00000001,10000)]).x
	return eta[0]


#Define initial policy
policyMu = [-.5,-.5,0,0]
policyCov = np.diag([0.1, 0.1,1.,1. ])

for policyupdate in range(5):
	SkillParam=[]
	R=[]

	#Collect data
	for samp in range(15):
		SkillParam.append(np.random.multivariate_normal(policyMu,policyCov))
		projPos=ri.RolloutPolicy(SkillParam[-1])
		Reward= ri.RewardFunction(projPos)
		R.append(Reward)
		PlotRewards.append(Reward)
		print(policyupdate, samp, Reward)

	## REPS update of weights
	Eta=computeEta(R,1.5)
	w=np.exp(R/Eta)
	w=w/np.sum(w)

	#Compute new policy parameters from weighted samples
	policyMu =np.average(SkillParam,axis=0,weights=w)
	policyCov =np.cov(SkillParam,rowvar=False,aweights=w)

plot_rewards(PlotRewards)
