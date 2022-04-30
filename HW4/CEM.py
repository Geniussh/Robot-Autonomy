import numpy as np
import time
import random
import copy as cp
import robot_interface as ri
import matplotlib.pyplot as plt
from plotter import plot_rewards
np.random.seed(13)

#initialize policy
policyMu = [-.5,-.5,0,0]
policyCov = np.diag([0.1, 0.1,1.,1. ])
PlotRewards=[]

for policyupdate in range(5):
	SkillParam=[]
	R=[]
	#Collect new samples
	for samp in range(15):
		SkillParam.append(np.random.multivariate_normal(policyMu,policyCov))
		projPos=ri.RolloutPolicy(SkillParam[-1])
		Reward= ri.RewardFunction(projPos)
		R.append(Reward)
		PlotRewards.append(Reward)
		print(policyupdate, samp, Reward)

	## Find k=5 highest rewarding samples
	SortR=cp.copy(R)
	SortR.sort()
	w=[ 1.0* (R[i]>= SortR[-5]) for i in range(len(R))]

	##Update policy with weighted samples
	policyMu =np.average(SkillParam,axis=0,weights=w)
	policyCov =np.cov(SkillParam,rowvar=False,aweights=w)

plot_rewards(PlotRewards)
