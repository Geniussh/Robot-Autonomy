import numpy as np
import time
import copy as cp
import matplotlib.pyplot as plt
from scipy import optimize
from plotter import plot_rewards
import robot_interface as ri
np.random.seed(13)
PlotRewards=[]


class GaussProcess:
	def __init__(self,sigma_a,sigma_n,sigma_l): #Store. amplitude, noise, and lengthscale hyperparameters
		self.sigma_a=np.array(sigma_a)
		self.sigma_n=np.array(sigma_n)
		self.sigma_l=np.array(sigma_l)

	def train(self,x_train,y_train): #Store training data and compute K matrix
		self.x_train=x_train
		self.y_train=np.array(y_train)

		self.K=np.zeros([len(self.x_train),len(self.x_train)])
		for i in range(len(self.x_train)):
			for j in range(len(self.x_train)):
				# TODO: Populate kernel matrix K (AKA Gram Matrix)
				self.K[i,j]=self.sigma_a**2 * np.exp(-0.5 * np.matmul(np.matmul(self.x_train[i] - self.x_train[j], np.diag(self.sigma_l)**2), self.x_train[i] - self.x_train[j])) #compute kernel matrix K between all training points
				if i==j:
					self.K[i,j]+=self.sigma_n**2
		self.K_inv=np.linalg.inv(self.K)

	def test(self,x): #Compute k vector for test case and compute mean and standard deviation prediction
		k_vec=np.zeros([1,len(self.x_train)])
		for j in range(len(self.x_train)):
			k_vec[0,j]=self.sigma_a**2*np.exp(-0.5*np.matmul(np.matmul(x-self.x_train[j],np.diag(self.sigma_l)**2),x-self.x_train[j]))
		y_mu= np.matmul(np.matmul(k_vec, self.K_inv), self.y_train.transpose()) #TODO: compute expected mean for new point x
		y_std= np.sqrt(self.sigma_n**2 + self.sigma_a**2 - np.matmul(np.matmul(k_vec, self.K_inv), k_vec.transpose())) #TODO: compute variance (and then take sqrt) of prediction at x
		return np.squeeze(y_mu),np.squeeze(y_std)


x_train=[]
y_train=[]

#Collet a couple of initial samples
for samp in range(2):
	x_train.append(np.random.multivariate_normal([-.5,-.5,0,0],np.diag([0.1, 0.1,1.,1. ])))
	projPos= ri.RolloutPolicy(x_train[-1])
	Reward= ri.RewardFunction(projPos)
	y_train.append(Reward)

GP=GaussProcess(1.,0.15,[1/0.05,1/0.05,1/0.05,1/0.05])
GP.train(x_train, y_train);


#Bayesiann Optimization Loop
for iter in range(45):
	#Use CEM to maximize acquisition function
	policyMu = [-.5,-.5,0,0]
	policyCov = np.diag([0.1, 0.1,1.,1. ])
	for policyupdate in range(100):
		SkillParam=[]
		R=[]
		for samp in range(250):
			SkillParam.append(np.random.multivariate_normal(policyMu,policyCov))
			y_mu,y_std=GP.test(SkillParam[-1])
			Acquisition= y_mu + y_std #TODO: compute UCB acquisition function value
			R.append(Acquisition)
		# CEM fitting of next distribution
		SortR=cp.copy(R)
		SortR.sort()
		w=[ 1.0* (R[i]>= SortR[-30]) for i in range(len(R))]

		policyMu =np.average(SkillParam,axis=0,weights=w)
		policyCov =np.cov(SkillParam,rowvar=False,aweights=w)
		y_mu,y_std=GP.test(policyMu)

	#Collect a new sample.  Using mean, but could also use highest weight one
	y_mu,y_std=GP.test(policyMu)
	param = np.random.multivariate_normal(policyMu, policyCov)
	projPos=ri.RolloutPolicy(param)  #TODO: rollout
	Reward= ri.RewardFunction(projPos)
	PlotRewards.append(Reward)
	print(iter, y_mu, y_mu+y_std, Reward)

	#Update Gaussian Process
	x_train.append(param) #TODO
	y_train.append(Reward) #TODO
	GP.train(x_train, y_train);

plot_rewards(PlotRewards)
