#!/usr/bin/python
#coding: utf-8

#Ignoring warnings that arise due to scipy being built over the new numpy
import warnings
warnings.filterwarnings("ignore", message="numpy.dtype size changed")
warnings.filterwarnings("ignore", message="numpy.ufunc size changed")

import math
import numpy as np
from scipy.integrate import solve_ivp
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from matplotlib.pylab import *
from mpl_toolkits.axes_grid1 import host_subplot
import matplotlib.animation as animation
from cvxopt import matrix
from cvxopt import solvers


fig = figure(num = 0, figsize = (12, 8))#, dpi = 100)
fig.suptitle("Cruise Control", fontsize=12)

global x0,TFinal,m,f0,f1,f2,vd,v0,eps,gamma,p_sc,Fr
#Setting parameter values. Forcing type double to prevent computational errors
x0 = np.array([[900.0],[20.0],[100.0],[1000.0],[13.89]])
TFinal = 20
#Setting values used by model and Controller
m = 1650.0
f0 = 0.1
f1 = 5.0
f2 = 0.25
vd = 24.0
v0 = 13.89
eps = 10.0
gamma = 1.0
p_sc = 1e-5
		

def model(x,u):
	# INPUTS
	# x = state vector
	#	x(0:2) = [pos, vel, distave of x(5) - x(1)
	Fr = f0 + f1*x[1] + f2*x[1]**2
	
	dx = [0,1,2,3,4]#np.empty([5,1])
	dx[0] = x[1]
	dx[1] = -1/m*Fr + 1/m*u[0]
	dx[2] = x[4] - x[1]
	dx[3] = v0
	dx[4] = 0
	return dx


def controller(x):
	Fr = f0 + f1*x[1] + f2*x[1]**2
		
	# CLF
	phi0 = -2 * (x[1]-vd) * Fr/m + eps*(x[1]-vd)**2
	phi1 = 2 * (x[1]-vd)/m

	# CBF
	z = x[3] - x[0]
	h = x[2] - 1.8*x[1]
	Bf = -math.log(h/(1+h))
	denum = m*(1-1.8*x[1]+z)*(-1.8*x[1]+z)
	LfB = -(1.8*Fr+m*(x[4]-x[2]))/denum
	LgB = -1.8/denum

	# Quadratic Problem Solving
	A_clf = [phi1, -1]
	b_clf = -phi0
	A_cbf = [LgB, 0]
	b_cbf = -LfB + gamma/Bf
	H_acc = 2*np.array([[1/m**2,0],[0,p_sc]])
	F_acc = -2*np.array([[Fr/m**2],[0]])
	

	n = H_acc.shape[1]


	A_input = np.array([A_clf,A_cbf])
	b_input = np.array([b_clf,b_cbf])
	P = matrix(H_acc,tc='d')
	q = matrix(F_acc,tc='d')
	G = matrix(A_input,tc='d')
	h = matrix(b_input,tc='d')

	
	sol = solvers.qp(P,q,G,h)
	u = (sol['x'])
	return u


def control(x,t):
	u = controller(x)
	return u	


def ode_func(x,t):
	dx = model(x,control(x,t))
	return dx


def ode_solver(TFinal):
	time = np.linspace(0,TFinal,1000)
	x_init = [900.0,20.0,100.0,1000.0,13.89]
	x = odeint(ode_func,x_init,time)		
	return x,time


[x,time] = ode_solver(TFinal)


plt.plot(time,x)

plt.show()











































































