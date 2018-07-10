#!/usr/bin/python
#coding: utf-8

#Ignoring warnings that arise due to scipy being built over the new numpy
import warnings
warnings.filterwarnings("ignore", message="numpy.dtype size changed")
warnings.filterwarnings("ignore", message="numpy.ufunc size changed")

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from scipy.integrate import odeint




def exponential(y,t):
	dydt = 2*t #np.matrix([y[1]],[(1-y[0]^2)*y[1]-y[0]])
	return dydt

t = np.linspace(0,5,100)
y0 = [0]
sol = odeint(exponential,y0,t)

plt.figure(1)
plt.plot(t,sol)
plt.xlabel('Time')
plt.ylabel('Value')
plt.title('dy/dt = 2 * t')




def van_der_Pol(y,t):
	dydt = [y[1],(1-y[0]**2)*y[1]-y[0]]
	return dydt

y1 = [2,0]
t1 = np.linspace(0,20,100)
sol1 = odeint(van_der_Pol,y1,t1)

plt.figure(2)
plt.plot(t1,sol1)
plt.xlabel('Time')
plt.ylabel('Value')
plt.title('Van Der Pol Differential Equation')


print(sol1)

plt.show()



































