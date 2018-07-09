#!/usr/bin/python
#coding: utf-8

import warnings
warnings.filterwarnings("ignore", message="numpy.dtype size changed")
warnings.filterwarnings("ignore", message="numpy.ufunc size changed")

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from scipy.integrate import odeint


def van_der_Pol(y,t):
	dydt = 2*t #np.matrix([y[1]],[(1-y[0]^2)*y[1]-y[0]])
	return dydt

t = np.linspace(0,5,100)
y0 = [0]
sol = odeint(van_der_Pol,y0,t)

#print(sol)
plt.plot(t,sol)
plt.show()










































