import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import math as m
#function
def model(y,t):
    vt = 300
    nu = 1.5
    vm = nu * vt
    out1 = vt * np.cos(y[9] - y[1]) - vm
    out2 = (vt * np.sin(y[9] - y[1] - vm * np.sin(y[6] - y[1]))) / y[0]
    out3 = vm * np.cos(y[6])
    out4 = vm * np.sin(y[6])
    out5 = vt * np.cos(y[9])
    out6 = vt * np.sin(y[9])
    out7 = out2
    out8 = out2 * out2 * y[0]
    out9 = -(y[7] + vm) * out2
    out10 = m.radians(30)
    out = [out1, out2, out3, out4, out5, out6, out7, out8, out9, out10]
    return out
# Fixed values
alpha_t0 = np.pi/4
print(alpha_t0)
alpha_m0 = np.pi/6
vt = 300
nu = 1.5
vm = nu*vt
#Initial values
R0 = 5000
theta0 = np.pi/6
Vr0 = vt*np.cos(alpha_t0 - theta0) - vm
Vtheta0 = vt*np.sin(alpha_t0 - theta0)
tspan = np.linspace(0,40,num=4000)
k = (R0*(1+np.cos(alpha_t0-theta0))**(nu))/((np.sin(alpha_t0-theta0 ))**(nu-1));
y0 = [R0, theta0, 0, 0, R0*np.cos(theta0), R0*np.sin(theta0),theta0, Vr0, Vtheta0,alpha_t0]
print(y0)
#time points
t=np.linspace(0,40,num=4000)
# ode solver equation
y=odeint(model,y0,t)
plt.plot(t,y[:,0])
plt.plot(y[:,3],y[:,4],y[:,5],y[:,6])
plt.show()
