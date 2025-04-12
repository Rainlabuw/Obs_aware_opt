import numpy as np
import cvxpy as cvx
import matplotlib.pyplot as plt
import time
import cProfile
import re
from scipy.integrate import odeint, solve_ivp
from scipy.linalg import expm
import control 
import matplotlib.pyplot as plt

xlqr = np.load('x_lqr3.npy')
xobs = np.load('x_obs3.npy')


[nx,T] = xlqr.shape

# working parameters
TOL = 1e-5
tf = 15
dt = 0.1
round = lambda x: np.round(x, 3)
seed = 25
np.random.seed(seed)

mu = 3.986e14
a = 6.9e6
n = np.sqrt(mu/a**3)
nx = 6
nu = 3

A = np.array([[0, 0, 0, 1, 0, 0],
              [0, 0, 0 ,0, 1, 0],
              [0, 0, 0, 0, 0, 1],
              [3*n**2, 0, 0, 0, 2*n, 0],
              [0, 0, 0, -2*n, 0, 0],
              [0, 0, -n**2, 0, 0, 0]])

B = np.array([[0, 0, 0],
              [0, 0, 0],
              [0, 0, 0],
              [1, 0, 0],
              [0, 1, 0],
              [0, 0, 1]])

C = np.array([[1, 0, 0, 0, 0, 0],
              [0, 1, 0, 0, 0, 0],
              [0, 0, 1, 0, 0, 0]])


Qx = 10
Qv = 10

Q = np.block([[Qx*np.eye(3),np.zeros((3,3))],[np.zeros((3,3)),Qv*np.eye(3)]])
R = 0.01*np.eye(3)

D = expm(np.block([[A,B],[np.zeros((nu,nx)),np.zeros((nu,nu))]])*dt)

Ad = D[:nx,:nx]
Bd = D[:nx,nx:]

p = [0.9,0.9,0.9,0.7,0.7,0.7]

LT = control.place(Ad.T, C.T, p)
L = LT.T


def f(x,t,u) -> np.ndarray:
    dx = np.array([x[3],
                   x[4],
                   x[5],
                   3*n**2*x[0] + 2*n*x[4] + u[0],
                   -2*n*x[3] + u[1],
                   -n**2*x[2] + u[2]])
    return dx

def fdiscrete(x0, u):
    t = np.linspace(0,dt,11)
    sol = odeint(f,x0,t, args=(u,))
    x = sol[-1,:]
    return x


def df_dx(x: np.ndarray, u: np.ndarray) -> np.ndarray:
    return Ad

def df_du(x: np.ndarray, u: np.ndarray) -> np.ndarray:
    return Bd


t = np.linspace(0,151,152)


x0 = np.array([20,20,20,0,10,10])
x_des = np.array([0,0,0,0,0,0])


# def uncert_size(x):
#     a1 = 1
#     b1 = 5
#     p = np.array([-10,0,0])
#     xr = x/np.linalg.norm(x)
    
#     # dot  = p@xr
#     dot  = p@xr
#     pp = 10
    
#     # return 1 + dot*pp
#     return pp * np.arccos(dot)+ b1
#     # return np.linalg.norm(dot) + b1
#     # return dot*pp

def uncert_size(x):
    a1 = 1
    b1 = 1
    # p = np.array([1,0,0])

    # p = np.array([-1,-1,1])
    p = np.array([10,0,0])
    c = -p

    p = p/np.linalg.norm(p)
    xr = x/np.linalg.norm(x)
    
    # dot  = p@xr
    # dot  = p@x
    # dot  = cvx.log_det(p@x)
    pp = 10

    eps = 0.001
    # return 1 + dot*pp
    # return pp * np.arccos(dot)**2
    # return dot
    # y = df_dx(xk,uk)**k
    # n = np.linalg.norm(Ad)**k
    J =  0.1*(x-c.T).T@(x-c.T).T 
    return J 
# def L_param(x):
#     for i in range

def obs_cost(x):
    return uncert_size(x)

def obs_sample(x):
    r = 0.5 - np.random.uniform(0,1,(3,))
    
    return x + 0.1*uncert_size(x)*r



def filtered_traj(x0, x_d):
    
    
    x = np.zeros((nx,T))
    u = np.zeros((nu,T-1))
    y = np.zeros((nu,T-1))
    x[:,0] = x0
    y = np.zeros((3,T))
    
    K, _, _ = control.dlqr(Ad, Bd, Q, R)
    
    for k in range(T-1):
        xk = x[:,k]
        x_des = x_d[:,k]
        
        uk = -K@(xk - x_des)
        # print(x_des)
        y[:,k] = obs_sample(C@xk)
        x[:,k + 1] = (Ad - L@C)@(xk) + Bd@uk  + L@(y[:,k])
        u[:,k] = uk
        
    return x, u, y







fig1 = plt.figure(1)
fig2 = plt.figure(2)
ax1 = fig1.add_subplot(111)
ax2 = fig2.add_subplot(111)


for k in range(4):

    # # Analysis
    # NN = 10000
    
    NN = 10000
    X1 = np.zeros((nx,T,NN))
    # X2 = np.zeros((nx,T,NN))

    e1 = np.zeros((1,T,NN))
    # e2 = np.zeros((1,T,NN))

    xobs = np.load('xfilt'+str(k)+'.npy')
    # print(xobs)


    for i in range(NN):
        x1,u1,y1 = filtered_traj(xobs[:,0],xobs)
        # x2,u2,y2 = filtered_traj(xobs[:,0],xobs)
        
        X1[:,:,i] = x1 - xobs
        # X2[:,:,i] = x2 - xobs
        e1[:,:,i] = np.linalg.norm(x1 - xobs,2,0)        


        # X1[:,:,i] = x1
        # X2[:,:,i] = x2

    # V1 = np.zeros((6,T+1))
    # V2 = np.zeros((6,T+1))

    # e1m = np.max(e1)
    # e2m = np.max(e2)


    V1 = np.zeros((1,T+1))
    E1 = np.zeros((1,T+1))
    e1m = np.zeros((1,T+1))

    print(X1.shape)
    for j in range(T):
        a=X1[:,j,:]    
        # print(a)
        # print(a.shape)
        # print(np.var(X1[:,j,:]))
        V1[:,j] = np.var(X1[:,j,:])

        
        E1[:,j] = np.mean(e1[:,j,:])

        
        e1m[:,j] = np.max(e1[:,j,:])
    
    # print(V1)
    # print(0)
    # print(E1)
    # print(e1m)

        
        






    ax1.plot(t[:-2],V1[:,:-2].T, color= [0.3*k,0, 0.3*k], label = r'$\lambda_{obs} = $'+ str(k),linewidth=3)
    ax1.set_ylabel('Trace of State variance', fontsize = 20)
    ax1.set_xlabel('Time', fontsize = 20)
    ax1.legend(fontsize = 16)



    ax2.plot(t[:-2],E1[:,:-2].T, ls = '--',color= [0.3*k,0, 0.3*k],linewidth=3.0,label =  '_nolegend_')
    ax2.plot(t[:-2],e1m[:,:-2].T, color= [0.3*k,0, 0.3*k],label =  '_nolegend_')
    ax2.set_ylabel('Tracking error norm', fontsize = 20)
    ax2.set_xlabel('Time', fontsize = 20)
    
    


ax2.plot(t[:-2],E1[:,:-2].T, ls = '--',color= [0.3*k,0, 0.3*k],linewidth=3.0, label =  'mean error')
ax2.plot(t[:-2],e1m[:,:-2].T, color= [0.3*k,0, 0.3*k], label = 'max error')
ax2.set_ylabel('Tracking error norm', fontsize = 20)
ax2.set_xlabel('Time', fontsize = 20)
ax2.legend(fontsize = 16, loc = 'upper right')


ax1.tick_params(axis='both', which='major', labelsize=14)
ax1.tick_params(axis='both', which='minor', labelsize=10)

ax2.tick_params(axis='both', which='major', labelsize=14)
ax2.tick_params(axis='both', which='minor', labelsize=10)

ax2.grid()
ax1.grid()

plt.show()
