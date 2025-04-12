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




'''
# Plot the sphere
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot3D(xlqr[0,:], xlqr[1,:], xlqr[2,:], '-go', linewidth = 3 )
ax.set_xlim(-7, 13)
ax.set_ylim(-7, 13)
ax.set_zlim(-7, 13)
ax.quiver(-10, 0, 0, 1, 0, 0, length=10, normalize=False,color = 'red', alpha = .8, lw = 3)
plt.show()
'''
# working parameters

TOL = 1e-5
N = 150

# N = 100

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


Qx = 1
Qv = 2

Q = np.block([[Qx*np.eye(3),np.zeros((3,3))],[np.zeros((3,3)),Qv*np.eye(3)]])
R = 1*np.eye(3)

D = expm(np.block([[A,B],[np.zeros((nu,nx)),np.zeros((nu,nu))]])*dt)

Ad = D[:nx,:nx]
Bd = D[:nx,nx:]



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


x0 = np.array([7,20,10,10,-5,5])
x_des = np.array([0,0,0,0,0,0])
# x0 = np.array([0,-20,10,10,-5,0])
# x_des = np.array([0,0,0,0,0,0])

penalty_lambda = 1*1e4
state_lambda = 1*1e-1
control_lambda = 1*1e-1
final_state_lambda = 1*1e-1
# obs_lambda = 5*1e3
obs_lambda = 1*1e1


print(np.linalg.norm(Ad)**5)
def uncert_size(x,u,k):
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
    J =  2*(C@x-c.T).T@(C@x-c.T).T + np.linalg.norm(C@x-c.T)*eps 
    return J  
    



# def L_param(x):
#     for i in range

def obs_cost(x,u,k):
    return uncert_size(x,u,k)


def cvx_uncert_size(x,u,k):
    a1 = 1
    b1 = 1
    p = np.array([1,0,0])
    c = -p
    # xr = x/cvx.norm2(x)

    # p = np.array([-1,-1,1])
    p = p/np.linalg.norm(p)
    
    # dot  = p@xr
    # dot  = cvx.convolve(p,C@x)
    # dot  = p@x
    # dot  = cvx.log_det(p@x)
    pp = 10
    eps=1
    
    n = np.linalg.norm(Ad)**k
    # return dot
    # return (cvx.norm2(dot) + b1)
    J =  2*cvx.quad_form(C@x-c.T,np.eye(3)) + cvx.norm2(C@x-c.T)*eps
    return J 
    


def cvx_obs_cost(x,u,k):
    return cvx_uncert_size(x,u,k)



# def trajectory(x0, u):
#     x = np.zeros((6, N + 1))
#     x[:,0] = x0
#     for k in range(N):
#         sol = odeint(f, x[:,k],[], args=(u))
#         x[:,k + 1] = sol[-1,:]
#     return x


def cvx_penalty(v):
    return cvx.norm(v, 1)

def penalty(v):
    return np.linalg.norm(v, 1)

def cost(x, u):
    J = 0 
    for k in range(N):
        xk = x[:,k]
        uk = u[:,k]
        xk_p1 = x[:, k + 1]
        J += state_lambda*(xk - x_des).T@Q@(xk - x_des) + \
            control_lambda*uk.T@R@uk + \
            obs_lambda*obs_cost(xk,uk,k) + \
            penalty_lambda*penalty(xk_p1 - fdiscrete(xk, uk)) + \
            penalty_lambda*np.abs(s(xk))
    xN = x[:,N]
    J += final_state_lambda*(xN - x_des).T@Q@(xN - x_des)
    return J

def true_cost(x, u):
    J = 0 
    for k in range(N):
        xk = x[:,k]
        uk = u[:,k]
        xk_p1 = x[:, k + 1]
        J += state_lambda*(xk - x_des).T@Q@(xk - x_des) + \
            control_lambda*uk.T@R@uk + obs_lambda*obs_cost(xk,uk,k)

    xN = x[:,N]
    J += final_state_lambda*(xN - x_des).T@Q@(xN - x_des)
    return J



def cvx_linearized_cost(x, u, x_des, eta, xi, v, s_prime):
    J = 0
    for k in range(N):
        xk = x[:,k]
        uk = u[:,k]
        etak = eta[:,k]
        xik = xi[:,k]
        vk = v[:,k]
        sk = s_prime[:,k]
        J += state_lambda*cvx.quad_form(xk + etak - x_des,Q) + \
            control_lambda*cvx.quad_form(uk + xik,R) +\
            obs_lambda*cvx_obs_cost((xk + etak),uk + xik,k)+ \
            penalty_lambda*cvx_penalty(vk) + \
            penalty_lambda*cvx_penalty(sk)
    xN = x[:,N]
    etaN = eta[:,N]
    J += final_state_lambda*cvx.quad_form(xN + etaN - x_des,Q)
    return J


def linearized_cost(x, u, x_des, eta, xi, v, s_prime):
    J = 0

    for k in range(N):
        xk = x[:,k]
        uk = u[:,k]
        etak = eta[:,k]
        xik = xi[:,k]
        vk = v[:,k]
        sk = s_prime[:,k]
        J += state_lambda*(xk + etak - x_des).T@Q@(xk + etak - x_des) + \
            control_lambda*(uk + xik).T@R@(uk + xik) +\
            obs_lambda*obs_cost((xk + etak),uk + xik,k) + \
            penalty_lambda*penalty(vk) + \
            penalty_lambda*penalty(sk)
    xN = x[:,N]
    etaN = eta[:,N]
    J += final_state_lambda*(xN + etaN - x_des).T@Q@(xN + etaN - x_des)
    return J

rad = 5

rad = 10
def s(x):
    return rad**2 - (C@x - C@x_des).T@(C@x - C@x_des)

def dS_dx(x):
    return  - 2*C.T@C@x + 2*C.T@C@x_des


def init_traj(x0, x_des):
    x = np.zeros((nx,N + 1))
    u = np.zeros((nu,N))
    x[:,0] = x0
    q1 = np.zeros((1,N + 1))
    # q1[:,0] = uncert_size(C@x0)
    K, _, _ = control.dlqr(Ad, Bd, Q, R)
    for k in range(N):
        xk = x[:,k]
        uk = -K@xk
        # x[:,k + 1] = f(xk, uk)
        x[:,k + 1] = Ad@(xk - x_des) + Bd@uk
        u[:,k] = uk
        q1[:,k + 1] = uncert_size(xk,uk,k)
    return x, u, q1

x,u, q1 = init_traj(x0,x_des)

# print(x.shape)
# print(u.shape)




# Generate data for the sphere
us = np.linspace(0, 2 * np.pi, 100)
vs = np.linspace(0, np.pi, 100)
xs = rad*np.outer(np.cos(us), np.sin(vs))
ys = rad*np.outer(np.sin(us), np.sin(vs))
zs = rad*np.outer(np.ones(np.size(us)), np.cos(vs))





def convex_optimal_control_subproblem(x, u, r):
    eta = cvx.Variable((nx, N + 1))
    xi = cvx.Variable((nu, N))
    v = cvx.Variable((nx, N))
    s_prime = cvx.Variable((1, N))
    constraints = [eta[:,0] == np.zeros(nx)]
    
    for k in range(N):
        xk = x[:,k]
        xk_p1 = x[:, k + 1]
        
        uk = u[:,k]
        
        etak = eta[:,k]
        etak_p1 = eta[:,k + 1]
        
        xik = xi[:,k]
        
        Ak = df_dx(xk, uk)
        Bk = df_du(xk, uk)

        Sk = dS_dx(xk)
        
        vk = v[:,k]
        sk = s_prime[:,k]
        
        constraints.extend([
            # etak_p1 + xk_p1 == f(xk, uk) + Ak@etak + Bk@xik + vk,
            etak_p1 + xk_p1 == fdiscrete(xk, uk) + Ak@etak + Bk@xik + vk,
            cvx.norm(xik, 2) <= r,
            s(xk) + Sk@etak - sk <= 0,
            sk >= 0
        ])

    J = cvx_linearized_cost(x, u, x_des, eta, xi, v, s_prime)
    problem = cvx.Problem(cvx.Minimize(J), constraints)
    opt_value = problem.solve(solver=cvx.CLARABEL, verbose = False)
    return eta.value, xi.value, v.value, s_prime.value







# r = 0.1
r = 1
rl = 0
alpha = 2
beta = 3.2
eps_tol = 1e-3
rho0 = 0
rho1 = .25
rho2 = .7



k = 0
cost_hist = []
x_data=[]
x_data.append(x)
while True:
    
    # step 1     
    eta, xi, v, s_prime = convex_optimal_control_subproblem(x, u, r)
    
    # step 2    
    Delta_J = cost(x, u) - cost(x + eta, u + xi)
    Delta_L = cost(x, u) - linearized_cost(x, u, x_des, eta, xi, v, s_prime)
    # yy = linearized_cost(x, u, x_des, eta, xi, v, s_prime)

    if np.abs(Delta_J) < eps_tol:
        print("|Delta_J| < eps_tol")
        break
    else:
        rho_k = np.abs(Delta_J)/Delta_L
    print(f"step: {k}, \
        log10_r: {np.log10(r)}, \
        log10_rho_k: {np.log10(rho_k)}, \
        true cost: {true_cost(x, u)}, \
        Euc_cost: {cost(x, u)}, \
        log10_Delta_J: {np.log10(np.abs(Delta_J))}")
    cost_hist.append(cost(x, u))
    # step 3
    
    if rho_k < rho0:
        r = r/alpha
    else:
        x = x + eta
        u = u + xi

        x_data.append(x)
        if rho_k < rho1:
            r = r/alpha
        elif rho_k >= rho2:
            r = r*beta
        
        r = max(r, rl)
        k = k + 1
    
    if k>20:
        break




i=5
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot3D(x[0,:], x[1,:], x[2,:], linestyle = '-', color= [0,0, 1-1/(i+1)], linewidth = 2 )
ax.scatter(x[0,:], x[1,:], x[2,:], marker = 'o', color= [0,0, 1-1/(i+1)], linewidth = 1 )

# ax.plot3D(xlqr[0,:], xlqr[1,:], xlqr[2,:], linestyle = '-', color= [0,1, 1-1/(i+1)], linewidth = 2 )
# ax.scatter(xlqr[0,:], xlqr[1,:], xlqr[2,:], marker = 'o', color= [0,0, 1-1/(i+1)], linewidth = 1 )

ax.set_xlim(-7, 21)
ax.set_ylim(-7, 21)
ax.set_zlim(-7, 21)
ax.quiver(-10, 0, 0, 1, 0, 0, length=10, normalize=False,color = 'red', alpha = .8, lw = 3)
# ax.quiver(10, 10, 10, 0, 0, 0, length=20, normalize=False,color = 'red', alpha = .8, lw = 3)
ax.scatter(x[0,0], x[1,0], x[2,0], marker = 'o',color='black', linewidth = 6 )

ax.scatter(x[0,-1], x[1,-1], x[2,-1], marker = 'o', color = 'green', linewidth = 6 )
# ax.scatter(xlqr[0,-1], xlqr[1,-1], xlqr[2,-1], marker = 'o', color = 'green', linewidth = 6 )

ax.scatter(0, 0, 0, marker = 'o', color = 'blue', linewidth = 6 )
ax.plot_surface(xs, ys, zs, alpha=0.3)  # Set alpha for transparency
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()


# np.save('xobs',x)





