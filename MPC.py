
######################################################################
# Porting MPC code  from C++  to python
#
# Author: Ghicheon Lee
# date  : 2019.8.11
#
# Original C++ MPC code is from Udacity Self Driving Car Course.
# it's slow.But it might be better to understand what's going on.
######################################################################

import numpy as np    # np.polyfit
import math
from scipy.optimize import minimize
import matplotlib.pyplot as plt

from scipy.optimize import rosen, rosen_der

N =  10  #XXXXXXXXXXXXXXX
dt = 0.05

Lf = 2.67

ref_v = 40

n_variables = N * 6 + (N - 1) * 2       #  60 + 18   
n_constraints = N * 6

x_start      = 0
y_start      = x_start + N
psi_start    = y_start + N
v_start      = psi_start + N
cte_start    = v_start + N
epsi_start   = cte_start + N
delta_start  = epsi_start + N
a_start      = delta_start + N - 1

g_state=None
g_coeffs = [0,0]

def objective(variables):
    cost = 0

    for t in range(N-2):
        cost += 1000*pow(variables[delta_start + t + 1] - variables[delta_start + t], 2)
        cost += pow(variables[a_start + t + 1]     - variables[a_start + t], 2)

        cost += pow(variables[delta_start + t], 2)
        cost += pow(variables[a_start + t], 2)

        cost += pow(variables[cte_start + t], 2)
        cost += pow(variables[epsi_start + t], 2)
        cost += pow(variables[v_start + t] - ref_v, 2)

    for t in range(N-2,N-1):
        cost += pow(variables[delta_start + t], 2)
        cost += pow(variables[a_start + t], 2)

    for t in range(N-1,N):
        cost += pow(variables[cte_start + t], 2)
        cost += pow(variables[epsi_start + t], 2)
        cost += pow(variables[v_start + t] - ref_v, 2)


    return cost


def return_all(variables, t):
    x     = float(variables[x_start + t])
    y     = float(variables[y_start + t])
    psi   = float(variables[psi_start + t])
    v     = float(variables[v_start + t])
    cte   = float(variables[cte_start + t])
    epsi  = float(variables[epsi_start + t])
    return  x,y,psi,v,cte,epsi

def constraint01( variables):
    return variables[x_start] - g_state[0]

def constraint02( variables):
    return variables[y_start] - g_state[1]

def constraint03( variables):
    return variables[psi_start] - g_state[2]

def constraint04( variables):
    return variables[v_start] - g_state[3]

def constraint05( variables):
    return variables[cte_start] - g_state[4]

def constraint06( variables):
    return variables[epsi_start] - g_state[5]


def core_constraint1(variables, t):
    global g_coeffs

  #  x1 ,y1 ,psi1 ,v1 ,cte1 ,epsi1 = return_all(variables, t)   #t+1
  #  x0 ,y0 ,psi0 ,v0 ,cte0 ,epsi0 = return_all(variables, t-1) #t

    x1     = float(variables[x_start + t])
    x0     = float(variables[x_start + t-1])
    v0     = float(variables[v_start + t-1])
    psi0   = float(variables[psi_start + t-1])

    delta0 = float(variables[delta_start + t - 1])
    a0     = float(variables[a_start + t - 1])
    
  #  f0     = float(g_coeffs[0] + g_coeffs[1] * x0)
  #  psides0= float(math.atan(g_coeffs[1]) )
    
    return   x1 - (x0 + v0 * math.cos(psi0) * dt)
    #fg[y_start + t]    = y1 - (y0 + v0 * math.sin(psi0) * dt)
    #fg[psi_start + t]  = psi1 - (psi0 + v0 * delta0 / Lf * dt)
    #fg[v_start + t]    = v1 - (v0 + a0 * dt)
    #fg[cte_start + t]  = cte1 - ((f0 - y0) + (v0 * math.sin(epsi0) * dt))
    #fg[epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt)


def core_constraint2(variables,t):
    global g_coeffs

    y1     = float(variables[y_start + t])
    y0     = float(variables[y_start + t-1])
    v0     = float(variables[v_start + t-1])
    psi0   = float(variables[psi_start + t-1])

    #fg[x_start + t]    = x1 - (x0 + v0 * math.cos(psi0) * dt)
    return  y1 - (y0 + v0 * math.sin(psi0) * dt)
    #fg[psi_start + t]  = psi1 - (psi0 + v0 * delta0 / Lf * dt)
    #fg[v_start + t]    = v1 - (v0 + a0 * dt)
    #fg[cte_start + t]  = cte1 - ((f0 - y0) + (v0 * math.sin(epsi0) * dt))
    #fg[epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt)
        
def core_constraint3( variables,t):

    delta0 = float(variables[delta_start + t - 1])
    psi1   = float(variables[psi_start + t])
    psi0   = float(variables[psi_start + t-1])
    v0     = float(variables[v_start + t-1])
    #fg[x_start + t]    = x1 - (x0 + v0 * math.cos(psi0) * dt)
    #return  y1 - (y0 + v0 * math.sin(psi0) * dt)
    return  psi1 - (psi0 + v0 * delta0 / Lf * dt)
    #fg[v_start + t]    = v1 - (v0 + a0 * dt)
    #fg[cte_start + t]  = cte1 - ((f0 - y0) + (v0 * math.sin(epsi0) * dt))
    #fg[epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt)

def core_constraint4( variables,t):
    v1     = float(variables[v_start + t])
    v0     = float(variables[v_start + t-1])
    a0     = float(variables[a_start + t - 1])

    #fg[x_start + t]    = x1 - (x0 + v0 * math.cos(psi0) * dt)
    #return  y1 - (y0 + v0 * math.sin(psi0) * dt)
    #fg[psi_start + t]  = psi1 - (psi0 + v0 * delta0 / Lf * dt)
    return  v1 - (v0 + a0 * dt)
    #fg[cte_start + t]  = cte1 - ((f0 - y0) + (v0 * math.sin(epsi0) * dt))
    #fg[epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt)

def core_constraint5( variables,t):
    x0     = float(variables[x_start + t-1])

    cte1   = float(variables[cte_start + t])
    f0     = float(g_coeffs[0] + g_coeffs[1] * x0)
    y0     = float(variables[y_start + t-1])
    v0     = float(variables[v_start + t-1])
    epsi0  = float(variables[epsi_start + t-1])
    #fg[x_start + t]    = x1 - (x0 + v0 * math.cos(psi0) * dt)
    #return  y1 - (y0 + v0 * math.sin(psi0) * dt)
    #fg[psi_start + t]  = psi1 - (psi0 + v0 * delta0 / Lf * dt)
    #fg[v_start + t]    = v1 - (v0 + a0 * dt)
    return  cte1 - ((f0 - y0) + (v0 * math.sin(epsi0) * dt))
    #fg[epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt)

def core_constraint6( variables,t):

    v0     = float(variables[v_start + t-1])
    psides0= float(math.atan(g_coeffs[1]) )
    psi0   = float(variables[psi_start + t-1])
    epsi1  = float(variables[epsi_start + t])

    delta0 = float(variables[delta_start + t - 1])
    #fg[x_start + t]    = x1 - (x0 + v0 * math.cos(psi0) * dt)
    #return  y1 - (y0 + v0 * math.sin(psi0) * dt)
    #fg[psi_start + t]  = psi1 - (psi0 + v0 * delta0 / Lf * dt)
    #fg[v_start + t]    = v1 - (v0 + a0 * dt)
    #return  cte1 - ((f0 - y0) + (v0 * math.sin(epsi0) * dt))
    return  epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt)


    #for t in range(1,N):

#t 1
def constraint11( variables): return core_constraint1(variables,1)
def constraint12( variables): return core_constraint2(variables,1)
def constraint13( variables): return core_constraint3(variables,1)
def constraint14( variables): return core_constraint4(variables,1)
def constraint15( variables): return core_constraint5(variables,1)
def constraint16( variables): return core_constraint6(variables,1)
#t 2
def constraint21( variables): return core_constraint1(variables,2)
def constraint22( variables): return core_constraint2(variables,2)
def constraint23( variables): return core_constraint3(variables,2)
def constraint24( variables): return core_constraint4(variables,2)
def constraint25( variables): return core_constraint5(variables,2)
def constraint26( variables): return core_constraint6(variables,2)
#t 3
def constraint31( variables): return core_constraint1(variables,3)
def constraint32( variables): return core_constraint2(variables,3)
def constraint33( variables): return core_constraint3(variables,3)
def constraint34( variables): return core_constraint4(variables,3)
def constraint35( variables): return core_constraint5(variables,3)
def constraint36( variables): return core_constraint6(variables,3)
#t 4
def constraint41( variables): return core_constraint1(variables,4)
def constraint42( variables): return core_constraint2(variables,4)
def constraint43( variables): return core_constraint3(variables,4)
def constraint44( variables): return core_constraint4(variables,4)
def constraint45( variables): return core_constraint5(variables,4)
def constraint46( variables): return core_constraint6(variables,4)
#t 5
def constraint51( variables): return core_constraint1(variables,5)
def constraint52( variables): return core_constraint2(variables,5)
def constraint53( variables): return core_constraint3(variables,5)
def constraint54( variables): return core_constraint4(variables,5)
def constraint55( variables): return core_constraint5(variables,5)
def constraint56( variables): return core_constraint6(variables,5)
#t 6
def constraint61( variables): return core_constraint1(variables,6)
def constraint62( variables): return core_constraint2(variables,6)
def constraint63( variables): return core_constraint3(variables,6)
def constraint64( variables): return core_constraint4(variables,6)
def constraint65( variables): return core_constraint5(variables,6)
def constraint66( variables): return core_constraint6(variables,6)
#t 7
def constraint71( variables): return core_constraint1(variables,7)
def constraint72( variables): return core_constraint2(variables,7)
def constraint73( variables): return core_constraint3(variables,7)
def constraint74( variables): return core_constraint4(variables,7)
def constraint75( variables): return core_constraint5(variables,7)
def constraint76( variables): return core_constraint6(variables,7)
#t 8
def constraint81( variables): return core_constraint1(variables,8)
def constraint82( variables): return core_constraint2(variables,8)
def constraint83( variables): return core_constraint3(variables,8)
def constraint84( variables): return core_constraint4(variables,8)
def constraint85( variables): return core_constraint5(variables,8)
def constraint86( variables): return core_constraint6(variables,8)
#t 9
def constraint91( variables): return core_constraint1(variables,9)
def constraint92( variables): return core_constraint2(variables,9)
def constraint93( variables): return core_constraint3(variables,9)
def constraint94( variables): return core_constraint4(variables,9)
def constraint95( variables): return core_constraint5(variables,9)
def constraint96( variables): return core_constraint6(variables,9)


#    state, coeffs
def MPCSolve(state,coeffs):
    global g_coeffs
    global g_state
    g_state = state

    x    = state[0]
    y    = state[1]
    psi  = state[2]
    v    = state[3]
    cte  = state[4]
    epsi = state[5]

    #variables
    variables = np.zeros(n_variables)  

    variables[x_start]    = x
    variables[y_start]    = y
    variables[psi_start]  = psi
    variables[v_start]    = v
    variables[cte_start]  = cte
    variables[epsi_start] = epsi
    g_coeffs[0] = coeffs[0]
    g_coeffs[1] = coeffs[1]

    #print(objective(state)) #test

    xbound      = (x,x)
    ybound      = (y,y)
    psibound    = (psi,psi)
    vbound      = (v,v)
    ctebound    = (cte,cte)
    epsibound   = (epsi,epsi)

    bound       = (-1.0e19 , 1.0e19)
    bound_delta = (-0.436332, 0.436332)
    bount_a     = (-1.0 , 1.0)
    
    #bounds = (bound,bound,bound,bound,bound,bound,bound,bound,bound,bound,bound,bound,bound_delta,bount_a)
    bounds =  (bound,bound,bound,bound,bound,bound,\
               bound,bound,bound,bound,bound,bound,\
               bound,bound,bound,bound,bound,bound,\
               bound,bound,bound,bound,bound,bound,\
               bound,bound,bound,bound,bound,bound,\
               bound,bound,bound,bound,bound,bound,\
               bound,bound,bound,bound,bound,bound,\
               bound,bound,bound,bound,bound,bound,\
               bound,bound,bound,bound,bound,bound,\
               bound,bound,bound,bound,bound,bound,\
               bound_delta, bound_delta, bound_delta, bound_delta, bound_delta, bound_delta, bound_delta, bound_delta,bound_delta,
               bount_a, bount_a, bount_a, bount_a, bount_a, bount_a, bount_a, bount_a, bount_a) 
    
    
    con01 = { 'type': 'eq' , 'fun': constraint01 }
    con02 = { 'type': 'eq' , 'fun': constraint02 }
    con03 = { 'type': 'eq' , 'fun': constraint03 }
    con04 = { 'type': 'eq' , 'fun': constraint04 }
    con05 = { 'type': 'eq' , 'fun': constraint05 }
    con06 = { 'type': 'eq' , 'fun': constraint06 }

    con11 = { 'type': 'eq' , 'fun': constraint11}
    con12 = { 'type': 'eq' , 'fun': constraint12}
    con13 = { 'type': 'eq' , 'fun': constraint13}
    con14 = { 'type': 'eq' , 'fun': constraint14}
    con15 = { 'type': 'eq' , 'fun': constraint15}
    con16 = { 'type': 'eq' , 'fun': constraint16}
    con21 = { 'type': 'eq' , 'fun': constraint21}
    con22 = { 'type': 'eq' , 'fun': constraint22}
    con23 = { 'type': 'eq' , 'fun': constraint23}
    con24 = { 'type': 'eq' , 'fun': constraint24}
    con25 = { 'type': 'eq' , 'fun': constraint25}
    con26 = { 'type': 'eq' , 'fun': constraint26}
    con31 = { 'type': 'eq' , 'fun': constraint31}
    con32 = { 'type': 'eq' , 'fun': constraint32}
    con33 = { 'type': 'eq' , 'fun': constraint33}
    con34 = { 'type': 'eq' , 'fun': constraint34}
    con35 = { 'type': 'eq' , 'fun': constraint35}
    con36 = { 'type': 'eq' , 'fun': constraint36}
    con41 = { 'type': 'eq' , 'fun': constraint41}
    con42 = { 'type': 'eq' , 'fun': constraint42}
    con43 = { 'type': 'eq' , 'fun': constraint43}
    con44 = { 'type': 'eq' , 'fun': constraint44}
    con45 = { 'type': 'eq' , 'fun': constraint45}
    con46 = { 'type': 'eq' , 'fun': constraint46}
    con51 = { 'type': 'eq' , 'fun': constraint51}
    con52 = { 'type': 'eq' , 'fun': constraint52}
    con53 = { 'type': 'eq' , 'fun': constraint53}
    con54 = { 'type': 'eq' , 'fun': constraint54}
    con55 = { 'type': 'eq' , 'fun': constraint55}
    con56 = { 'type': 'eq' , 'fun': constraint56}
    con61 = { 'type': 'eq' , 'fun': constraint61}
    con62 = { 'type': 'eq' , 'fun': constraint62}
    con63 = { 'type': 'eq' , 'fun': constraint63}
    con64 = { 'type': 'eq' , 'fun': constraint64}
    con65 = { 'type': 'eq' , 'fun': constraint65}
    con66 = { 'type': 'eq' , 'fun': constraint66}
    con71 = { 'type': 'eq' , 'fun': constraint71}
    con72 = { 'type': 'eq' , 'fun': constraint72}
    con73 = { 'type': 'eq' , 'fun': constraint73}
    con74 = { 'type': 'eq' , 'fun': constraint74}
    con75 = { 'type': 'eq' , 'fun': constraint75}
    con76 = { 'type': 'eq' , 'fun': constraint76}
    con81 = { 'type': 'eq' , 'fun': constraint81}
    con82 = { 'type': 'eq' , 'fun': constraint82}
    con83 = { 'type': 'eq' , 'fun': constraint83}
    con84 = { 'type': 'eq' , 'fun': constraint84}
    con85 = { 'type': 'eq' , 'fun': constraint85}
    con86 = { 'type': 'eq' , 'fun': constraint86}
    con91 = { 'type': 'eq' , 'fun': constraint91}
    con92 = { 'type': 'eq' , 'fun': constraint92}
    con93 = { 'type': 'eq' , 'fun': constraint93}
    con94 = { 'type': 'eq' , 'fun': constraint94}
    con95 = { 'type': 'eq' , 'fun': constraint95}
    con96 = { 'type': 'eq' , 'fun': constraint96}


    cons = ([con01,con02,con03,con04,con05,con06 , \
             con11 , con12, con13 , con14, con15, con16 ,\
             con21, con22, con23, con24, con25, con26,\
             con31, con32, con33, con34, con35, con36,\
             con41, con42, con43, con44, con45, con46,\
             con51, con52, con53, con54, con55, con56,\
             con61, con62, con63, con64, con65, con66,\
             con71, con72, con73, con74, con75, con76,\
             con81, con82, con83, con84, con85, con86,\
             con91, con92, con93, con94, con95, con96 ])


#    methods = ['Nelder-Mead' ,'Powell', 'CG', 'BFGS', 'Newton-CG',
#               'L-BFGS-B', 'TNC', 'COBYLA', 'SLSQP', 'trust-constr', 'dogleg',
#               'trust-ncg', 'trust-krylov', 'trust-exact' ]

    #print(len(variables), len(bounds))
    #sol = minimize(objective,state,method=, \

    sol = minimize(objective, x0=variables,method='SLSQP', \
              bounds = bounds , constraints = cons)
    
    #print("ans=" , sol.x)

    

    #return sol.x[0:8]   #XXXXXXXX_XXXXXXXXXXXXX XXX

    return [sol.x[x_start + 1],   sol.x[y_start + 1],
            sol.x[psi_start + 1], sol.x[v_start + 1],
            sol.x[cte_start + 1], sol.x[epsi_start + 1],
            sol.x[delta_start],   sol.x[a_start]]



######################################################################################

if __name__ == "__main__":
    iters = 50

    #print("_____", a_start)
    ptsx = [-100, 100]
    ptsy = [-1, -1]

    coeffs = np.polyfit(ptsx, ptsy, 1)

    x = -1.0
    y = 10.0
    psi = 0.0
    v = 10.0
    cte = np.polyval(coeffs, x) - float(y)
    epsi = psi - float(math.atan(coeffs[1]))



    state = np.zeros(n_variables)
    state[0] = x
    state[1] = y
    state[2] = psi
    state[3] = v
    state[4] = cte
    state[5] = epsi

    x_vals     = [state[0]]
    y_vals     = [state[1]]
    psi_vals   = [state[2]]
    v_vals     = [state[3]]
    cte_vals   = [state[4]]
    epsi_vals  = [state[5]]
    delta_vals = []
    a_vals     = []

    for i in range(iters):
        print("Iteration ",i)

        x, y, psi, v, cte, epsi, delta, a_ = MPCSolve(state, coeffs)

        x_vals.append(x)
        y_vals.append(y)
        psi_vals.append(psi)
        v_vals.append(v)
        cte_vals.append(cte)
        epsi_vals.append(epsi)

        delta_vals.append( delta)
        a_vals.append( a_ )

        #setting for restart
        state = np.zeros(n_variables)
        state[0] = x
        state[1] = y
        state[2] = psi
        state[3] = v
        state[4] = cte
        state[5] = epsi

        print("x = "    , x    ) 
        print("y = "    , y    ) 
        print("psi = "  , psi  ) 
        print("v = "    , v    ) 
        print("cte = "  , cte  ) 
        print("epsi = " , epsi ) 
        print("delta = ", delta) 
        print("a = "    , a_) 

    plt.subplot(3, 1, 1)
    plt.title("CTE")
    plt.plot(cte_vals)
    plt.subplot(3, 1, 2)
    plt.title("Delta (Radians)")
    plt.plot(delta_vals)
    plt.subplot(3, 1, 3)
    plt.title("Velocity")
    plt.plot(v_vals)

    plt.show()
