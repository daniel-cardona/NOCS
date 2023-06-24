#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar  1 11:01:07 2023

@author: sebastian
"""
from casadi import *
import casadi_kin_dyn.py3casadi_kin_dyn as cas_kin_dyn

import numpy as np

from urdf_parser_py.urdf import URDF

import matplotlib.pyplot as plt

import os
path = absPath = os.path.dirname(os.path.abspath(__file__)) + '/../../../build/example/localCollocation/pinocchio_examples/snake_robot/urdf/' 


files = sorted(os.listdir(path))
files= np.delete(files,0)

filename=path+'0_dataFile.txt'
delimiter='\n'

data=np.loadtxt(filename,delimiter=delimiter)

# Open the text file for reading
with open(filename, 'r') as f:
    # Read the lines of the file and remove newline characters
    lines = [line.rstrip('\n') for line in f]
    
    # Convert the lines to an array of integers
    nDof = np.array([int(line) for line in lines])

print(nDof)

counter=0

N=60; #Discrete points

for i in files:

    path_to_urdf=path + i
    
    print(path_to_urdf)
    
    n_joints=nDof[counter]
    
    print(n_joints)
    
    counter=counter+1
    
    urdf_file=open(path_to_urdf,"r")
    
    data=urdf_file.read()
    
    urdf_file.close()
    
    kindyn=cas_kin_dyn.CasadiKinDyn(data)
    
    qddot=kindyn.aba()
 
    
     #States and control vectors
    
    
    q= SX.sym("q",n_joints);
    qd= SX.sym("qd",n_joints);
    u= SX.sym("u",n_joints);
    
    gravity = [0, 0, -9.81]
    
    
     #OCP DATA
    
    nx=n_joints*2; #Number of states
    nu=n_joints; #Number of controls
    
    solMatrix=np.resize(nx+nu,2*N-1)
    
    t0=0;
    tF=10;
    
    
    t_i=SX.sym("t_i",1);
    t_f=SX.sym("t_f",1);
    
    
     #Initial and final states
    
    q0=np.zeros(nx);
    qf=np.random.rand(nx);
    
    
     #States and control bounds
    
    q_max=kindyn.q_max()
    q_min=kindyn.q_min()
    
    
    q_min_lim=np.zeros(n_joints);
    qd_min_lim=np.zeros(n_joints);
    
    q_max_lim=np.zeros(n_joints);
    qd_max_lim=np.zeros(n_joints);
    
    x_min=np.zeros(n_joints*2);
    x_max=np.zeros(n_joints*2);
    
    u_min=np.zeros(n_joints);
    u_max=np.zeros(n_joints);
    
    for i in range(n_joints):
        
        q_min_lim[i]=q_min[i];
        q_max_lim[i]=q_max[i];
        
        qd_min_lim[i]=-10;
        qd_max_lim[i]=10;
        
        u_min[i]=-150;
        u_max[i]=150;
    
    x_min=np.concatenate((q_min_lim,qd_min_lim ));
    x_max=np.concatenate((q_max_lim,qd_max_lim));
    
    #Initial guess of the problem
    
    x_0=np.zeros(nx);
    u_0=np.zeros(n_joints);
    
        
    #Derivative vector
    
    xdot=qddot;
    
     #Cost function
    
    cost=u.T@u
    
     #ca . Function ( ’ Nombre ’ , [ Argumentos sx de entrada ] ,
     #[ Funciones a ser evaluadas con los argumentos de entrada] )
    
    f=Function('f',[q,qd,u],[cost]);
    
    
     #Transcription of the problem
    
     #nDecVar=N*(nx+nu)+2; //TRapezoidal
    
    nDecVar=(2*N-1)*(nx+nu)+2; #H-S
    
    
    
     #DEcision vector
    
    z=MX.sym("z",nDecVar);
    
    z_lb=np.zeros(nDecVar);
    z_ub=np.zeros(nDecVar);
    z_0=np.zeros(nDecVar);
    
     #....
    
    X=np.resize(np.array([],dtype=MX),2*N-1);
    U=np.resize(np.array([],dtype=MX),2*N-1);
    
    offset=2;
    
    z_lb[0]=0;
    z_ub[0]=0;
    
    z_lb[1]=10;
    z_ub[1]=10;
    
    
    z_0[0]=0;
    z_0[1]=5;
    
    for k in range(2*N-1):
        
        X[k]=z[offset:offset+nx]
        
        #Initial solution
        
        z_0[offset:offset+nx]=x_0;
        
        # if k==0:
        #     z_lb[offset:offset+nx]=xi_min;
        #     z_ub[offset:offset+nx]=xi_max;
        # else:
        z_lb[offset:offset+nx]=x_min;
        z_ub[offset:offset+nx]=x_max;
            
        offset+=nx;
        
        #Controls
        
        U[k]=z[offset:offset+nu];
        
        z_0[offset:offset+nu]=u_0;
        
        #Transcribe the bounds
        
        z_lb[offset:offset+nu]=u_min
        z_ub[offset:offset+nu]=u_max
        
        offset+=nu
    
     #Constraints vectors
    
    g=[];
    glb=[];
    gub=[];
    
     #
    
    J=0;
    
    car_idx=0;
        
    for k in range(N-1):
        
        #k   -> k point
        #k+1 -> midpoint
        #k+2 -> k+1 point
        
        k_idx=k*2;
        
        mid_idx=k_idx+1;
        cardinal_idx=k_idx+2;
        
        #Cost and dynamics at k and k+1
        
        #Cost at k
        costk=f(X[k_idx][:n_joints],X[k_idx][n_joints:nx],U[k_idx]);
        qdd=xdot(X[k_idx][:n_joints],X[k_idx][n_joints:nx],U[k_idx]);
        fk=vertcat(X[k_idx][n_joints:nx],qdd)
        
        #Cost at k+1/2
        costk1_2=f(X[mid_idx][:n_joints],X[mid_idx][n_joints:nx],U[mid_idx]);
        qdd1_2=xdot(X[mid_idx][:n_joints],X[mid_idx][n_joints:nx],U[mid_idx]);
        fk1_2=vertcat(X[mid_idx][n_joints:nx],qdd1_2)
        
        #Cost at k+1
        costk1=f(X[cardinal_idx][:n_joints],X[cardinal_idx][n_joints:nx],U[cardinal_idx]);
        qdd1=xdot(X[cardinal_idx][:n_joints],X[cardinal_idx][n_joints:nx],U[cardinal_idx]);
        fk1=vertcat(X[cardinal_idx][n_joints:nx],qdd1)
        
        h=(z[1]-z[0])/N; #Compute the value of the final and initial times
        
        #Construct the collocation constraint
        
        g.append(X[cardinal_idx]-X[k_idx]-(h/6)*(fk+(4*fk1_2)+fk1)); #Simpson quadrature
        
        #Lower and upper bounds 
        
        glb.append(np.zeros(nx))
        gub.append(np.zeros(nx))
            
        g.append(X[mid_idx]-0.5*(X[k_idx]+X[cardinal_idx])-(h/8)*(fk-fk1)); #Hermite Interpolant
        
        #Lower and upper bounds 
        
        glb.append(np.zeros(nx))
        gub.append(np.zeros(nx))
    
        
        #Se obtiene el valor de la funcion de costo
        
        J+=h*(costk1+ (4*costk1_2)+costk);
        
     #Event constraints!
    
    g.append(X[0]-q0)
    glb.append(np.zeros(nx))
    gub.append(np.zeros(nx))
    
    
     #
    g.append(X[2*N-2]-qf);
    glb.append(np.zeros(nx))
    gub.append(np.zeros(nx))
    
    #Compute final cost
    
    J=J*(1/6);
    
    g=vertcat(*g)    
    
    #Cosntruct NLP solver
    
    nlp={'x':z,'f':J,'g':g}
    
    opts ={}
    opts["expand"]=True
    opts["verbose"]=False
    
    #Lets use IPOPT
    
    solver=nlpsol("solver","ipopt",nlp,opts)
    arg={}
    
    #NLP initial guess
    arg["x0"]=z_0
    
    #NLP bounds
    
    arg["lbx"]=z_lb
    arg["ubx"]=z_ub
    
    #NLP constraints bounds
    
    arg["lbg"]=np.concatenate(glb)
    arg["ubg"]=np.concatenate(gub)
    
    res=solver(**arg)
    
    z_opt=np.array(res["x"])
        
    
    res_x_u=z_opt[-(nDecVar-2):]

