#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 14 23:19:16 2020

@author: alex
------------------------------------


Fichier d'amorce pour les livrables de la problématique GRO640'


"""

import numpy as np

from pyro.control  import robotcontrollers
from pyro.control.robotcontrollers import EndEffectorPD
from pyro.control.robotcontrollers import EndEffectorKinematicController


###################
# Part 1
###################

import numpy as np

def dh2T(r, d, theta, alpha):
    """
    Parameters
    ----------
    r     : float 1x1
        Link length (distance along x-axis)
    d     : float 1x1
        Link offset (distance along z-axis)
    theta : float 1x1
        Joint angle (rotation around z-axis)
    alpha : float 1x1
        Link twist (rotation around x-axis)
    
    Returns
    -------
    T     : float 4x4 (numpy array)
        Transformation matrix
    """
    
    T = np.zeros((4, 4))
    
    T[0, 0] = np.cos(theta)
    T[0, 1] = -np.sin(theta) * np.cos(alpha)
    T[0, 2] = np.sin(theta) * np.sin(alpha)
    T[0, 3] = r * np.cos(theta)
    
    T[1, 0] = np.sin(theta)
    T[1, 1] = np.cos(theta) * np.cos(alpha)
    T[1, 2] = -np.cos(theta) * np.sin(alpha)
    T[1, 3] = r * np.sin(theta)
    
    T[2, 1] = np.sin(alpha)
    T[2, 2] = np.cos(alpha)
    T[2, 3] = d
    
    T[3, 3] = 1
    
    return T

def dhs2T(r, d, theta, alpha):
    """
    Parameters
    ----------
    r     : float nx1
        Array of link lengths (distances along x-axes)
    d     : float nx1
        Array of link offsets (distances along z-axes)
    theta : float nx1
        Array of joint angles (rotations around z-axes)
    alpha : float nx1
        Array of link twists (rotations around x-axes)
    
    Returns
    -------
    WTT     : float 4x4 (numpy array)
              Global transformation matrix from base frame to tool frame
    """
    
    WTT = np.eye(4)
    
    n = len(r)
    
    for i in range(n):
        Ti = dh2T(r[i], d[i], theta[i], alpha[i])
        WTT = np.dot(WTT, Ti)
    
    return WTT


def f(q):
    """
    

    Parameters
    ----------
    q : float 6x1
        Joint space coordinates

    Returns
    -------
    r : float 3x1 
        Effector (x,y,z) position

    """
    r = np.zeros((3,1))
    
    ###################
    # Votre code ici
    ###################
    
    return r

r = [1.0, 0.5]  # Link lengths
d = [2.0, 1.5]  # Link offsets
theta = [np.pi / 4, np.pi / 6]  # Joint angles (45 degrees, 30 degrees)
alpha = [np.pi / 6, np.pi / 4]  # Link twists (30 degrees, 45 degrees)

# Compute the global transformation matrix
WTT = dhs2T(r, d, theta, alpha)

# Extract the position of the end-effector
position = WTT[:3, 3]

# Extract the rotation matrix
rotation_matrix = WTT[:3, :3]

# Print the results
print("Global Transformation Matrix (WTT):")
print(WTT)
print("\nPosition of the end-effector (x, y, z):")
print(position)
print("\nOrientation (rotation matrix):")
print(rotation_matrix)

###################
# Part 2
###################
    
class CustomPositionController( EndEffectorKinematicController ) :
    
    ############################
    def __init__(self, manipulator ):
        """ """
        
        EndEffectorKinematicController.__init__( self, manipulator, 1)
        
        ###################################################
        # Vos paramètres de loi de commande ici !!
        ###################################################
        
    
    #############################
    def c( self , y , r , t = 0 ):
        """ 
        Feedback law: u = c(y,r,t)
        
        INPUTS
        y = q   : sensor signal vector  = joint angular positions      dof x 1
        r = r_d : reference signal vector  = desired effector position   e x 1
        t       : time                                                   1 x 1
        
        OUPUTS
        u = dq  : control inputs vector =  joint velocities             dof x 1
        
        """
        
        # Feedback from sensors
        q = y
        
        # Jacobian computation
        J = self.J( q )
        
        # Ref
        r_desired   = r
        r_actual    = self.fwd_kin( q )
        
        # Error
        e  = r_desired - r_actual
        
        ################
        dq = np.zeros( self.m )  # place-holder de bonne dimension
        
        ##################################
        # Votre loi de commande ici !!!
        ##################################

        
        return dq
    
    
###################
# Part 3
###################
        

        
class CustomDrillingController( robotcontrollers.RobotController ) :
    """ 

    """
    
    ############################
    def __init__(self, robot_model ):
        """ """
        
        super().__init__( dof = 3 )
        
        self.robot_model = robot_model
        
        # Label
        self.name = 'Custom Drilling Controller'
        
        
    #############################
    def c( self , y , r , t = 0 ):
        """ 
        Feedback static computation u = c(y,r,t)
        
        INPUTS
        y  : sensor signal vector     p x 1
        r  : reference signal vector  k x 1
        t  : time                     1 x 1
        
        OUPUTS
        u  : control inputs vector    m x 1
        
        """
        
        # Ref
        f_e = r
        
        # Feedback from sensors
        x = y
        [ q , dq ] = self.x2q( x )
        
        # Robot model
        r = self.robot_model.forward_kinematic_effector( q ) # End-effector actual position
        J = self.robot_model.J( q )      # Jacobian matrix
        g = self.robot_model.g( q )      # Gravity vector
        H = self.robot_model.H( q )      # Inertia matrix
        C = self.robot_model.C( q , dq ) # Coriolis matrix
            
        ##################################
        # Votre loi de commande ici !!!
        ##################################
        
        u = np.zeros(self.m)  # place-holder de bonne dimension
        
        return u
        
    
###################
# Part 4
###################
        
    
def goal2r( r_0 , r_f , t_f ):
    """
    
    Parameters
    ----------
    r_0 : numpy array float 3 x 1
        effector initial position
    r_f : numpy array float 3 x 1
        effector final position
    t_f : float
        time 

    Returns
    -------
    r   : numpy array float 3 x l
    dr  : numpy array float 3 x l
    ddr : numpy array float 3 x l

    """
    # Time discretization
    l = 1000 # nb of time steps
    
    # Number of DoF for the effector only
    m = 3
    
    r = np.zeros((m,l))
    dr = np.zeros((m,l))
    ddr = np.zeros((m,l))
    
    #################################
    # Votre code ici !!!
    ##################################
    
    
    return r, dr, ddr


def r2q( r, dr, ddr , manipulator ):
    """

    Parameters
    ----------
    r   : numpy array float 3 x l
    dr  : numpy array float 3 x l
    ddr : numpy array float 3 x l
    
    manipulator : pyro object 

    Returns
    -------
    q   : numpy array float 3 x l
    dq  : numpy array float 3 x l
    ddq : numpy array float 3 x l

    """
    # Time discretization
    l = r.shape[1]
    
    # Number of DoF
    n = 3
    
    # Output dimensions
    q = np.zeros((n,l))
    dq = np.zeros((n,l))
    ddq = np.zeros((n,l))
    
    #################################
    # Votre code ici !!!
    ##################################
    
    
    return q, dq, ddq



def q2torque( q, dq, ddq , manipulator ):
    """

    Parameters
    ----------
    q   : numpy array float 3 x l
    dq  : numpy array float 3 x l
    ddq : numpy array float 3 x l
    
    manipulator : pyro object 

    Returns
    -------
    tau   : numpy array float 3 x l

    """
    # Time discretization
    l = q.shape[1]
    
    # Number of DoF
    n = 3
    
    # Output dimensions
    tau = np.zeros((n,l))
    
    #################################
    # Votre code ici !!!
    ##################################
    
    
    return tau