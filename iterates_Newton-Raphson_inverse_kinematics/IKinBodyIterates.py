#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jul  6 14:10:33 2019

@author: Ychen7175000
"""
import math
import numpy as np
from modern_robotics import *

def printlog(i, thetalist, Tsb, Vb, omg_b_norm, v_b_norm, isappend="w"):
    """
    Exmple Input:
        i = 0
        thetalist = np.array([0,np.pi/6])
        Tsb = np.array([[0.866,-0.500,0.000,1.866],
                        [0.500,0.866,0.000,0.500],
                        [0.000,0.000,1.000,0.000],
                        [0.000,0.000,0.000,1.000]])
        Vb = np.array([0.000,0.000,1.571,0.498,1.858,0.000])
        omg_b_norm = 1.5708
        v_b_norm = 1.9238
        
    Output:
    Iteration 0:

    joint vector (rad):
    0.000,0.524
    
    SE(3) end-effector config:
    0.866,-0.500,0.000,1.866
    0.500,0.866,0.000,0.500
    0.000,0.000,1.000,0.000
    0.000,0.000,0.000,1.000
    
    error twist V_b:
    0.000,0.000,1.571,0.498,1.858,0.000
    
    angular error magnitude ||omega_b||: 1.5708
    
    linear error magnitude ||v_b||: 1.9238 
    """
    with open("log.txt", isappend) as text_file:
        print(f"Iteration {i}:\n", file=text_file)
        print("joint vector (rad):", file=text_file)
        # print(*thetalist, sep = ", ", file=text_file)
        # print([ "{:0.3f}".format(x) for x in thetalist ], file=text_file)
        # print([ f"{x:.3f}" for x in thetalist], file=text_file)
        print("\n".join([",".join(["{:0.3f}".format(item) for item in thetalist])]), file = text_file)
        print("\n" "SE(3) end-effector config:", file=text_file)
        print("\n".join([",".join(["{:0.3f}".format(item) for item in row]) for row in Tsb]), file = text_file)
        print("\n" "error twist V_b:", file=text_file)
        print("\n".join([",".join(["{:0.3f}".format(item) for item in Vb])]), file = text_file)
        print(f"\nangular error magnitude ||omega_b||: {omg_b_norm:.4f}\n", file=text_file)
        print(f"linear error magnitude ||v_b||: {v_b_norm:.4f} \n\n ", file=text_file)

def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    """Computes inverse kinematics in the body frame for an open chain robot

    :param Blist: The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param M: The home configuration of the end-effector
    :param T: The desired end-effector configuration Tsd
    :param thetalist0: An initial guess of joint angles that are close to
                       satisfying Tsd
    :param eomg: A small positive tolerance on the end-effector orientation
                 error. The returned joint angles must give an end-effector
                 orientation error less than eomg
    :param ev: A small positive tolerance on the end-effector linear position
               error. The returned joint angles must give an end-effector
               position error less than ev
    :return thetamatrix: Iterations of Joint angles to achieve T within the 
                        specified tolerances,
    :return success: A logical value where TRUE means that the function found
                     a solution and FALSE means that it ran through the set
                     number of maximum iterations without finding a solution
                     within the tolerances eomg and ev.
    Uses an iterative Newton-Raphson root-finding method.
    The maximum number of iterations before the algorithm is terminated has
    been hardcoded in as a variable called maxiterations. It is set to 20 at
    the start of the function, but can be changed if needed.
    
    This function also saves two output files:
        log.txt: records each iteration for the iteration number i, the joint 
                vector \theta^i , the end-effector configuration
                T_{sb}(\theta^i), the error twist Vb, and the angular and 
                linear error magnitudes, ∥ωb∥ and ∥vb∥. 
        iterates.csv: each row of the text file consists of the comma separated 
                    joint values for that iterate.

    Example Input:
        Blist = np.array([[0,0,1,0,2,0],[0,0,1,0,1,0]]).T
        M = np.array([[1,0,0,2],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        T = np.array([[-0.5,-0.866,0,0.366],[0.866,-0.5,0,1.366],[0,0,1,0],[0,0,0,1]])
        thetalist0 = np.array([0,np.pi/6])
        eomg = 0.001
        ev = 10**-4
    Output:
        (np.array([[0.        , 0.52359878], 
                   [0.59749064, 1.38191994], 
                   [0.52324156, 1.5746616 ], 
                   [0.52358947, 1.57082969]]), True)
    """

    thetalist = np.array(thetalist0).copy()
    thetamatrix = thetalist.T
    i = 0
    maxiterations = 20
    Tsb = FKinBody(M, Blist, thetalist)
    Vb = se3ToVec(MatrixLog6(np.dot(TransInv(Tsb), T)))
    omg_b_norm = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
    v_b_norm = np.linalg.norm([Vb[3], Vb[4], Vb[5]])
    err =  omg_b_norm > eomg or v_b_norm > ev
    printlog(i, thetalist, Tsb, Vb, omg_b_norm, v_b_norm)      
    
                
    while err and i < maxiterations:
        thetalist = thetalist \
                    + np.dot(np.linalg.pinv(JacobianBody(Blist, \
                                                         thetalist)), Vb)
        thetamatrix = np.vstack((thetamatrix , thetalist.T))           
        i = i + 1
        Tsb = FKinBody(M, Blist, thetalist)
        Vb = se3ToVec(MatrixLog6(np.dot(TransInv(Tsb), T)))
        omg_b_norm = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
        v_b_norm = np.linalg.norm([Vb[3], Vb[4], Vb[5]])
        err =  omg_b_norm > eomg or v_b_norm > ev
        # print details to log.txt
        printlog(i, thetalist, Tsb, Vb, omg_b_norm, v_b_norm, isappend = "a")   
    
    # save joint angles to iterates.csv    
    with open("iterates.csv", "w") as text_file:
        print("\n".join([",".join(["{:0.8f}".format(item) for item in row]) for row in thetamatrix]), file = text_file)
        
    return (thetamatrix, not err)

if __name__ == "__main__":
    Blist = np.array([[0,0,1,0,2,0],[0,0,1,0,1,0]]).T
    M = np.array([[1,0,0,2],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    T = np.array([[-0.5,-0.866,0,0.366],[0.866,-0.5,0,1.366],[0,0,1,0],[0,0,0,1]])
    thetalist0 = np.array([0,np.pi/6])
    eomg = 0.001
    ev = 10**-4
    thetamatrix, isconverge = IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev)
    
#    
#    W1 = 0.109
#    W2 = 0.082
#    L1 = 0.425
#    L2 = 0.392
#    H1 = 0.089
#    H2 = 0.095
#    
#    M = np.array([[-1,0,0,L1+L2],[0,0,1,W1+W2],[0,1,0,H1-H2],[0,0,0,1]])    
#    Blist = np.array([[0,1,0,W1+W2,0,L1+L2],[0,0,1,H2,-L1-L2,0],[0,0,1,H2,-L2,0],\
#                      [0,0,1,H2,0,0],[0,-1,0,-W2,0,0],[0,0,1,0,0,0]]).T
#    Tsd = np.array([[0,1,0,-0.5],[0,0,-1,0.1],[-1,0,0,0.1],[0,0,0,1]])
#    thetalist0 = np.array([np.pi*5/6,np.pi/6,-np.pi/2,np.pi/3,-np.pi/6,-np.pi/2])
#    eomg = 0.001
#    ev = 0.0001
#    thetamatrix, isconverge = IKinBodyIterates(Blist, M, Tsd, thetalist0, eomg, ev)
    #thetamatrix = np.rad2deg(thetamatrix)
        
