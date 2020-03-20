# -*- coding: utf-8 -*-
"""
Created on Tue Oct 15 22:31:20 2019

@author: 7175000
"""

import numpy as np
import modern_robotics as mr


def course3wk2final(thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist,totalT, dt,filename):
    
    # initial the position log
    thetamatrix = thetalist
    for t in np.arange(0, totalT, dt):
        print(t)
        # get ddthetalist
        ddthetalist = mr.ForwardDynamics(thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist)
        # find next joint configuration from first-order Euler integration
        thetalistNext, dthetalistNext=mr.EulerStep(thetalist, dthetalist, ddthetalist, dt)
        thetamatrix = np.vstack((thetamatrix , thetalistNext)) 
        thetalist = thetalistNext
        dthetalist = dthetalistNext
    with open(filename, "w") as text_file:
        print("\n".join([",".join(["{:0.8f}".format(item) for item in row]) for row in thetamatrix]), file = text_file)
    return thetamatrix
           
if __name__ == "__main__":
    
    M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
    M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
    M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
    M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
    M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
    M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
    M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
    G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
    G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
    G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
    G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
    G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
    G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
    Glist = [G1, G2, G3, G4, G5, G6]
    Mlist = [M01, M12, M23, M34, M45, M56, M67] 
    Slist = [[0,         0,         0,         0,        0,        0],
             [0,         1,         1,         1,        0,        1],
             [1,         0,         0,         0,       -1,        0],
             [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
             [0,         0,         0,         0,  0.81725,        0],
             [0,         0,     0.425,   0.81725,        0,  0.81725]]
    g = np.array([0, 0, -9.81])
    # initial configuration, zero velocity
    dthetalist = np.array([0, 0, 0, 0, 0, 0])
    taulist = np.array([0, 0, 0, 0, 0, 0])
    Ftip = np.array([0,0,0,0,0,0])
    dt = 0.01
    
    # simulation 1
    thetalist = np.array([0, 0, 0, 0, 0, 0])
    totalT = 3
    filename = "simulation1.csv"
    course3wk2final(thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist,totalT, dt,filename)
    
    # simulation 2
    thetalist = np.array([0, -1, 0, 0, 0, 0])
    totalT = 5
    filename = "simulation2.csv"
    course3wk2final(thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist,totalT, dt,filename)
    