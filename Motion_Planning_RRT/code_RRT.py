import code_AstarSearch as myAstar
import csv
import numpy as np 
import math, random


def ED(p1,p2):## calculate Euclidean distance \sqrt{(x_1-x_2)^2 + (y_1-y_2)^2}
    V = p2-p1
    dist = math.sqrt( V.dot(V) )
    return dist

def findNearestNode(PointArray,x_sample):
    tempT = np.copy(PointArray) ## create a copy array, so it won't change the original array numbers
    ## calculate the difference between points in PointArray and x_sample
    tempT[:,0] -= x_sample[0]
    tempT[:,1] -= x_sample[1]
    allDist = tempT[:,0]**2+ tempT[:,1]**2
    ## find the index for shortest distance point
    nodeIdx = np.where(allDist==min(allDist))[0][0]
    ## return the index and the distance 
    return nodeIdx, min(allDist)

def checkColision(P2,P1,obstacleArray):
    isColisionFree = True
    ## find the closest obstacles in obstacleArray
    Q1idx , _ = findNearestNode(obstacleArray[:,0:2],P1)
    Q2idx , _ = findNearestNode(obstacleArray[:,0:2],P2)
    Qlist = list(np.unique(np.append(Q1idx,Q2idx)))
    for iObstacle in Qlist:
        ## check intersections of a line segment to a circle
        ## ref: https://codereview.stackexchange.com/questions/86421/line-segment-to-circle-collision-algorithm
        Q = obstacleArray[iObstacle,0:2]  # Centre of circle             
        r = obstacleArray[iObstacle,2]    # Radius of circle
        # P1 = constraint.point1      # Start of line segment
        V = P2 - P1  # Vector along line segment
        a = V.dot(V)
        b = 2 * V.dot(P1 - Q)
        c = P1.dot(P1) + Q.dot(Q) - 2 * P1.dot(Q) - r**2
        disc = b**2 - 4 * a * c
        ## if disc<0, no intersection between the line and the circle
        if disc>0: ## the line has 2 intersection with the circle
            sqrt_disc = math.sqrt(disc)
            t1 = (-b +sqrt_disc)/(2*a)
            t2 = (-b -sqrt_disc)/(2*a)
            if (0<=t1<=1 or 0<=t2<=1): ## check whether intersection is in the segment between P1 and P2
                isColisionFree = False
                break
        elif disc == 0: ## the line has 1 intersection with the circle
            t = -b/(2*a)
            if (0<=t<=1): ## check whether intersection is in the segment
                isColisionFree = False
                break
    return isColisionFree



def RRT(x_str,x_goal,obstacleArray):
    edgeList = []
    ## initial search tree with x_str
    T= np.array([[1,x_str[0], x_str[1],ED(x_str,x_goal)]])
    i_node = 1 ## keep track of how many nodes
    Size_maxTree = 1000 ## limit the search tree size
    stepSize = 0.01 ## limit the max step size from 1 node to another
    isSuccess = False
    X_goal_min = 0.35 ## define a area for X_goal
    i_Xgoal = 0 ## keep track how many times x_sample is in X_goal
    while (i_node < Size_maxTree and not(isSuccess)):
        print(f"nodes len in T: {i_node}")
        ## x_sample
        if (i_node%10==0 and i_Xgoal<30): ## if number of nodes devided by 10 ==0
            i_Xgoal +=1
            ## make 10% of x_sample in X_goal 
            x_sample = np.array([random.randint(X_goal_min*1000,500)/1000,random.randint(X_goal_min*1000,500)/1000])
            print(f"x_sample in Xgoal: {x_sample}")
        else:
            i_Xoal = 0
            x_sample = np.array([random.randint(-500,500)/1000,random.randint(-500,500)/1000])
            print(f"x_sample random: {x_sample}")
    
        ## x_nearest
        nodeIdx, nodeDist = findNearestNode(T[:,1:3],x_sample)
        x_nearest = T[nodeIdx,1:-1]
        ## local planner : straight line from x_nearest to x_new in the direction of x_sample
        if nodeDist<=stepSize:
            x_new = x_sample
            stepCost = nodeDist
        else:
            x_new = x_nearest+(x_sample-x_nearest)/nodeDist*stepSize
            stepCost = stepSize
        ## check colision by the sub-function checkColision
        isColisionFree = checkColision(x_new,x_nearest,obstacleArray)
        if isColisionFree:
            print(f"ColisionFrss x_new: {x_new}")
            i_node +=1
            ## add x_new to T: index, x_x, x_y, distance to x_goal     
            T = np.append(T, [np.append([i_node],np.append(x_new,ED(x_new,x_goal)))] ,axis=0)
            ## add edge from x_nearest to x_new: parent node, new node, stepSize
            edgeList.append([nodeIdx+1,i_node,stepCost])
            print(f"adding edgeList:{edgeList[-1]}")
            ## if x_new is in X_goal:
            if (X_goal_min<=x_new[0]<=0.5 and X_goal_min<=x_new[1]<=0.5):
                print(f"T tree in X_goal")
                i_node +=1
                ## add x_gaol to T
                T = np.append(T, [np.append([i_node],np.append(x_goal,0))] ,axis=0)
                ## add edge from x_new to x_goal
                edgeList.append([i_node-1,i_node,ED(x_new,x_goal)])
                print(f"adding edge:{edgeList[-1]}")
                isSuccess = True

    return isSuccess,T,edgeList


def readCsvToArray(filename):
    tempList = []
    with open(filename,'r',newline='\n',encoding='utf8') as csvfile: ## open the file
        csvreader = csv.reader(csvfile,delimiter = ',') ## use csv reader to read the file
        for curRow in csvreader: ## interpret each line
            if not curRow[0].startswith('#'): ## if it's not a comment
                tempList.append(list(curRow)) ## add the data into tempList
    finalArray = np.array([[float(i) for i in sublist] for sublist in tempList]) ## change list to array
    return finalArray


def printResult(T,edgeList,outputFolder):
    ## define output filenames
    nodefile = outputFolder + "nodes.csv" 
    egdefile = outputFolder + "edges.csv"
    pathfile = outputFolder + "path.csv"
    
    edge = np.array(edgeList)
    childnodes = edge[:,1] ## find the child nodes
    parentnodes = edge[:,0] ## find the parent nodes
    pathList = list([T[-1,0]]) ## start with x_goal, the last node in Tree
    while not pathList[-1]==1: ## if we haven't find the parent node = x_str, keep search
        nodeIdx = np.where(childnodes==pathList[-1])[0][0] ## find child node idx
        preNode = parentnodes[nodeIdx] ## find the corresponding parent node
        pathList.append(preNode) ## append parent node to path
    pathList = pathList[::-1] ## reverse the list for it starts with x_str, node 1

    ## write path.csv 
    with open(pathfile,'w', newline='\n') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(pathList)
    
    ## write nodes.csv
    Tlist = T.tolist() ## convert from array to list 
    with open(nodefile,'w',newline='\n') as csvfile:
        csvwriter = csv.writer(csvfile,delimiter = ',')
        csvwriter.writerows(Tlist) ## write the sublists for each row
    
    ## write edge.csv
    with open(egdefile,'w', newline='\n') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerows(edgeList)

if __name__ == "__main__":
    ObstacleFile = 'planning_coursera/obstacles.csv'
    obstacleArray = readCsvToArray(ObstacleFile) ## read csv and save into an array
    obstacleArray[:,2] /= 2  ## covert the diameter to radius, R
    obstacleArray[:,2] +=0.01  ## add robot's radius, r, and assume robot radius r = 0.01
    x_str = np.array([-0.5,-0.5]) ## set start point
    x_goal = np.array([0.5,0.5]) ## set goal point
    outputFolder = 'results/' ##specifile output folder
    isSuccess,T,edgeList = RRT(x_str,x_goal,obstacleArray) ## calculate RRT
    print(f"search finish with success: {isSuccess}")
    if isSuccess:
        printResult(T,edgeList,outputFolder)
