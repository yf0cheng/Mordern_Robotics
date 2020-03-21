import csv
from collections import defaultdict, Counter
import math
import pprint

## define the class for nodes using in A* search
class AstarNode():
    ## public variables, change will impact for all nodes
    num_Nodes = 0
    sortByField = '_ID'
    def __init__(self, ID, OptimisticCTG):
        ## instance variables, change only impacts individaul node
        self._ID = ID
        self._OptimisticCTG   = OptimisticCTG
        self._parentNode = 0
        self._pastCost = math.inf
        self._estTotCost = OptimisticCTG + self._pastCost
        ## keep track of # of instances
        AstarNode.num_Nodes += 1
    
    ## use getter and setter for individual property setting
    @property
    def parentNode(self):
        return self._parentNode
    @parentNode.setter
    def parentNode(self,ID):
         self._parentNode = ID

    @property
    def pastCost(self):
        return self._pastCost 
    @pastCost.setter
    def pastCost(self, pastCost):
         self._pastCost = pastCost
         self._estTotCost = self._OptimisticCTG + pastCost
    
    ## change the sorting field
    @classmethod
    def setSortByField(cls, field):
        if field !='_ID' and field !='_OptimisticCTG' and field != '_pastCost' and field != '_estTotCost':
            raise Exception ('Please use valid fieldname: _ID/_OptimisticCTG/_pastCost/_estTotCost')
        else:
            cls.sortByField = field

    ## determine sort by field
    def __lt__(self, other):
        return getattr(self, AstarNode.sortByField) < getattr(other, AstarNode.sortByField)
    ## define how to print. Defaul would be <__main__.AstarNode object at 0x10d8097b8> with no clear information
    def __repr__(self):
        return f"ID:{self._ID}\t pastCost:{self._pastCost}\t OptCTG:{self._OptimisticCTG}\t EstTotCost: {self._estTotCost}\t parentNode:{self._parentNode}"

## main function to implement A* search
def AstarSearch(nodefilename, edgefilename, pathfilename):
    pp = pprint.PrettyPrinter(indent=1, width=120)
    isSuccess = 0
    pathList = [1]
    ## extract nodes.csv to a Astar class and add into a list 
    nodeList = []
    with open(nodefilename, 'r', newline='\n', encoding="utf8") as csvfile:
        csvreader = csv.reader(csvfile, delimiter=',')
        for curRow in csvreader:
            if not curRow[0].startswith('#'):## ignore comment line
                extractNode = AstarNode(int(curRow[0]),float(curRow[3]))
                nodeList.append(extractNode)
    nodeList = sorted(nodeList)

    if len(nodeList)>100:
        print("input nodes >100. No search.")
    else:
        ## extract weightings from edges.csv to a dictionary and add into a list
        edgeList = []
        with open(edgefilename, 'r', newline='\n', encoding="utf8") as csvfile:
            csvreader = csv.reader(csvfile, delimiter=',')
            for curRow in csvreader:
                if not curRow[0].startswith('#'):## ignore comment line
                    keylist = ['ID1', 'ID2', 'Cost']
                    vallist = [int(curRow[0]),int(curRow[1]),float(curRow[2])]
                    EdgeDict = dict(zip(keylist, vallist))
                    edgeList.append(EdgeDict)

        ## build openList and ClosedList
        openList = []
        nodeList[0].pastCost = 0
        openList.append(nodeList[0])
        closedList = []
        
        while not len(openList)==0:
            ## change sorting scheme to estimated total cost, default was sorting by ID
            AstarNode.setSortByField('_estTotCost')
            curNode = openList[0]
            openList.remove(curNode)
            curID = curNode._ID
            closedList.append(curID)
            ## find neigbor Node
            for edge in edgeList:
                if edge['ID1']==curID or edge['ID2']==curID:
                    edgeNodes = [edge['ID1'], edge['ID2']]
                    edgeNodes.remove(curID)
                    nextID = edgeNodes[0]
                    ## if not in the closedList, calculate tempPastCost, update pastCost
                    if not any(closedNode ==nextID for closedNode in closedList):
                        nextNode = next((node for node in nodeList if node._ID == nextID ), None)
                        tempPastCost = edge['Cost']+curNode.pastCost
                        if tempPastCost < nextNode.pastCost:
                            nextNode.parentNode = curID
                            nextNode.pastCost = tempPastCost
                            if not any(openNode._ID==nextID for openNode in openList):
                                openList.append(nextNode)
                                openList = sorted(openList)
            ## check openList after search all the nbr nodes for curNode
            if openList[0]==nodeList[-1]:
                isSuccess = 1
                break
    if isSuccess:
        print("Success!")
        ## find path by parentNodes
        pathList = [nodeList[-1]._ID, nodeList[-1].parentNode]
        while not pathList[-1]==nodeList[0]._ID:
            preNode = next(node.parentNode for node in nodeList if node._ID == pathList[-1])
            pathList.append(preNode)
        pathList = sorted(pathList)
    else:
        print("failed...")
    ## print out the answer and save to path
    with open(pathfilename,'w') as textfile:
        print((','.join(["{:d}".format(node) for node in pathList])), file=textfile)
    return pathList

if __name__=='__main__':
    nodefilename = 'results/nodes.csv'
    edgefilename = 'results/edges.csv'
    pathfilename = 'results/path.csv'
    print(AstarSearch(nodefilename, edgefilename, pathfilename))
