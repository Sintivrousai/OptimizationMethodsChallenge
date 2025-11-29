from VRP_Model import *
from SolDrawer import *
import random

class Solution:
    def __init__(self):
        self.cost = 0.0
        self.routes = []
        self.sequenceOfNodes = []

class Saving:
    def __init__(self, n1, n2, sav):
        self.n1 = n1
        self.n2 = n2
        self.score = sav
        
class RelocationMove(object):
    def __init__(self):
        self.originRoutePosition = None
        self.targetRoutePosition = None
        self.originNodePosition = None
        self.targetNodePosition = None
        self.costChangeOriginRt = None
        self.costChangeTargetRt = None
        self.moveCost = None

    def Initialize(self):
        self.originRoutePosition = None
        self.targetRoutePosition = None
        self.originNodePosition = None
        self.targetNodePosition = None
        self.costChangeOriginRt = None
        self.costChangeTargetRt = None
        self.moveCost = 10 ** 9


class SwapMove(object):
    def __init__(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.costChangeFirstRt = None
        self.costChangeSecondRt = None
        self.moveCost = None
    def Initialize(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.costChangeFirstRt = None
        self.costChangeSecondRt = None
        self.moveCost = 10 ** 9


class TwoOptMove(object):
    def __init__(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.moveCost = None
    def Initialize(self):
        self.positionOfFirstRoute = None
        self.positionOfSecondRoute = None
        self.positionOfFirstNode = None
        self.positionOfSecondNode = None
        self.moveCost = 10 ** 9

class CustomerInsertion(object):
    def __init__(self):
        self.customer = None
        self.route = None
        self.cost = 10 ** 9

class CustomerInsertionAllPositions(object):
    def __init__(self):
        self.customer = None
        self.route = None
        self.insertionPosition = None
        self.cost = 10 ** 9

class CustomerInsertion(object):
    def __init__(self):
        self.customer = None
        self.route = None
        self.cost = 10 ** 9

class CustomerInsertionAllPositions(object):
    def __init__(self):
        self.customer = None
        self.route = None
        self.insertionPosition = None
        self.cost = 10 ** 9

class Solver:
    def __init__(self, m):
        self.allNodes = m.allNodes
        self.customers = m.customers
        self.depot = m.allNodes[0]
        self.distanceMatrix = m.matrix
        self.capacity = m.capacity
        self.sol = None
        self.bestSolution = None
        self.list1=[]
        self.searchTrajectory = []
        self.minTabuTenure = 50
        self.maxTabuTenure = 60
        self.tabuTenure = 20


    def solve(self):
        
        self.sol = Solution() 
        self.sol.routes=self.find_distances_and_routes()
        self.sol.cost=self.CalculateTotalCost(self.sol)
        self.ReportSolution(self.sol)
        SolDrawer.draw("solution", self.sol, self.allNodes)
        
        #self.ReportSolution(self.sol)
        self.TabuSearch(0)
        self.ReportSolution(self.sol)
        SolDrawer.draw("final", self.sol, self.allNodes)
        
        #self.ReportSolution(self.sol)
        #self.VND()
        #self.ReportSolution(self.sol)
        
        #self.ReportSolution(self.sol)
        #self.TabuSearch(0)
        #self.ReportSolution(self.sol)
        
        return self.sol
    
    def find_distances_and_routes(self):
        rows = len(self.allNodes)
        dist = [[0.0 for x in range(rows)] for y in range(rows)]

        for i in range(rows):
            for j in range(rows):
                distance = math.sqrt(math.pow(self.allNodes[i].x - self.allNodes[j].x, 2) +
                                     math.pow(self.allNodes[i].y - self.allNodes[j].y, 2))
                dist[i][j] = distance

        visited = [False] * len(self.allNodes)
        starting_node = 0
        visited[starting_node] = True
        done = False
        max_weight = self.capacity
        rt=Route(self.depot,self.capacity)
        while not done:
            min_distance = max(dist[starting_node])
            minpos = -1

            for i in range(len(self.allNodes)):
                max_weight =max_weight
                if dist[starting_node][i] <= min_distance and max_weight - self.allNodes[i].demand >= 0 and not visited[i]:
                    min_distance = dist[starting_node][i]
                    minpos = i

            if minpos != -1:
                visited[minpos] = True
                starting_node = minpos
                max_weight = max_weight - self.allNodes[minpos].demand
                rt.sequenceOfNodes.append(self.allNodes[starting_node])
            else:
                starting_node = 0
                max_weight = self.capacity
                self.sol.routes.append(rt)

            if all(visited):
                done = True
                self.sol.routes.append(rt)
            elif starting_node == 0:
                rt=Route(self.depot,self.capacity)
        return self.sol.routes
    
    def TabuSearch(self, operator):
        solution_cost_trajectory = []
        random.seed(1)
        self.bestSolution = self.cloneSolution(self.sol)
        terminationCondition = False
        localSearchIterator = 0

        rm = RelocationMove()
        sm = SwapMove()
        top:TwoOptMove = TwoOptMove()

        SolDrawer.draw(0, self.sol, self.allNodes)

        while terminationCondition is False:
            operator = random.randint(0,2)

            rm.Initialize()
            sm.Initialize()
            top.Initialize()
        
            # Relocations
            if operator == 0:
                self.FindBestRelocationMove(rm, localSearchIterator)
                if rm.originRoutePosition is not None:
                    self.ApplyRelocationMove(rm, localSearchIterator) 
            # Swaps
            
            elif operator == 1:
                self.FindBestSwapMove(sm, localSearchIterator)
                if sm.positionOfFirstRoute is not None:
                    self.ApplySwapMove(sm, localSearchIterator)
            elif operator == 2:
                self.FindBestTwoOptMove(top, localSearchIterator)
                if top.positionOfFirstRoute is not None:
                    self.ApplyTwoOptMove(top, localSearchIterator)

            # self.ReportSolution(self.sol)
            self.TestSolution()
            #solution_cost_trajectory.append(self.sol.cost)

            print(localSearchIterator, self.sol.cost, self.bestSolution.cost)

            if (self.sol.cost < self.bestSolution.cost):
                self.bestSolution = self.cloneSolution(self.sol)

            # SolDrawer.draw(localSearchIterator, self.sol, self.allNodes)

            localSearchIterator = localSearchIterator + 1

            if localSearchIterator > 1000:
                terminationCondition = True

        #SolDrawer.draw('final_ts', self.bestSolution, self.allNodes)
        #SolDrawer.drawTrajectory(solution_cost_trajectory)

        self.sol = self.bestSolution
        
    def MoveIsTabu(self, n: Node, iterator, moveCost):
        if moveCost + self.sol.cost < self.bestSolution.cost - 0.001:
            return False
        if iterator < n.isTabuTillIterator:
            return True
        return False

    def SetTabuIterator(self, n: Node, iterator):
        # n.isTabuTillIterator = iterator + self.tabuTenure
        n.isTabuTillIterator = iterator + random.randint(self.minTabuTenure, self.maxTabuTenure)

    
    def MinimumInsertions(self):
        model_is_feasible = True
        self.sol = Solution()
        insertions = 0

        while insertions < len(self.customers):
            best_insertion = CustomerInsertionAllPositions()
            self.Always_keep_an_empty_route()
            self.IdentifyMinimumCostInsertion(best_insertion)

            if best_insertion.customer is not None:
                self.ApplyCustomerInsertionAllPositions(best_insertion)
                insertions += 1
            else:
                print('FeasibilityIssue')
                model_is_feasible = False
                break
        if model_is_feasible:
            self.TestSolution()

    def LocalSearch(self, operator):
        self.bestSolution = self.cloneSolution(self.sol)
        terminationCondition = False
        localSearchIterator = 0

        rm = RelocationMove()
        sm = SwapMove()
        top = TwoOptMove()
        
        self.FindKNearestNeighborsForNode(5)
        
        while terminationCondition is False:

            self.InitializeOperators(rm, sm, top)
            #SolDrawer.draw(localSearchIterator, self.sol, self.allNodes)

            # Relocations
            if operator == 0:
                self.FindBestRelocationMove(rm)
                if rm.originRoutePosition is not None:
                    if rm.moveCost < 0:
                        self.ApplyRelocationMove(rm)
                    else:
                        terminationCondition = True
            # Swaps
            elif operator == 1:
                self.FindBestRelocationMove(rm)
                if rm.originRoutePosition is not None:
                    if rm.moveCost < 0:
                        self.ApplyRelocationMove(rm)
                    else:
                        terminationCondition = True
                self.FindBestSwapMove(sm)
                if sm.positionOfFirstRoute is not None:
                    if sm.moveCost < 0:
                        self.ApplySwapMove(sm)
                    else:
                        terminationCondition = True
            elif operator == 2:
                self.FindBestTwoOptMove(top)
                if top.positionOfFirstRoute is not None:
                    if top.moveCost < 0:
                        self.ApplyTwoOptMove(top)
                    else:
                        terminationCondition = True

            #self.TestSolution()

            if (self.sol.cost < self.bestSolution.cost):
                self.bestSolution = self.cloneSolution(self.sol)

            localSearchIterator = localSearchIterator + 1
            print(localSearchIterator, self.sol.cost)

        self.sol = self.bestSolution
    
   

    def cloneRoute(self, rt:Route):
        cloned = Route(self.depot, self.capacity)
        cloned.cost = rt.cost
        cloned.load = rt.load
        cloned.sequenceOfNodes = rt.sequenceOfNodes.copy()
        return cloned

    def cloneSolution(self, sol: Solution):
        cloned = Solution()
        for i in range (0, len(sol.routes)):
            rt = sol.routes[i]
            clonedRoute = self.cloneRoute(rt)
            cloned.routes.append(clonedRoute)
        cloned.cost = self.sol.cost
        return cloned
    
    
    def FindRoute(self, targetNode):
        
        for i in range(len(self.sol.routes)):
            route:Route = self.sol.routes[i]
            #print(route.sequenceOfNodes)
            for j in range(len(route.sequenceOfNodes)):
                #print(route.sequenceOfNodes[j].ID)
                if route.sequenceOfNodes[j].ID == targetNode:
                    #print(j)
                    #print(i)
                    return j, i
    
    def FindKNearestNeighborsForNode(self, node):
        
        knearest = []
        distances = [(j, self.distanceMatrix[node][j]) for j in range(len(self.distanceMatrix[node]))]
        distances.sort(key=lambda x : x[1])
        knearest = distances[1:8]
        #print(knearest)  
              
        return knearest
    
    def FindBestRelocationMove(self, rm,iterator):
        for originRouteIndex in range(0, len(self.sol.routes)):
            rt1:Route = self.sol.routes[originRouteIndex]
            for originNodeIndex in range(1, len(rt1.sequenceOfNodes) - 2):
                for targetRouteIndex in range (0, len(self.sol.routes)):
                    rt2:Route = self.sol.routes[targetRouteIndex]
                    for targetNodeIndex in range (1, len(rt2.sequenceOfNodes) - 1):

                        #the nodes cannot be the same
                        #the target node cannot be the previous to the origin node
                        if originRouteIndex == targetRouteIndex and (targetNodeIndex < originNodeIndex+2):
                            continue
                        B = rt1.sequenceOfNodes[originNodeIndex]

                        newroute1=self.cloneRoute(rt1)
                        newroute2=self.cloneRoute(rt2)
                        newcost1=10**9
                        newcost2=10**9

                        # route before change
                        original_cost1 = self.UpdateRouteCostAndLoad(rt1)
                        original_cost2 = self.UpdateRouteCostAndLoad(rt2)
                                     
                        if rt1==rt2:
                            
                            # route after change
                            newroute1.sequenceOfNodes.remove(B)
                            newroute1.sequenceOfNodes.insert(targetNodeIndex-1,B)
                            newcost1 = self.UpdateRouteCostAndLoad(newroute1)
                            moveCost = newcost1-original_cost1
                            if self.MoveIsTabu(B, iterator, moveCost):
                                continue
                            if abs(newroute1.load - self.capacity)>0.000000000000000001:
                                    continue
                            if (moveCost < rm.moveCost):
                                self.StoreBestRelocationMove(originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex-1, moveCost, rm)
                        
                        else:
                            # capacity of target route not violated
                            if rt2.load + B.demand > rt2.capacity:
                                continue
                            newroute1.sequenceOfNodes.remove(B)
                            newroute2.sequenceOfNodes.insert(targetNodeIndex,B)
                            newcost1 = self.UpdateRouteCostAndLoad(newroute1)
                            newcost2 = self.UpdateRouteCostAndLoad(newroute2)
                            moveCost = newcost1 + newcost2 - original_cost1 - original_cost2 
                            #print('Move Cost:',moveCost)
                            if newroute1.load>self.capacity or newroute2.load>self.capacity:
                                continue
                            if self.MoveIsTabu(B, iterator, moveCost):
                                continue
                            if (moveCost < rm.moveCost):
                                self.StoreBestRelocationMove(originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex, moveCost, rm)

    
    
        
    
    def FindBestSwapMove(self, sm,iterator):
        for firstRouteIndex in range(0, len(self.sol.routes)):
            rt1:Route = self.sol.routes[firstRouteIndex]
            for secondRouteIndex in range (firstRouteIndex, len(self.sol.routes)):
                rt2:Route = self.sol.routes[secondRouteIndex]
                for firstNodeIndex in range (1, len(rt1.sequenceOfNodes) - 1):
                    startOfSecondNodeIndex = 1
                    if rt1 == rt2:
                        startOfSecondNodeIndex = firstNodeIndex + 1
                    for secondNodeIndex in range (startOfSecondNodeIndex, len(rt2.sequenceOfNodes) - 1):

                        a1 = rt1.sequenceOfNodes[firstNodeIndex - 1]
                        b1 = rt1.sequenceOfNodes[firstNodeIndex]
                        c1 = rt1.sequenceOfNodes[firstNodeIndex + 1]

                        a2 = rt2.sequenceOfNodes[secondNodeIndex - 1]
                        b2 = rt2.sequenceOfNodes[secondNodeIndex]
                        c2 = rt2.sequenceOfNodes[secondNodeIndex + 1]

                        moveCost = None
                        costChangeFirstRoute = None
                        costChangeSecondRoute = None

                        newroute1=self.cloneRoute(rt1)
                        newroute2=self.cloneRoute(rt2)
                        moveCost=10**9

                        if rt1 == rt2:
                            if firstNodeIndex == secondNodeIndex - 1:
                                # case of consecutive nodes swap
                                #first route before change
                                costRemoved = self.UpdateRouteCostAndLoad(rt2)
                                #first route after change
                                newroute2.sequenceOfNodes[secondNodeIndex - 1]=b2
                                newroute2.sequenceOfNodes[secondNodeIndex]=a2
                                costAdded = self.UpdateRouteCostAndLoad(newroute2)
                                moveCost = costAdded - costRemoved
                            else:

                                costRemoved1 = self.UpdateRouteCostAndLoad(rt1)
                                
                                newroute1.sequenceOfNodes[secondNodeIndex]=b1
                                newroute1.sequenceOfNodes[firstNodeIndex]=b2
                                costAdded1 = self.UpdateRouteCostAndLoad(newroute1)
                                if newroute1.load>self.capacity:
                                    continue
                                moveCost = costAdded1 - costRemoved1
                                
                        else:
                            if rt1.load - b1.demand + b2.demand > self.capacity:
                                continue
                            if rt2.load - b2.demand + b1.demand > self.capacity:
                                continue

                            costRemoved1 = self.UpdateRouteCostAndLoad(rt1)
                            newroute1.sequenceOfNodes[firstNodeIndex]=b2
                            costAdded1 = self.UpdateRouteCostAndLoad(newroute1)
                            
                            costRemoved2 = self.UpdateRouteCostAndLoad(rt2)
                            newroute2.sequenceOfNodes[secondNodeIndex]=b1
                            costAdded2 = self.UpdateRouteCostAndLoad(newroute2)
                            if newroute2.load>self.capacity or newroute1.load>self.capacity:
                                continue

                            costChangeFirstRoute = costAdded1 - costRemoved1
                            costChangeSecondRoute = costAdded2 - costRemoved2

                            moveCost = costAdded1 + costAdded2 - (costRemoved1 + costRemoved2) 
                        if self.MoveIsTabu(b1, iterator, moveCost) or self.MoveIsTabu(b2, iterator, moveCost):
                            continue

                        if moveCost < sm.moveCost:
                            if firstRouteIndex==2:
                                print("Route2:",rt1.load - b1.demand + b2.demand)
                                #print(rt2.load - b2.demand + b1.demand)
                            self.StoreBestSwapMove(firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex,
                                                   moveCost, costChangeFirstRoute, costChangeSecondRoute, sm)

    def ApplyRelocationMove(self, rm: RelocationMove,iterator):
        
        oldCost = self.CalculateTotalCost(self.sol)
        #we take the routes  
        rt1 = self.sol.routes[rm.originRoutePosition] # originRt
        rt2 = self.sol.routes[rm.targetRoutePosition] # targetRt

        B = rt1.sequenceOfNodes[rm.originNodePosition]
        
        if (rm.originRoutePosition == 247):
            print("ApplyRelocationMove")


        if rt1 == rt2:
            del rt1.sequenceOfNodes[rm.originNodePosition]
            if (rm.originNodePosition < rm.targetNodePosition):
                rt2.sequenceOfNodes.insert(rm.targetNodePosition, B)
            else:
                rt2.sequenceOfNodes.insert(rm.targetNodePosition + 1, B)

            rt1.cost += rm.moveCost
            
        else:
            del rt1.sequenceOfNodes[rm.originNodePosition]
            rt2.sequenceOfNodes.insert(rm.targetNodePosition, B)
            rt1.cost = self.UpdateRouteCostAndLoad(rt1)
            rt2.cost = self.UpdateRouteCostAndLoad(rt2)
            
        self.sol.cost += rm.moveCost    
        newCost = self.CalculateTotalCost(self.sol)
        self.SetTabuIterator(B, iterator)
        #debuggingOnly
        if abs((newCost - oldCost) - rm.moveCost) > 0.0001:
            print('Cost Issue with:',newCost,oldCost+rm.moveCost)


    def ApplySwapMove(self, sm, iterator):
       oldCost = self.CalculateTotalCost(self.sol)
       rt1 = self.sol.routes[sm.positionOfFirstRoute]
       rt2 = self.sol.routes[sm.positionOfSecondRoute]
       b1 = rt1.sequenceOfNodes[sm.positionOfFirstNode]
       b2 = rt2.sequenceOfNodes[sm.positionOfSecondNode]
       rt1.sequenceOfNodes[sm.positionOfFirstNode] = b2
       rt2.sequenceOfNodes[sm.positionOfSecondNode] = b1
       
       if (sm.positionOfFirstRoute == 247):
            print("ApplySwapMove")
       

       if (rt1 == rt2):
           rt1.cost += sm.moveCost
       else:
           rt1.cost += sm.costChangeFirstRt
           rt2.cost += sm.costChangeSecondRt
           rt1.load = rt1.load - b1.demand + b2.demand
           rt2.load = rt2.load + b1.demand - b2.demand

       self.sol.cost += sm.moveCost
       self.SetTabuIterator(b1, iterator)
       self.SetTabuIterator(b2, iterator)

       newCost = self.CalculateTotalCost(self.sol)
       # debuggingOnly
       if abs((newCost - oldCost) - sm.moveCost) > 0.0001:
           print('Cost Issue')

    
    def ReportSolution(self, sol):
        for i in range(0, len(sol.routes)):
            rt = sol.routes[i]
            for j in range (0, len(rt.sequenceOfNodes)):
                print(rt.sequenceOfNodes[j].ID, end=' ')
            print(rt.cost, rt.load)
        print (self.sol.cost)

    def GetLastOpenRoute(self):
        if len(self.sol.routes) == 0:
            return None
        else:
            return self.sol.routes[-1]

    
    
    def StoreBestRelocationMove(self, originRouteIndex, targetRouteIndex, originNodeIndex, targetNodeIndex, moveCost, rm:RelocationMove):
        rm.originRoutePosition = originRouteIndex
        rm.originNodePosition = originNodeIndex
        rm.targetRoutePosition = targetRouteIndex
        rm.targetNodePosition = targetNodeIndex
        rm.moveCost = moveCost

    def StoreBestSwapMove(self, firstRouteIndex, secondRouteIndex, firstNodeIndex, secondNodeIndex, moveCost, costChangeFirstRoute, costChangeSecondRoute, sm):
        sm.positionOfFirstRoute = firstRouteIndex
        sm.positionOfSecondRoute = secondRouteIndex
        sm.positionOfFirstNode = firstNodeIndex
        sm.positionOfSecondNode = secondNodeIndex
        sm.costChangeFirstRt = costChangeFirstRoute
        sm.costChangeSecondRt = costChangeSecondRoute
        sm.moveCost = moveCost

    def InitializeOperators(self, rm, sm, top):
        rm.Initialize()
        sm.Initialize()
        top.Initialize()

    
    def FindBestTwoOptMove(self, top, iterator):
        for rtInd1 in range(0, len(self.sol.routes)):
            rt1:Route = self.sol.routes[rtInd1]
            for rtInd2 in range(rtInd1, len(self.sol.routes)):
                rt2:Route = self.sol.routes[rtInd2]
                for nodeInd1 in range(0, len(rt1.sequenceOfNodes) -1):
                    start2 = 0
                    if (rt1 == rt2):
                        start2 = nodeInd1 + 2
                    for nodeInd2 in range(start2, len(rt2.sequenceOfNodes)-1):
                        moveCost = 10 ** 9

                        A = rt1.sequenceOfNodes[nodeInd1]
                        B = rt1.sequenceOfNodes[nodeInd1 + 1]
                        K = rt2.sequenceOfNodes[nodeInd2]
                        L = rt2.sequenceOfNodes[nodeInd2 + 1]
                        
                        costChangeFirstRoute = None
                        costChangeSecondRoute = None

                        
                        moveCost=10**9
                        
                        if rt1 == rt2:
                            newroute1=self.cloneRoute(rt1)
                            reversedSegment = reversed(rt1.sequenceOfNodes[nodeInd1 + 1: nodeInd2 + 1])
                            newroute1.sequenceOfNodes[nodeInd1 + 1 : nodeInd2 + 1] = reversedSegment

                            if nodeInd1 == 0 :
                                continue
                            costAdded = self.UpdateRouteCostAndLoad(newroute1)
                            if newroute1.load>8 :
                                continue
                            costRemoved = self.UpdateRouteCostAndLoad(rt1)
                            moveCost = costAdded - costRemoved
                        else:
                            newroute1=self.cloneRoute(rt1)
                            newroute2=self.cloneRoute(rt2)

                            
                            relocatedSegmentOfRt1 = newroute1.sequenceOfNodes[nodeInd1 + 1 :]

                            relocatedSegmentOfRt2 = newroute2.sequenceOfNodes[nodeInd2 + 1 :]

                            del newroute1.sequenceOfNodes[nodeInd1 + 1 :]
                            del newroute2.sequenceOfNodes[nodeInd2 + 1 :]

                            newroute1.sequenceOfNodes.extend(relocatedSegmentOfRt2)
                            newroute2.sequenceOfNodes.extend(relocatedSegmentOfRt1)

                            if nodeInd1 == 0 and nodeInd2 == 0:
                                continue
                            if nodeInd1 == len(rt1.sequenceOfNodes) - 2 and  nodeInd2 == len(rt2.sequenceOfNodes) - 2:
                                continue
                            if self.CapacityIsViolated(rt1, nodeInd1, rt2, nodeInd2):
                                continue
                            costAdded = self.UpdateRouteCostAndLoad(newroute1) + self.UpdateRouteCostAndLoad(newroute2)
                            costRemoved = self.UpdateRouteCostAndLoad(rt1) + self.UpdateRouteCostAndLoad(rt2)
                            moveCost = costAdded - costRemoved
                        if self.MoveIsTabu(A, iterator, moveCost) or self.MoveIsTabu(K, iterator, moveCost):
                            continue 
                        if moveCost < top.moveCost:
                            self.StoreBestTwoOptMove(rtInd1, rtInd2, nodeInd1, nodeInd2, moveCost, top)
   
    def CapacityIsViolated(self, rt1, nodeInd1, rt2, nodeInd2):

        rt1FirstSegmentLoad = 0
        for i in range(0, nodeInd1 + 1):
            n = rt1.sequenceOfNodes[i]
            rt1FirstSegmentLoad += n.demand
        rt1SecondSegmentLoad = rt1.load - rt1FirstSegmentLoad

        rt2FirstSegmentLoad = 0
        for i in range(0, nodeInd2 + 1):
            n = rt2.sequenceOfNodes[i]
            rt2FirstSegmentLoad += n.demand
        rt2SecondSegmentLoad = rt2.load - rt2FirstSegmentLoad

        if (rt1FirstSegmentLoad + rt2SecondSegmentLoad > rt1.capacity):
            return True
        if (rt2FirstSegmentLoad + rt1SecondSegmentLoad > rt2.capacity):
            return True

        return False

    def StoreBestTwoOptMove(self, rtInd1, rtInd2, nodeInd1, nodeInd2, moveCost, top):
        top.positionOfFirstRoute = rtInd1
        top.positionOfSecondRoute = rtInd2
        top.positionOfFirstNode = nodeInd1
        top.positionOfSecondNode = nodeInd2
        top.moveCost = moveCost

    
    def ApplyTwoOptMove(self, top,iterator):
        
        rt1:Route = self.sol.routes[top.positionOfFirstRoute]
        rt2:Route = self.sol.routes[top.positionOfSecondRoute]
        
        if (top.positionOfFirstNode + 1 == 247):
            print("ApplyTwoOptMove")

        if rt1 == rt2:
            # reverses the nodes in the segment [positionOfFirstNode + 1,  top.positionOfSecondNode]
            reversedSegment = reversed(rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1])
            #lst = list(reversedSegment)
            #lst2 = list(reversedSegment)
            rt1.sequenceOfNodes[top.positionOfFirstNode + 1 : top.positionOfSecondNode + 1] = reversedSegment

            #reversedSegmentList = list(reversed(rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1]))
            #rt1.sequenceOfNodes[top.positionOfFirstNode + 1: top.positionOfSecondNode + 1] = reversedSegmentList

            rt1.cost = self.UpdateRouteCostAndLoad(rt1)

        else:
            #slice with the nodes from position top.positionOfFirstNode + 1 onwards
            relocatedSegmentOfRt1 = rt1.sequenceOfNodes[top.positionOfFirstNode + 1 :]

            #slice with the nodes from position top.positionOfFirstNode + 1 onwards
            relocatedSegmentOfRt2 = rt2.sequenceOfNodes[top.positionOfSecondNode + 1 :]

            del rt1.sequenceOfNodes[top.positionOfFirstNode + 1 :]
            del rt2.sequenceOfNodes[top.positionOfSecondNode + 1 :]

            rt1.sequenceOfNodes.extend(relocatedSegmentOfRt2)
            rt2.sequenceOfNodes.extend(relocatedSegmentOfRt1)

            self.UpdateRouteCostAndLoad(rt1)
            self.UpdateRouteCostAndLoad(rt2)
        #self.SetTabuIterator(rt1.sequenceOfNodes[top.positionOfFirstNode], iterator)
        #self.SetTabuIterator(rt1.sequenceOfNodes[top.positionOfSecondNode], iterator)
        self.sol.cost += top.moveCost
    
    def UpdateRouteCostAndLoad(self, rt: Route):
        tot_dem = sum(n.demand for n in rt.sequenceOfNodes)
        #print(tot_dem)
        empty_vehicle_weight = 6
        tot_load = empty_vehicle_weight + tot_dem
        rt.load = tot_dem
        
        tn_km = 0
        for i in range(len(rt.sequenceOfNodes) - 1):
            from_node = rt.sequenceOfNodes[i]
            to_node = rt.sequenceOfNodes[i+1]
            tn_km += self.distanceMatrix[from_node.ID][to_node.ID] * tot_load
            tot_load -= to_node.demand
        rt.cost = tn_km
        return tn_km 
            
        
        
    def TestSolution(self):
        print("TestSolution")
        totalSolCost = 0
        for i in range (0, len(self.sol.routes)):
            rt: Route = self.sol.routes[i]
            tot_dem = sum(n.demand for n in rt.sequenceOfNodes)
            empty_vehicle_weight = 6
            tot_load = empty_vehicle_weight + tot_dem
            rtCost = 0
            for i in range(len(rt.sequenceOfNodes) - 1):
                from_node = rt.sequenceOfNodes[i]
                to_node = rt.sequenceOfNodes[i+1]
                rtCost += self.distanceMatrix[from_node.ID][to_node.ID] * tot_load
                tot_load -= to_node.demand
            if abs(rtCost - rt.cost) > 0.0001:
                print ('Route Cost problem with:',rtCost,rt.cost)
            if abs(tot_dem - rt.load) > 0.0001:
                print ('Route Load problem with:',tot_dem,rt.load)
            totalSolCost += rt.cost
            if (rt.sequenceOfNodes[i].ID==247):
                print(rt.sequenceOfNodes[i].ID, tot_dem,rt.load)
        if abs(totalSolCost - self.sol.cost) > 0.0001:
            print('Solution Cost problem')


    def SetRoutedFlagToFalseForAllCustomers(self):
        for i in range(0, len(self.customers)):
            self.customers[i].isRouted = False

    
    #sid feasible solution
    def CalculateTotalCost(self, sol):
        tc=0
        for i in range(len(self.sol.routes)):
            rt: Route = self.sol.routes[i]
            tn_km=self.UpdateRouteCostAndLoad(rt)
            tc+=tn_km
        return tc
        
    
    
        
    def Clarke_n_Wright(self):
        #Makes 250 different routes [0,1] [0,2] [0,3] ..
        self.sol = self.create_initial_routes()
        #example: how much distance i save if instead of [0,1] and [0,2] i do [0,1,2] OR [0,2,1]  (i calculate all the nodes with every neibour)
        savings: list = self.calculate_savings()
        #Sorts in fthinousa because i want the merge that has the biggest saving to be first 
        savings.sort(key=lambda s: s.score, reverse=True)
        for i in range(0, len(savings)):
            sav = savings[i]
            n1 = sav.n1   #node 1
            n2 = sav.n2   #node 2
            rt1 = n1.route  #route of node 1
            rt2 = n2.route   #route of node 2
            
            #if one of them is true merge isn't possible
            if n1.route == n2.route:  
                continue
            if self.not_first_or_last(rt1, n1) or self.not_first_or_last(rt2, n2):  #they should both be starting  or ending node to connect properly
                continue
            if rt1.load + rt2.load > self.capacity:  
                continue
                
            self.merge_routes(n1, n2)   #merge is possible so i go to the method to do the appropriate one

            #self.sol.cost -= sav.score
        self.sol.cost= self.CalculateTotalCost(self.sol)    #just the total cost like the old algorith
        #print(self.sol.cost)
        #just for a list to see the routes it will change
        for route in self.sol.routes:
            temp_route=[]
            for node in route.sequenceOfNodes:
                temp_route.append(node.ID)
                
            self.list1.append(temp_route)
            #print(temp_route)
        

    #i find the saving i will have in a merge and i calculate ALL customers with all their neighbours
    def calculate_savings(self):
        savings = []
        for i in range(0, len(self.customers)):
            n1 = self.customers[i]
            for j in range(i + 1, len(self.customers)):
                n2 = self.customers[j]

                score = self.distanceMatrix[n1.ID][self.depot.ID] + self.distanceMatrix[self.depot.ID][n2.ID] 
                score -= self.distanceMatrix[n1.ID][n2.ID]

                sav = Saving(n1, n2, score)
                savings.append(sav)

        return savings

    def create_initial_routes(self):
        s = Solution()
        for i in range(0, len(self.customers)):
            #print(i, self.customers[i].ID)
            n = self.customers[i]
            rt = Route(self.depot, self.capacity)
            n.route = rt
            n.position_in_route = 1
            rt.sequenceOfNodes.insert(1, n)
            rt.load = n.demand
            rt.cost = self.distanceMatrix[self.depot.ID][n.ID]
            s.routes.append(rt)
            s.cost += rt.cost
        return s
        
    #it check if they are NOT starting or ending node. if they aren't merge is NOT possible
    def not_first_or_last(self, rt, n):
        if n.position_in_route != 1 and n.position_in_route != len(rt.sequenceOfNodes) - 1:
            return True
        return False

    def merge_routes(self, n1, n2):
        rt1 = n1.route
        rt2 = n2.route
        #the end of the 2nd route will merge with the start of the 1st route. e.g [0,1,2] [0,3,4] merge 4 and 1==[0,3,4,1,2]
        
        if n1.position_in_route == 1 and n2.position_in_route == len(rt2.sequenceOfNodes) - 1:
            # for i in range(len(rt2.sequenceOfNodes) - 1, 0, -1):
            #     n = rt2.sequenceOfNodes[i]
            #     rt1.sequenceOfNodes.insert(1, n)
            rt1.sequenceOfNodes[1:1] = rt2.sequenceOfNodes[1:]
        
        #th start of route1 will merge with thestart of route2. e.g [0,1,2] [0,3,4] merge 1 and 3==[0,4,3,1,2]
        #you revert the table [0,3,4]--> [4,3,0]
        elif n1.position_in_route == 1 and n2.position_in_route == 1:
            # for i in range(1, len(rt2.sequenceOfNodes) - 1, 1):
            #     n = rt2.sequenceOfNodes[i]
            #     rt1.sequenceOfNodes.insert(1, n)
            rt1.sequenceOfNodes[1:1] = rt2.sequenceOfNodes[len(rt2.sequenceOfNodes) - 1:0:-1]

        #like the first one but reverse
        elif n1.position_in_route == len(rt1.sequenceOfNodes) - 1 and n2.position_in_route == 1:
            # for i in range(1, len(rt2.sequenceOfNodes) - 1, 1):
            #     n = rt2.sequenceOfNodes[i]
            #     rt1.sequenceOfNodes.insert(len(rt1.sequenceOfNodes) - 1, n)
            rt1.append(rt2.sequenceOfNodes[1:len(rt2.sequenceOfNodes) - 1])
        
        #same with the second but both are the last
        elif n1.position_in_route == len(rt1.sequenceOfNodes) - 1 and n2.position_in_route == len(rt2.sequenceOfNodes) - 1:
            # for i in range(len(rt2.sequenceOfNodes) - 2, 0, -1):
            #     n = rt2.sequenceOfNodes[i]
            #     rt1.sequenceOfNodes.insert(len(rt1.sequenceOfNodes) - 1, n)
            rt1.append(rt2.sequenceOfNodes[len(rt2.sequenceOfNodes) - 1:0:-1])
        #upadates that we don;t need to change
        rt1.load += rt2.load
        self.sol.routes.remove(rt2)
        self.update_route_customers(rt1)

    def update_route_customers(self, rt):
        for i in range(1, len(rt.sequenceOfNodes) - 1):
            n = rt.sequenceOfNodes[i]
            n.route = rt
            n.position_in_route = i

    def IdentifyMinimumCostInsertion(self, best_insertion):
        for i in range(0, len(self.customers)):
            candidateCust: Node = self.customers[i]
            if candidateCust.isRouted is False:
                for rt in self.sol.routes:
                    if rt.load + candidateCust.demand <= rt.capacity:
                        for j in range(0, len(rt.sequenceOfNodes) - 1):
                            A = rt.sequenceOfNodes[j]
                            B = rt.sequenceOfNodes[j + 1]
                            costAdded = self.distanceMatrix[A.ID][candidateCust.ID] + \
                                        self.distanceMatrix[candidateCust.ID][
                                            B.ID]
                            costRemoved = self.distanceMatrix[A.ID][B.ID]
                            trialCost = costAdded - costRemoved
                            if trialCost < best_insertion.cost:
                                best_insertion.customer = candidateCust
                                best_insertion.route = rt
                                best_insertion.insertionPosition = j
                                best_insertion.cost = trialCost
                    else:
                        continue


    def print_to_file(self, filename='output.txt'):
        with open(filename, 'w') as output_file:
            output_file.write(f'Cost:\n{self.sol.cost}\n')
            output_file.write(f'Routes:\n{len(self.sol.routes)}\n')

            for route in self.sol.routes:
                output_file.write(f'{",".join(map(str, [node.ID for node in route.sequenceOfNodes]))}\n')
 

    
