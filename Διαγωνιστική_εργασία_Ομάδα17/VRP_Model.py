
import math
from decimal import Decimal, getcontext

getcontext().prec = 50

class Model:

# instance variables
    def __init__(self):
        self.allNodes = []
        self.customers = []
        self.matrix = []
        self.capacity = -1

    def BuildModel(self):
        
        depot = Node(0,20,20,0)  #it will be read from the txt 
        self.allNodes.append(depot)

        self.capacity = 8.00
        totalCustomers = 251

        with open('Instance.txt', 'r') as file:
            lines = file.readlines()

        # Initialize empty lists to store parameters
        ids = []
        x = []
        y = []
        demand = []

        # Iterate through lines and extract parameters
        for line in lines[5:]:  # Skip the first 4 lines (headers)
            parts = line.strip().split(',')
            ids.append(int(parts[0]))
            x.append(int(parts[1]))
            y.append(int(parts[2]))
            demand.append(float(parts[3]))
        
        

        for i in range (1, totalCustomers):
            cust = Node(ids[i], x[i], y[i], demand[i])
            self.allNodes.append(cust)
            self.customers.append(cust)
        
        rows = len(self.allNodes)
        self.matrix = [[0.0 for x in range(rows)] for y in range(rows)]

        for i in range(0, len(self.allNodes)):
            for j in range(0, len(self.allNodes)):
                a = self.allNodes[i]
                b = self.allNodes[j]
                dist = math.sqrt(math.pow(a.x - b.x, 2) + math.pow(a.y - b.y, 2))
                self.matrix[i][j] = dist
        

class Node:
    def __init__(self, idd, xx, yy, dem):
        self.x = xx
        self.y = yy
        self.ID = idd
        self.demand = dem
        self.isRouted = False
        self.isTabuTillIterator = -1

class Route:
    def __init__(self, dp, cap):
        self.sequenceOfNodes = []
        self.sequenceOfNodes.append(dp)
        self.cost = 0
        self.capacity = cap
        self.load = 0