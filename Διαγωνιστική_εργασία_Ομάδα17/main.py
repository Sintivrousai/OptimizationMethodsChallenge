#from TSP_Model import Model
from Solver import *
from VRP_Model import *

m = Model()
m.BuildModel()
s = Solver(m)
sol = s.solve()
print(sol.cost)

s.print_to_file('my_solution.txt')