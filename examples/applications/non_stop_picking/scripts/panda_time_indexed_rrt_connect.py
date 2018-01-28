#!/usr/bin/env python

import pyexotica as exo
from numpy import array
from numpy import matrix
from pyexotica.publish_trajectory import *

exo.Setup.initRos()
(sol, prob)=exo.Initializers.loadXMLFull('{non_stop_picking}/resources/panda/panda_time_indexed_rrt_connect.xml')
problem = exo.Setup.createProblem(prob)
solver = exo.Setup.createSolver(sol)
solver.specifyProblem(problem)

solution = solver.solve()

Ts = solution[:,0]
solution = solution[:,1:]
#print(Ts)
#print(solution)
publishTimeIndexedTrajectory(solution, Ts, problem)
