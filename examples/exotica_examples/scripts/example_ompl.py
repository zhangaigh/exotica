#!/usr/bin/env python

import pyexotica as exo
from numpy import array
from numpy import matrix
from pyexotica.publish_trajectory import *

exo.Setup.initRos()
solver = exo.Setup.loadSolver('{exotica_examples}/resources/configs/ompl_solver_demo.xml')

solution = solver.solve()

t=exo.Timer()
for i in range(1000):
    solver.getProblem().update(solution[i%len(solution)])
dt=t.getDuration()
print(dt)

#plot(solution)

#publishTrajectory(solution, 3.0, problem)
