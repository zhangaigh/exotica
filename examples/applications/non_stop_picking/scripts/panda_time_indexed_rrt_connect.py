#!/usr/bin/env python

import pyexotica as exo
from numpy import array
from numpy import matrix
import numpy
from pyexotica.publish_trajectory import *
import time

exo.Setup.initRos()
(sol, prob)=exo.Initializers.loadXMLFull('{non_stop_picking}/resources/panda/random_1/time_indexed_rrt_connect.xml')
problem = exo.Setup.createProblem(prob)
solver = exo.Setup.createSolver(sol)
solver.specifyProblem(problem)

plan_time = numpy.ones(100)

for i in range(0,100):
	start_time = time.time()
	solution = solver.solve()
	plan_time[i]=time.time() - start_time
	print "T ",i," ", plan_time[i], "sec"

print "Mean ", 1000*numpy.mean(plan_time), " Std ", 1000*numpy.std(plan_time)
#numpy.savetxt("/home/yiming/Documents/IROS_NONSTOP/random_1.txt",plan_time)

Ts = solution[:,0]
solution = solution[:,1:]
#print(Ts)
#print(solution)
problem.getScene().setModelStateMap({'panda_finger_joint1': 0.04, 'panda_finger_joint2': 0.04})
publishTimeIndexedTrajectory(solution, Ts, problem)
