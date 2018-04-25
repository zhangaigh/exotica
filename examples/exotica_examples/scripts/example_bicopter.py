#!/usr/bin/env python
import pyexotica as exo
import time

exo.Setup.initRos()

prob = exo.Initializers.loadXML('{exotica_examples}/resources/configs/example_bicopter_visualization.xml')
problem = exo.Setup.createProblem(prob)
scene = problem.getScene()

for i in range(100):
    scene.setModelStateMap({'world_joint/trans_z': .01*i})
    scene.getSolver().publishFrames()
    time.sleep(0.02)
