import numpy as np
import util.SAAPlanner as planner

plan = planner.Planner(np.array([26.084206, 76.462398]), np.array([26.084333, 76.461182]))  # 128 metres across approx.

plan.Pathplanning(np.array([[0,3],[0,10],[-1,7.5]]))

plan.Pathplanning(np.array([[-1,5],[0,5],[1,5]]))  # This one failing most of the time

plan.Pathplanning(np.array([[-1,5],[-1.2,5.2],[-1.5,5.5]]))