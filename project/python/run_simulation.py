# import libraries
import numpy as np
import pybullet as p
import itertools
from time import sleep

# the main class to do the simulation
from swarm_simulation import World


# the main control loop

#initialize the simulation
world = World()

t = 0.
counter = 0

# starts a simulation
while True:
    world.stepSimulation()
