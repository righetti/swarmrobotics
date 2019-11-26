import numpy as np
import pybullet as p
import itertools

from robot import Robot
    
class World():
    def __init__(self):
        # create the physics simulator
        self.physicsClient = p.connect(p.GUI)
        p.setGravity(0,0,-9.81)
        
        self.max_communication_distance = 2.0

        # We will integrate every 4ms (250Hz update)
        self.dt = 1./250.
        p.setPhysicsEngineParameter(self.dt, numSubSteps=1)

        # Create the plane.
        self.planeId = p.loadURDF("../models/plane.urdf")
        p.changeDynamics(self.planeId, -1, lateralFriction=5., rollingFriction=0)

        self.goalId = p.loadURDF("../models/goal.urdf")
        self.goalId = p.loadURDF("../models/goal2.urdf")
        
        # the balls
        self.ball1 = p.loadURDF("../models/ball1.urdf")
        p.resetBasePositionAndOrientation(self.ball1, [2., 4., 0.5], (0., 0., 0.5, 0.5))
        self.ball2 = p.loadURDF("../models/ball2.urdf")
        p.resetBasePositionAndOrientation(self.ball2, [4., 2., 0.5], (0., 0., 0.5, 0.5))

        p.resetDebugVisualizerCamera(7.0,90.0, -43.0, (1., 1., 0.0))
        
        # Add objects
        wallId = p.loadSDF("../models/walls.sdf")[0]
        p.resetBasePositionAndOrientation(wallId, [0., -1., 0], (0., 0., 0.5, 0.5))
        wallId = p.loadSDF("../models/walls.sdf")[0]
        p.resetBasePositionAndOrientation(wallId, [0., 1., 0], (0., 0., 0.5, 0.5))
        wallId = p.loadSDF("../models/walls.sdf")[0]
        p.resetBasePositionAndOrientation(wallId, [3., -1., 0], (0., 0., 0.5, 0.5))
        wallId = p.loadSDF("../models/walls.sdf")[0]
        p.resetBasePositionAndOrientation(wallId, [3., 1., 0], (0., 0., 0.5, 0.5))
        wallId = p.loadSDF("../models/walls.sdf")[0]
        p.resetBasePositionAndOrientation(wallId, [1., 2., 0], (0., 0., 0., 1.))
        wallId = p.loadSDF("../models/walls.sdf")[0]
        p.resetBasePositionAndOrientation(wallId, [2., -2., 0], (0., 0., 0., 1.))

        # tube
        # wallId = p.loadSDF("../models/walls.sdf")[0]
        # p.resetBasePositionAndOrientation(wallId, [-1., 5., 0], (0., 0., 0., 1.))
        # wallId = p.loadSDF("../models/walls.sdf")[0]
        # p.resetBasePositionAndOrientation(wallId, [-1., 6., 0], (0., 0., 0., 1.))
        
        # #arena
        # wallId = p.loadSDF("../models/walls.sdf")[0]
        # p.resetBasePositionAndOrientation(wallId, [-2, 4., 0], (0., 0., 0.5, 0.5))
        # wallId = p.loadSDF("../models/walls.sdf")[0]
        # p.resetBasePositionAndOrientation(wallId, [-2., 7., 0], (0., 0., 0.5, 0.5))
        # wallId = p.loadSDF("../models/walls.sdf")[0]
        # p.resetBasePositionAndOrientation(wallId, [-2., 9., 0], (0., 0., 0.5, 0.5))
        # wallId = p.loadSDF("../models/walls.sdf")[0]
        # p.resetBasePositionAndOrientation(wallId, [-2., 11., 0], (0., 0., 0.5, 0.5))
        # wallId = p.loadSDF("../models/walls.sdf")[0]
        # p.resetBasePositionAndOrientation(wallId, [-2., 13., 0], (0., 0., 0.5, 0.5))
        # wallId = p.loadSDF("../models/walls.sdf")[0]
        # p.resetBasePositionAndOrientation(wallId, [-3., 3., 0], (0., 0., 0., 1.))
        # wallId = p.loadSDF("../models/walls.sdf")[0]
        # p.resetBasePositionAndOrientation(wallId, [-5., 3., 0], (0., 0., 0., 1.))
        # wallId = p.loadSDF("../models/walls.sdf")[0]
        # p.resetBasePositionAndOrientation(wallId, [-7., 3., 0], (0., 0., 0., 1.))
        # wallId = p.loadSDF("../models/walls.sdf")[0]
        # p.resetBasePositionAndOrientation(wallId, [-8, 4., 0], (0., 0., 0.5, 0.5))
        # wallId = p.loadSDF("../models/walls.sdf")[0]
        # p.resetBasePositionAndOrientation(wallId, [-8., 6., 0], (0., 0., 0.5, 0.5))
        # wallId = p.loadSDF("../models/walls.sdf")[0]
        # p.resetBasePositionAndOrientation(wallId, [-8., 8., 0], (0., 0., 0.5, 0.5))
        # wallId = p.loadSDF("../models/walls.sdf")[0]
        # p.resetBasePositionAndOrientation(wallId, [-8., 10., 0], (0., 0., 0.5, 0.5))
        # wallId = p.loadSDF("../models/walls.sdf")[0]
        # p.resetBasePositionAndOrientation(wallId, [-8., 12., 0], (0., 0., 0.5, 0.5))

        
        # create 6 robots
        self.robots = []
        for (i,j) in itertools.product(range(3), range(2)):
            self.robots.append(Robot([1. * i + 0.5, 1. * j - 0.5, 0.3], 2*i+j, self.dt))
            p.stepSimulation()
        
        self.time = 0.0
        
        self.stepSimulation()
        self.stepSimulation()

    def reset(self):
        """
        Resets the position of all the robots
        """
        for r in self.robots:
            r.reset()
        p.stepSimulation()
        
    def stepSimulation(self):
        """
        Simulates one step simulation
        """
        
        # for each robot construct list of neighbors
        for r in self.robots:
            r.neighbors = [] #reset neighbors
            r.messages_received = [] #reset message received
            pos1, or1 = r.get_pos_and_orientation()
            for j,r2 in enumerate(self.robots):
                if(r.id != r2.id):
                    pos2, or2 = r2.get_pos_and_orientation()
                    if(np.linalg.norm(pos1-pos2) < self.max_communication_distance):
                        r.neighbors.append(j)
        
        # for each robot send and receive messages
        for i,r in enumerate(self.robots):
            for msg in r.messages_to_send:
                if msg[0] in r.neighbors: #then we can send the message
                    self.robots[msg[0]].messages_received.append([i, msg[1]]) #add the sender id
            r.messages_to_send = []
        
        # update the controllers
        if self.time > 1.0:
            for r in self.robots:
                r.compute_controller()
        
        # do one simulation step
        p.stepSimulation()
        self.time += self.dt
        
