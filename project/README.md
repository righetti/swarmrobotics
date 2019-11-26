# Goals of the project

There are several tasks to achieve:
1. Make a square formation in the room
2. Get all the robot out of the room and make a circle formation outside
3. Move the purple ball on the purple square
4. Move the red ball on the red square
5. Get back into the room and make a diamond formation

Each of these tasks need to be achieved using decentralized algorithms seen in class.

# Required software
* Python 3.7
* PyBullet

To install PyBullet with pip type 
```
pip install pybullet
```

To install PyBullet with anaconda type
```
 conda install -c hcc pybullet 
```


Documentation:

https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3

https://github.com/bulletphysics/bullet3/tree/master/docs


# Running a simulation

In the python directory, type 
``` 
python run_simulation.py 
```
It will run the example simulation.

Description of the  python files:
* swarm_simulation.py contains the code to construct the simulation environment
* run_simulation.py is an example script that instantiate an environment and runs the simulation until closed
* robot.py is the code to handle one robot. The class Robot defines access to all sensors of the robot, allows to send and receive messages to neighboring robots and to control the wheels of the robot. The function 'compute_controller' is the function to be changed to implement the code for the project. This function is called at every control cycle (every 4ms) by the simulator and simulates what a real robot controller would do.

Note that you can also use the code in run_simulation.py in a Jupyter Notebook if you prefer.
