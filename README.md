# RO47005 Project Planning & Decision Making (Group 30)  
------------------------------
This file contains all the information and instructions related to the final assignment of the robot practical course.

## Practical information
------------------------------
This section contains some useful practical information.
Name                | Student Nr.
--------------------|-------------
Joris Weeda         | 5641551
Demi Breen          | 4591380
Kenny Brandon       | 4461428

## Introduction to the assignment
------------------------------
For the project, the chosen subject contains the implementation of a small four-wheel car as an automated guided vehicle (AGV). This vehicle will be used to navigate to a desired point within the area of a standard warehouse. The purpose of the autonomous vehicle is to improve the logistics of gathering the products. This can be done by transporting products, which are placed on and taken off by workers, to certain locations within the warehouse.

## Setup of the packages
------------------------------
There are three main packages which are combined in the main to create and support the automated vehicle in a simulated environment.
* PDM
  * _global_planner_
      * __init__.py
      * rrt.py
  * _local_planner_
      * __init__.py
      * mpc_controller.py
      * vehicle_dynamics.py
  * _map_environment_
      * __init__.py
      * environment.py
      * obstacles.py
  * _main.py_
