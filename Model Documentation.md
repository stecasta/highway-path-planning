# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Goals

The goal of this project was to safely drive in a simulated highway environment. In particular the car had to avoid
 collisions and exceeding speed and jerk limits while being able to perform lane changes in order to get past slow
  vehicles.
   
### Path Planning Problem

The path planning problem consists of 3 main steps:

- Prediction
- Behavior Planning
- Trajectory generation

The inputs to the path planning module are sensor data with information of the surrounding environment and the
 localization of the
 vehicle in the environment. With this information the planner should compute a feasible trajectory to be fed to the
  controller.
  
  In the following I will explain how each of this steps was handled in the project.
  
  ### Prediction Phase
  
The vehicle is equipped with multiple sensors (e.g. Lidar, cameras) which carry information about other agents on the
 environment (e.g. cars, pedestrians). With this information it is often useful to predict what those agents are
  about to do in order to plan more meaningful trajectories. The two most common approaches are model
   based and data driven. In general, the complexity of this problem increases with the number of actions that the
    agent can take in the environment. In the case of highway driving, the number of possible actions is limited. If
     we over-simplify we can say that vehicles can only keep lane or change left or right.
     
  In the project I considered that the vehicles would always keep lane. While this greatly simplified the trajectory
   generation process, could be dangerous in the case in which other vehicles didn't check before changing lanes, thus crashing into us.
   
   ### Behavior Planning Phase
   
   In this phase the goal is to plan the high level behavior of the car. A simple and effective way of doing this is
    by using a finite state machine. The challenge in doing so is to find a balance between not defining too many
     states and not over-simplifying the system. In our case we only considered 3 states: 
 - Keep lane
 - Change lane left
 - Change lane right
 
 Whenever the road in front of the vehicle is free, we keep lane and we try to go at 50 miles/hour. If there's a
  vehicle in front of us within 30 meters and it is moving slower then us, then we must start to slow down and at the
   same time check if we can switch to a faster lane. Whenever changing lane we first check if a 30 meters range of
    road is free and if there's a vehicle in a 60 meters range we want our velocity to be greater if it is behind us
     and slower if it is in front.
     
 This logig is implemented in the main.cpp file from line 115 to line 190. In the implementation we loop through
  every vehicle only once and we check which are the feasible options. Then based on the results we decide whether to
   speed up or slow down and whether to keep or change lane.
   
 One possible improvement of the planner could be looking at every lane and pick the fastest one. Another possible
  improvement could be implementing a cost function which maximizes speed while ensuring safety.
   
 ### Trajectory Generation Phase
 
 Once we defined the high level action we want the vehicle to perform, we have to provide a path to the controller in
  order to make the vehicle move. The path consists in x,y coordinates and associated velocities. In this project the
   car will visit a point each 0.2 seconds, so we only
   provide a set of points and the velocity depends on how far from each other these points are. Since
   we already took care of the safety constraints in the behavioral planning phase, in this phase the main focus will
    be in generating a smooth trajectory with minimal jerk. This translates in properly spacing the points so that
     they are not too far from each other and that they are evenly spread. This was implemented by taking advantage
      of the spline library as suggested by the Q&A session of the self-driving project. 
      
   We decide to provide 50 points each iteration and we keep the points from the past iteration so that transitions are
  smoother.
   The number of points between the last point of the previous iteration and the target location  depends on the
    reference velocity based on this
    formula: 
```
 N = target_dist / (.02 * ref_vel / 2.24);
```

where 2.24 is a factor to convert to m/s.

In order to simplify the math we performed to conversion: the first one was from x,y coordinates to Frenet
 coordinates and the second one was from the global frame to the vehicle frame so the the heading was 0.

A possible improvement of the trajectory generator could include generating multiple trajectory and then pick the one
 which minimizes jerk.