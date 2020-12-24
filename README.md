# Autonomous Robots: Model Predictive Control
Repositary for Model Predictive Control Course

Course link: www.udemy.com/course/model-predictive-control/

Required Packages <br>
python=3.7.4 <br>
numpy=1.16.4 <br>
matplotlib=3.1.0 <br>
scipy=1.2.1

<h4>Assigment 0: Intro</h4>
-> Intro to MPC shown on temperature regulation <br>
-> Used to explain cost function and how optimization works

<h4>Assigment 1: Highway speed control</h4>
-> More robotic´s related problem, when car cannot exceed speed limit in 1D track

<h4>Assigment 2: Parking Control</h4>
-> 2D control problem, where goal is to park car at any selected place <br>
-> Bicycle model is used to approximately represent model of the car <br>
-> Cost function is designed so that, sharp turning or accelerating is being punished <br>
-> Nicely shows how even quite hard task can be easily done with MPC

<h4>Assigment 3: Obstacle Avoidance</h4>
-> 2D control problem, where goal is to get to the goal but avoid obstacle at the track <br>
-> This example demonstrate more complex cost function and what problem can occur when optimization gets stuck in local minimum. <br>
-> The problem with local minimum will happen only in simulation and if the car and obstacle starts in straight line. (In real life this specific case would not happen due to sensor noise) <br>
<h5>Assigment 3: Bonus </h5>
-> 3d_cost.py visualize cost function, where can be easily seen problem why optimizer got stuck in local minima

