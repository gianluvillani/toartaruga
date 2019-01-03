# Toartaruga
Group 2 code for SML Grand Challenge Project Course (KTH)

Short summary of what we have done week by week and instructions about how to run the second part of the final demonstration.

### Basic Instructions:
- Clone the repo into your catkin_ws/src
- Catkin_make into your repo

### Run the manual(teleop) controller
Start roscore  
<code>
  $ roscore
</code>

Start serial communication  
<code>
  $ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
</code>

Activate key_teleop to send signals via keyboard:   
<code>
  $ rosrun key_teleop key_teleop.py 
</code>


Translate message from Twist to lli_ctrl_request:   
<code>
  $ rosrun custom_teleop teleop_transcriber.py
</code>

If you want to simulate and visualize the vehicle as well   
<code>
  $ rosrun vehicle_simulation simulator.py   
  
  $ rviz rviz
</code>

### Simulator package
The simulator implements a simple kinematic bicycle model and a visualization in RViz. The discretized model used is taken from https://github.com/MPC-Berkeley/barc/wiki/Car-Model.
The simulated states are computed as often as a new input is sent to the car. 
Possible features to be implemented/modified:
- Control should not be necessarily 'synchronized' with the simulation. More explanation later


### First controller package 
The controller implements a simple pure pursuit algorithm for steering angle computation. The input velocity is kept fixed. 
Subscribes to Path, Odometry topics.
Publishes to lli_ctrl_request topic.

If you want to run this pursuit controller:   
<code>
  $ rosrun first_controller pure_pursuit.py   
</code>

### Pure Pursuit validation + Launch files (Week 1)
The pure pursuit controller works both in simulation and real world. Reference path, circle with 1m radius. 
Possible improvements:
- Decrease oscillations (Low pass filter/Adaptive pp/Something else). DONE
- Change parameters on the go with keyboard. (Path/Control parameters)
- Record data (e.g. Average deviation, steering angle history, speed history). ALMOST DONE. IMPROVEMENT ON THE GO


### Obstacle detection (Week 2)
The obstacle_detector package https://github.com/tysik/obstacle_detector is used to detect obstacles. The detected obstacles are published as circles and segments. For further information check the documentation at the repository above. 
We also implemented a simple transform broadcaster in order to place the obstacles in the global frame qualisys. (Check transform_broadcaster.py in transform_broadcaster package).
If you want to visualize the obstacles on your own computer, be sure to install the obstacle_detector as well.
To run complete autonomous path following with obstacle detection  follow the commands listed below.

### Obstacle emergency stop (Week 3)
A simple state machine has been implemented that handles the emergency stop and restart of the car when some obstacle is detected. (supervisor_dummy.py). The car is able to stop if an obstacle is detected inside the danger area and restart when the obstacle is not detected anymore.
The node obstacle_measurement checks all the obstacles and publishes a danger message (Float32). 
This message is continuosly checked by the state_machine that according to its value outputs the right decisions to the controller.

### Platooning PID (Week 4)
The vehicle is able to follow another car (leader object) using Qualysis. A PID controller for the distance and the previous pure pursuit controller for the steering were used.

### Static Obstacle Avoidance (Week 5)
The vehicle is able to perceive an obstacle on the path (one at a time) and to replan around it. The obstacle is assumed to be static and it is assumed that the obstacle is perceived early enough to avoid it. 

### Dynamic Obstacle Avoidance(failed) and Path Memorization (Week 6)
A new replanner, that takes into account all the obstacles on the path has been designed. Nevertheless, given the reactive nature of the planner and the noise unfiltered data used, that planned path was too unstable to be actually followed by the real car.
A path memorization node has been implemented in order to memorize the path of the leader car whenever the follower car is stopped somewhere else performing some tasks.

### Demo 2 (Week 7)
Final demonstration part 2: platoonig, path memorization and the obstacle avoidance.

### Full demo 2 command list
To start running the system
<code>
  $ roslaunch monitor main.launch 
</code>
To start following the leader:  
<code>
    $ rostopic pub /other_car std_msgs/Bool "data: True"
</code>

To stop following and start memorization:  
<code>
    $ rostopic pub /other_car std_msgs/Bool "data: False"
  
    $ rostopic pub /start_memorizing std_msgs/Bool "data: True"
</code>

To start following the memorized path:   
<code>
    $ rostopic pub /start_publish_top std_msgs/Float32 "data: 1.0"
</code>

If you want to simulate and visualize the vehicle as well   
<code>
  $ rviz rviz
</code>
