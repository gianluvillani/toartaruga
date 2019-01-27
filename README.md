# Toartaruga
Group 2 code for SML Grand Challenge Project Course (KTH)


## Current state:
Everything working in simulation (Pure Pursuit). Tested in SML on the real platform. Obstacle detection implemented.


### Basic Instructions:
- Clone the repo into your catkin_ws/src
- Catkin_make into your repo

### Run the manual(teleop) controller
Start roscore  
<code>
mkdir -p ./ws/src
cd ./ws
catkin init
cd src
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

### Simulator package (Week 0 - Friday)
The simulator implements a simple kinematic bicycle model and a visualization in RViz. The discretized model used is taken from https://github.com/MPC-Berkeley/barc/wiki/Car-Model.
The simulated states are computed as often as a new input is sent to the car. 
Possible features to be implemented/modified:
- Control should not be necessarily 'synchronized' with the simulation. More explanation later


### Control package (Week 1 - Tuesday)
The controller implements a simple pure pursuit algorithm for steering angle computation. At the moment the input velocity is kept fixed. 
Subscribes to Path, Odometry topics.
Publishes to lli_ctrl_request topic.

If you want to run the pure pursuit controller:   
<code>
  $ rosrun first_controller pure_pursuit.py   
</code>

### Pure Pursuit validation + Launch files (Week 1 - Wednesday)
The pure pursuit controller works both in simulation and real world. Reference path, circle with 1m radius. 


To run complete autonomous path following   
<code>
  $ roslaunch monitor main.launch 
</code>

### Obstacle detection (Week 2 - Sunday)
The obstacle_detector package https://github.com/tysik/obstacle_detector is used to detect obstacles. The detected obstacles are published as circles and segments. For further information check the documentation at the repository above. 
We also implemented a simple transform broadcaster in order to place the obstacles in the global frame qualisys. (Check transform_broadcaster.py in transform_broadcaster package).
If you want to visualize the obstacles on your own computer, be sure to install the obstacle_detector as well.
To run complete autonomous path following with obstacle detection  follow the commands listed below.

Initialize the car and comunication with qualisys               
<code>
    $ roslaunch monitor main.launch 
</code>

Start the lidar                     
<code>
    $ roslaunch rplidar_ros rplidar.launch
</code>

Run the transformation broadcaster            
<code>
    $ rosrun transform_broadcaster transform_broadcaster.py 
</code>

Launch the obstacle detector                  
<code>      
    $ roslaunch obstacle_detector nodes.launch
</code>

Launch rviz on your local computer;)
OBS. If you want to visualize obstacles on your laptop be sure to correctly build the obstacle_detector package.


### Obstacle emergency stop (Week 3)
A simple state machine has been implemented that handles the emergency stop and restart of the car when some obstacle is detected. (supervisor_dummy.py). The car is able to stop if an obstacle is detected inside the danger area and restart when the obstacle is not detected anymore.
The node obstacle_danger_measure (check name) checks all the obstacles and publishes a danger message (Float32). 
This message is continuosly checked by the state_machine that according to its value outputs the right decisions to the controller.

### Platooning PID (Week 4 - Sunday)
For the ghost publisher:  
Keyboard controller 

<code>
     $  rosrun key_teleop key_teleop.py
</code>

Simulator, /simulator/odom can be used as a ghost car to be followed                  
<code>      
    $  rosrun vehicle_simulation vehicle_simulation.py
</code>

### Static Obstacle Avoidance (Week 5)
The vehicle is able to perceive an obstacle on the path (one at a time) and to replan around it. The obstacle is assumed to be static and it is assumed that the obstacle is perceived early enough to avoid it. 

### Dynamic Obstacle Avoidance(failed) and Path Memorization (Week 6)
A new replanner, that takes into account all the obstacles on the path has been designed. Nevertheless, given the reactive nature of the planner and the noise unfiltered data used, that planned path was too unstable to be actually followed by the real car.
A path memorization node has been implemented in order to memorize the path of the leader car whenever the follower car is stopped somewher else performing some tasks.

### Full demo command list
To start following:  
<code>
    $ rostopic pub /another_car std_msgs/Bool "data: True"
</code>

To stop following and start memorization:  
<code>
    $ rostopic pub /another_car std_msgs/Bool "data: False"
  
    $ rostopic pub /start_memorizing std_msgs/Bool "data: True"
</code>

To start following the memorized path:   
<code>
    $ rostopic pub /start_publish_top std_msgs/Float32 "data: 1.0"
</code>
