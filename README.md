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
Possible improvements:
- Decrease oscillations (Low pass filter/Adaptive pp/Something else). DONE
- Change parameters on the go with keyboard. (Path/Control parameters)
- Record data (e.g. Average deviation, steering angle history, speed history). ALMOST DONE. IMPROVEMENT ON THE GO

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
<code>
    $ rosrun key_teleop key_teleop.py
</code>

Launch the obstacle detector                  
<code>      
    $ rosrun vehicle_simulation vehicle_simulation.py
</code>

IN PROGRESS
