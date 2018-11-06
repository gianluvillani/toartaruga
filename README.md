# Toartaruga
Group 2 code for SML Grand Challenge Project Course (KTH)

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

### Simulator package (Week 1 - Friday)
The simulator implements a simple kinematic bicycle model and a visualization in RViz. The discretized model used is taken from https://github.com/MPC-Berkeley/barc/wiki/Car-Model.
The simulated states are computed as often as a new input is sent to the car. 
Possible features to be implemented/modified:
- Control should not be necessarily 'synchronized' with the simulation. More explanation later



### Control package (Week 1 - Monday)
The controller implements a simple pure pursuit algorithm for steering angle computation. At the moment the input velocity is kept fixed. 
Subscribes to Path, Odometry topics.
Publishes to lli_ctrl_request topic.

Possible features to be implemented/modified:
- Adaptive Pure Pursuit


