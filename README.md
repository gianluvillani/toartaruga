# Toartaruga
Group 2 code for SML Grand Challenge Project Course (KTH)

# Basic Instructions:
- Clone the repo into your catkin_ws/src
- Catkin_make into your repo

# Run the manual(teleop) controller
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

If you want to simulate the vehicle as well   
<code>
  $ rosrun vehicle_simulation simulator.py 
</code>
