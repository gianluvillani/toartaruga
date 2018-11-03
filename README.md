# Toartaruga
Group 2 code for SML Grand Challenge Project Course (KTH)

# Basic Instructions:
- Clone the repo into your catkin_ws/src
- Catkin_make into your repo

# Run the manual(teleop) controller
Start roscore
<pre><code>
  $ roscore
</code></pre>

Start serial communication
<pre><code>
$ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
</code></pre>

Activate key_teleop to send signals via keyboard:
<pre><code>
$ rosrun key_teleop key_teleop.py 
</code></pre>


Translate message from Twist to lli_ctrl_request:
<pre><code>
$ rosrun custom_teleop teleop_transcriber.py
</code></pre>

If you want to simulate the vehicle as well
<pre><code>
$ rosrun vehicle_simulation simulator.py 
</code></pre>
