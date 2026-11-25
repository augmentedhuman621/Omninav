OmniNav bridge-simulation kit (Gazebo Classic 11 + ROS2 Humble, Ubuntu 22.04 VM)
==============================================================================

What it does
------------
Moves the /cmd_vel -> /odom chain out of the Gazebo plug-in and into the real
production path used on the robot:

  /cmd_vel  ->  stm32_bridge  ->  "V,vx,vy,w" over a pty  ->  fake_stm32.py
                                                                 (STM32 stand-in)
  fake_stm32.py  ->  "E,..." encoder frames  ->  stm32_bridge  ->  /odom + TF odom->base_link

Gazebo only renders the world and moves the robot body; the bridge owns /odom
and the odom->base_link TF, so exactly one publisher is active.

Files
-----
stm32_bridge/         ROS2 package source (rclpy node + protocol + kinematics)
fake_stm32.py         STM32 stand-in, creates a pty and speaks the serial protocol
libomninav_core.so    x86-64 build of firmware_core (the ONLY source of the mecanum math)
setup_bridge_env.sh   copies the three items above into ~/omninav_ws and colcon-builds the package
run_bridge_sim.sh     starts fake_stm32.py on a pty, then starts stm32_bridge on that pty
README.txt            this file

Steps
-----
0. Stop any running Gazebo launch first (Ctrl+C in the terminal that runs it).

1. Install / rebuild the bridge package (one-off, safe to re-run):

     bash /media/sf_vmshare/omninav_bridge_sim/setup_bridge_env.sh

   It creates ~/omninav_ws/{src,tools,lib}, copies the files, installs pyserial
   if missing (it may ask for the VM password), and runs colcon build.

2. Start Gazebo with the plug-in odom switched OFF (new terminal A):

     source /opt/ros/humble/setup.bash
     source ~/omninav_ws/install/setup.bash
     export GAZEBO_MODEL_DATABASE_URI=
     ros2 launch omninav_sim gazebo.launch.py gui:=false gazebo_odom:=false

   Note: setup_bridge_env.sh also refreshes ~/omninav_ws/src/omninav_sim from the
   copy next to this kit, because bridge mode needs the new gazebo_odom launch
   argument. If that copy is missing, re-copy it yourself:
     bash /media/sf_vmshare/setup_sim_env.sh

3. Start the fake STM32 + bridge (new terminal B):

     bash /media/sf_vmshare/omninav_bridge_sim/run_bridge_sim.sh

   It prints "PTY_SLAVE=/dev/pts/N" first (from fake_stm32.py), then the bridge
   connects to that pty. Ctrl+C stops both.

4. Verify (new terminal C):

     source /opt/ros/humble/setup.bash
     source ~/omninav_ws/install/setup.bash
     ros2 topic hz /odom                 # ~50 Hz, from stm32_bridge
     ros2 run tf2_ros tf2_echo odom base_link
     ros2 topic hz /scan                 # ~10 Hz, from Gazebo
     timeout 3 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
     ros2 topic echo --once /odom --field pose.pose.position

   Expected: after the 3 s move, odom position.x is about 0.6 m (open loop; the
   bridge integrates encoder counts reported by the fake STM32).

Troubleshooting
---------------
* "ModuleNotFoundError: No module named 'stm32_bridge'" ->
    fake_stm32.py must sit in ~/omninav_ws/tools/ and the package in
    ~/omninav_ws/src/stm32_bridge/. Re-run setup_bridge_env.sh.
* "CoreLibraryNotFound" -> export OMNINAV_CORE_SO=~/omninav_ws/lib/libomninav_core.so
    (run_bridge_sim.sh already exports it for both processes it starts).
* "No module named 'serial'" -> sudo apt-get install -y python3-serial
* /odom published twice / TF warnings about multiple publishers ->
    Gazebo was started without gazebo_odom:=false. Restart it with that flag.
* Bridge reports serial timeouts -> make sure no other program opened the pty;
    restart run_bridge_sim.sh (the pty path changes every run).
