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

The wheel joints in the URDF are ``continuous``, but the planar_move plug-in moves
the whole body kinematically, so nothing would turn them. run_bridge_sim.sh also
starts ``stm32_bridge/wheel_joint_states``: it integrates /cmd_vel into
/joint_states (using the same C mecanum_inverse() as the bridge), so RViz2 shows
the four wheels spinning. It is display-only: no /odom, no serial traffic.
Set WHEEL_SPIN=0 to skip it, or run it by hand:
  ros2 run stm32_bridge wheel_joint_states --ros-args -p use_sim_time:=true

Files
-----
stm32_bridge/         ROS2 package source (rclpy nodes + protocol + kinematics);
                      also holds wheel_joint_states, the display-only wheel animation
fake_stm32.py         STM32 stand-in, creates a pty and speaks the serial protocol
libomninav_core.so    x86-64 build of firmware_core (the ONLY source of the mecanum math)
pyserial_vendor/      pyserial 3.5 (BSD-3-Clause, pure Python), used only if the VM
                      has no pyserial and its apt mirror refuses the package
setup_bridge_env.sh   copies the items above into ~/omninav_ws and colcon-builds the package
run_bridge_sim.sh     starts fake_stm32.py on a pty, then starts stm32_bridge on that pty
check_odom.sh         graph probe (environment + nodes + topics + /odom count)
check_odom.py         the rclpy part of check_odom.sh (also usable on its own)
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

   It prints "PTY_SLAVE=/dev/pts/N" first (from fake_stm32.py), then starts the
   wheel animation node and finally the bridge on that pty. Ctrl+C stops all of
   them. In RViz2 the robot now drives forward/backward/sideways and the wheels
   turn accordingly; they stop 0.3 s after /cmd_vel stops (same watchdog as the
   bridge).

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
* apt says "403 Forbidden" for python3-serial, or "pyserial still missing" ->
    This VM's apt mirror is blocked. The kit already carries pyserial 3.5 in
    pyserial_vendor/, and setup_bridge_env.sh installs it into
    ~/omninav_ws/lib/pyserial_vendor with no network; run_bridge_sim.sh puts
    that directory on PYTHONPATH. Just re-run setup_bridge_env.sh.
* "ModuleNotFoundError: No module named 'stm32_bridge'" ->
    fake_stm32.py must sit in ~/omninav_ws/tools/ and the package in
    ~/omninav_ws/src/stm32_bridge/. Re-run setup_bridge_env.sh.
* "CoreLibraryNotFound" -> export OMNINAV_CORE_SO=~/omninav_ws/lib/libomninav_core.so
    (run_bridge_sim.sh already exports it for both processes it starts).
* "No module named 'serial'" -> sudo apt-get install -y python3-serial
* RViz2 shows the robot but the wheels are missing or frozen ->
    The wheels only get a TF once /joint_states arrives. In bridge mode
    run_bridge_sim.sh starts wheel_joint_states for you; check its log
    (/tmp/omninav_wheel_spin.log) and that the terminal B process is still alive.
    Without the bridge simulation, start it by hand:
      ros2 run stm32_bridge wheel_joint_states --ros-args -p use_sim_time:=true
    or add wheel_anim:=true to the ros2 launch command.
* /odom published twice / TF warnings about multiple publishers ->
    Gazebo was started without gazebo_odom:=false. Restart it with that flag.
* "ros2 topic hz /odom" prints "topic [/odom] does not appear to be published
  yet" -> run the graph probe instead of the ros2 CLI:
      bash /media/sf_vmshare/omninav_bridge_sim/check_odom.sh
  It does not depend on the ros2 CLI daemon and prints ROS_DOMAIN_ID /
  RMW_IMPLEMENTATION, the discovered nodes, and the /odom message count.
  Most likely causes, in order:
    a) the terminal that runs run_bridge_sim.sh was closed or Ctrl+C'ed (it is
       the only /odom publisher, and closing it stops both processes);
    b) this terminal's ROS_DOMAIN_ID or RMW_IMPLEMENTATION differs from the
       bridge terminal's (they are printed by both run_bridge_sim.sh and
       check_odom.sh -- compare the two lines and export the same values);
    c) Gazebo was not restarted with gazebo_odom:=false (the old plug-in then
       competes for /odom).
* Bridge floods "measured E-frame interval ... far exceeds the nominal period
  20.0 ms" -> diagnostic only. The bridge integrates odometry with the
  contract's nominal 20 ms, never with the measured interval, so the pose stays
  correct; the warnings mean the VM occasionally starves the fake STM32 thread
  (Gazebo software rendering). Ignore, or reduce Gazebo load.
* Bridge reports serial timeouts -> make sure no other program opened the pty;
    restart run_bridge_sim.sh (the pty path changes every run).
