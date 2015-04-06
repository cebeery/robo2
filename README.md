To run:

1. Work through the ROS tutorial for the hector quadrotor stack http://wiki.ros.org/hector_quadrotor/Tutorials/Quadrotor%20outdoor%20flight%20demo. Be sure you create a catkin_ws.
2. Follow the instructions in worlds/README.md to put the empty world files in the appropriate places
3. Follow the instructions in shell/README.md so you can use executables to start Gazebo/RViz
4. Put the directory quadcopter_control into catkin_ws/src, and, inside catkin_ws, ```catkin_make```
5. In one terminal window, start Gazebo/RViz using the ```emptyworld``` executable
6. In another terminal window, run the control program: ```rosrun quadcopter_control control_flat.py```
