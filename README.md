This is the code repository for the BU robotics dashboard

The Dash board uses sercer-client socket programming
in python and is intended to be used with ROS

It displays the names, IP addresses, and two estimates
of the pose, one based on a local estimate and one based on optitrack

To use this run the Robot_Dashboard.py on the main experimental computer 
making sure that the IP listed when it assigns the HOST 
is consistent with the computer that you are using. Also ensure that 
the HOST name on the client scritpt is consistent with the IP of the main computer

Ensure that the robot you are operating with has the Dashboard_Client script
and run it from ssh after you have started the main dashboard, otherwise the
connection will not go through
