roslaunch my_ros_package my_launch_file.launch
... logging to /home/akyuz/.ros/log/7b04ee4e-bc28-11ee-bbdb-f9072e06e002/roslaunch-akyuz-8362.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://akyuz:40503/

SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.16.0

NODES
  /
    publisher_node (my_ros_package/publisher_node.py)
    subscriber_node (my_ros_package/subscriber_node.py)

ROS_MASTER_URI=http://localhost:11311

process[publisher_node-1]: started with pid [8376]
process[subscriber_node-2]: started with pid [8377]
[INFO] [1706261657.069607]: Hello, ROS!
[INFO] [1706261658.071297]: Hello, ROS!
[INFO] [1706261658.075804]: Received: Hello, ROS!
[INFO] [1706261659.071303]: Hello, ROS!
[INFO] [1706261659.076108]: Received: Hello, ROS!
[INFO] [1706261660.071320]: Hello, ROS!


roslaunch my_ros_package my_launch_file.launch







