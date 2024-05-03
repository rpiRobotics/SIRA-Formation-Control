cd /home/$USER/SIRA-Formation-Control/catkin_ws/src/obstacle_detector
gnome-terminal -- roslaunch obstacle_detector nodes.launch
gnome-terminal -- rosrun control_nodes obstacle_dist.py
rosservice call /sirab/netft/bias "toBias: true\nforceMax: 300.0\ntorqueMax: 200.0"
gnome-terminal -- rosrun control_nodes control_node.py
