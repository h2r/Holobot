roslaunch movo_remote_teleop movo_remote_teleop.launch &
roslaunch movo_demos map_nav.launch &
rosrun tf tf_echo /map /base_link &
python tf_listener.py
