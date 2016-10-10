# my_action_server - author - James Patrizi - jdp99

PS3 for Modern Robotics Programming
Commands sinusoid with action server and action client to vel_cmd topic at user specified 
amplitude, frequency, and cycle_number

## Example usage 

Follow these steps for running this project and demoing usage:
catkin_make
roslaunch my_action_server ps3_simulator.launch ##setup of rqt_plot, minimal_controller, and minimal_simulator
rosrun my_action_server my_action_server_commander
rosrun my_action_server my_action_client 


## Running tests/demos
see PS3 writeup for demo/tests