# rosclass_minimal_nodes

Service/client mix on PS1.

Run order
rc_minimal_publisher src/minimal_publisher.cpp

rc_sleepy_minimal_publisher src/sleepy_minimal_publisher.cpp

rc_minimal_subscriber src/minimal_subscriber.cpp

rc_minimal_simulator src/minimal_simulator.cpp

rc_minimal_controller src/minimal_controller.cpp

rc_sin_commander src/sin_commander.cpp

rc_sin_client src/sin_client.cpp

## Example usage
rosrun rosclass_minimal_nodes rc_minimal_simulator

rosrun rosclass_minimal_nodes rc_minimal_controller

rosrun rosclass_minimal_nodes rc_sin_commander

rqt_plot

rosrun rosclass_minimal_nodes rc_sin_client 
Please enter an real value for amplitude:100
Please enter an real value for frequency:2

In rqt_plot, a sinusoidal waveform will appear thanks to the sin_commander, service/client edition. 
## Running tests/demos
    