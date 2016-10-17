# ros_controllers

PS4 Modern Robotics Programming
James Patrizi

This assignment makes use of Gazebo plugins to make 3 joints (2 revolute, and 1 continuous) move at sinusoidal forces. 
Follow the demo procedure below to run my implementation. 

(Once demo is run) From what you can see, I modified the minimal_robot_w_jnt_ctl so that the urdf included 3 joints. Important things to note when modifiying this urdf. 

1) You must have a transmission block per joint
2) Each transmission block must have a unique actuator name
3) You MUST include the following to tell gazebo you want to use its plugin for communication from topics and the joint states

    <gazebo>
     <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
       <robotNamespace>/one_DOF_robot</robotNamespace>
     </plugin>
    </gazebo>

The next thing that is needed is a modfied one_dof_ctl_params.yaml file. This file much specifiy the position controllers for *each* joint that wants to be controlled by the plugin. You can then specify default PID values(control gains) and clamping max and mins. *note* the clamping max and min will only take effect if a revolute joint is specified. You can adjust these params using the rqt_gui rqt_gui command in terminal. 

The last thing you need to bring everything together is a launch file that does the following:
1) Load the joint controller configurations from the yaml file to the param server
2) Spawn my modifed robot into Gazebo (the urdf that has the 3 joints and plugin information)
3) Start up the controller plugins with a controller manager. 

Syntax for 3) involves you naming a joint position controller *per* joint. Note this clip from the launch file:

    <!--start up the controller plug-ins via the controller manager -->
    <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
    output="screen" ns="/one_DOF_robot" args="joint_state_controller joint1_position_controller joint2_position_controller joint3_position_controller"/>
    
There are a total of 3 position controllers for all 3 joints. Once the robot is spawned with the launch file, the plugins activate and load the parameter server and activate these topics:

/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
/one_DOF_robot/joint1_position_controller/command
/one_DOF_robot/joint1_position_controller/pid/parameter_descriptions
/one_DOF_robot/joint1_position_controller/pid/parameter_updates
/one_DOF_robot/joint1_position_controller/state
/one_DOF_robot/joint2_position_controller/command
/one_DOF_robot/joint2_position_controller/pid/parameter_descriptions
/one_DOF_robot/joint2_position_controller/pid/parameter_updates
/one_DOF_robot/joint2_position_controller/state
/one_DOF_robot/joint3_position_controller/command
/one_DOF_robot/joint3_position_controller/pid/parameter_descriptions
/one_DOF_robot/joint3_position_controller/pid/parameter_updates
/one_DOF_robot/joint3_position_controller/state
/one_DOF_robot/joint_states

The topics that are highlighted are the topics that you can command an effort to in gazebo to be performed by your robot. The accompanying effort publisher for this assignment is sine_commander.cpp. This makes 3 publisher objects and publishes 3 distinct sine forces to each of the 3 distinct joints. This results in the behavior of the robot when you follow the demo steps and/or watch the attached movie in this project. 

## Example usage

## Running tests/demos
roscore
roslaunch gazebo_ros empty_world.launch
roslaunch ros_controllers minimal_robot_w_jnt_ctl.launch 
rosrun ros_controllers my_sin_commander 
