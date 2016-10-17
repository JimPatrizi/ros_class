//Jim Patrizi
// sine_commander node, gazebo plugin edition
// commands sinusoidal velocities to the minimal_joint_controller node
// publishes velocity on topics "pos_cmd" and "pos_cmd_2" and pos_cmd3 for three different sinusoidal waveforms defined below
#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_sine_commander"); // name of this node will be "minimal_publisher2"
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher my_publisher_sine_1 = n.advertise<std_msgs::Float64>("/one_DOF_robot/joint1_position_controller/command", 1);
    ros::Publisher my_publisher_sine_2 = n.advertise<std_msgs::Float64>("/one_DOF_robot/joint2_position_controller/command", 1);
    ros::Publisher my_publisher_sine_3 = n.advertise<std_msgs::Float64>("/one_DOF_robot/joint3_position_controller/command", 1);
    
    std_msgs::Float64 g_pos_cmd;
    std_msgs::Float64 g_pos_cmd_2;
    std_msgs::Float64 g_pos_cmd_3;
   
    ros::Rate naptime(1.0/0.1); //create a ros object from the ros “Rate” class; sets a good update rate for sinusoidal waveform
    
    //initilize std_msgs used for publishing to pos_cmd and pos_cmd_2
    g_pos_cmd.data = 0.0;
    g_pos_cmd_2.data = 0.0;
    g_pos_cmd_3.data = 0.0;

    //for sin computation, predefined variables
    double amplitude = 5.0;
    double frequency1 = 5.0;
    double frequency2 = 10.0;
    double frequency3 = 2.0;
    double rad_per_sec1 = 2*M_PI*frequency1;
    double rad_per_sec2 = 2*M_PI*frequency2;
    double rad_per_sec3 = 2*M_PI*frequency3;
    double t_sin = 0.0;
    
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok()) 
    {
        g_pos_cmd.data = amplitude*sin(rad_per_sec1*t_sin); //computes sine command for joint 1
        g_pos_cmd_2.data = amplitude*sin(rad_per_sec2*t_sin); //computes sine command for joint 2, notice both are at the same amp, but different frequencies.
        g_pos_cmd_3.data = amplitude*sin(rad_per_sec3*t_sin);
        t_sin = t_sin + 0.01;//increments sine waves 
        //publish to both topics
        my_publisher_sine_1.publish(g_pos_cmd);
        my_publisher_sine_2.publish(g_pos_cmd_2);
        my_publisher_sine_3.publish(g_pos_cmd_3);
	    naptime.sleep(); 
    }
}

