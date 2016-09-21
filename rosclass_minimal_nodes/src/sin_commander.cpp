//Jim Patrizi
// sin_commander node, service edition: 
// prompts for both amplitude and frequency
// commands sinusoidal velocities to the minimal_controller node
// publishes velocity on topic "velocity" 
#include<ros/ros.h> 
#include<std_msgs/Float64.h> 
#include<rosclass_minimal_nodes/sine_msg.h>
#include<math.h>
#include<iostream>
#include<string>
#include<sstream>

using namespace std;

//global variable declaration
std_msgs::Float64 g_vel_cmd;
double amplitude_;
double frequency_;

//callback function for vel_cmd topic
void myCallbackVelCmd(const std_msgs::Float64& message_holder) {
    // check for data on topic "vel_cmd" 
    //ROS_INFO("received velocity command value is: %f", message_holder.data);
    g_vel_cmd.data = message_holder.data; // post the received data in a global var for access by 
    //main prog. 
}

//callback function for sin_cmd service. Retrieves client request for a specific amplitude and frequency
//must be a bool
bool callBack(rosclass_minimal_nodes::sine_msgRequest& request, rosclass_minimal_nodes::sine_msgResponse& response)
{    
	ROS_INFO("callback activated");
    amplitude_ = request.amplitude;
    frequency_ = request.frequency;
    ROS_INFO("amplitude value: %f",request.amplitude);
    ROS_INFO("frequency: %f",request.frequency);
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sin_service"); //name this node 
    // when this compiled code is run, ROS will recognize it as a node called "sin_service"
    ros::NodeHandle nh; // node handle
    ros::ServiceServer service = nh.advertiseService("sin_cmd", callBack);//service object that utlizes callBack
    ros::Subscriber my_subscriber_object = nh.subscribe("vel_cmd", 1, myCallbackVelCmd);
    ros::Publisher my_publisher_object = nh.advertise<std_msgs::Float64>("vel_cmd", 1);

    ros::Rate naptime(1.0/0.1); 

    //initilize 
    double dt_sin = 0.0;
    g_vel_cmd.data = 0.0;
    
    //Y corrdinate evaluated at time t will be the velocity to be commanded
    while (ros::ok()) {
        //do sin stuff here
        g_vel_cmd.data = amplitude_*sin(frequency_*dt_sin);//calculates sine wave with user designated freq and amp
        dt_sin = dt_sin+0.01; //increments delt t 
        my_publisher_object.publish(g_vel_cmd); // publish the control effort computed by this 
        //sin_commander
        ROS_INFO("vel_cmd command = %f", g_vel_cmd.data);
        ros::spinOnce(); //allow data update from callback; 
        naptime.sleep(); // wait for remainder of specified period; 
    }
    return 0; // should never get here, unless roscore dies 
}