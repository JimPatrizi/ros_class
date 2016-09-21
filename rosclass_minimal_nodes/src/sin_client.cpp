 //example ROS client:
// first run: rosrun example_ROS_service example_ROS_service
// then start this node:  rosrun example_ROS_service example_ROS_client



#include <ros/ros.h>
#include <rosclass_minimal_nodes/sine_msg.h>// this message type is defined in the current package
#include <iostream>
#include <string>
using namespace std;

int main(int argc, char **argv) { 
    ros::init(argc, argv, "sin_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<rosclass_minimal_nodes::sine_msg>("sin_cmd");
    rosclass_minimal_nodes::sine_msg srv;
    //bool found_on_list = false;
    //string in_name;

    if(client.call(srv))
    {
    cout << "Please enter an real value for amplitude: ";
    cin >> srv.response.amplitude;
    cout << "Please enter an real value for frequency: ";
    cin >> srv.response.frequency;
    ROS_INFO("Amplitude: %f",srv.response.amplitude);
    ROS_INFO("Frequency: %f",srv.response.frequency);
    }

	cout << "The value you entered for amplitude is  " << srv.response.amplitude << "and the value for frequency is " << srv.response.frequency;
   
    while(ros::ok()){
    	
    }
    return 0;
}