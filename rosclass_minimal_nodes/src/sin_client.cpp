//Jim Patrizi
//ROS client for use with sin_commander, service edition:
// first run: rosrun rosclass_minimal_nodes sin_commander
// then start this node
#include <ros/ros.h>
#include <rosclass_minimal_nodes/sine_msg.h>
#include <iostream>
#include <string>
using namespace std;

int main(int argc, char **argv) { 
	//init node, nodehandle, serviceclient, and srv object
    ros::init(argc, argv, "sin_client");
    ros::NodeHandle n;
    ros::ServiceClient client =  n.serviceClient<rosclass_minimal_nodes::sine_msg>("sin_cmd");
    rosclass_minimal_nodes::sine_msg srv;
    //Obtain user input for desired amplitude and frequency
    cout << "Please enter an real value for amplitude: ";
    cin >> srv.request.amplitude;
    cout << "Please enter an real value for frequency: ";
    cin >> srv.request.frequency;

    //have the client call the service, this needed to be after the user specified values
    //else the values of ampplitude and frequency would of been always 0
    if(client.call(srv))
    {
    
    ROS_INFO("Amplitude: %f",srv.request.amplitude);//print out values for ampltiude and frequency
    ROS_INFO("Frequency: %f",srv.request.frequency);
	}
	else
	ROS_ERROR("Something bad happened");
   
    return 0;
}