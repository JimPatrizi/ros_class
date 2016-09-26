//Jim Patrizi
//ROS client for use with sin_commander, service edition:
// first run: rosrun rosclass_minimal_nodes sin_commander
// then start this node
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <my_action_server/serverAction.h>
#include <iostream>
#include <string>
using namespace std;

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
        const my_action_server::serverResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    bool diff = result->succeed;
    ROS_INFO("got result output = %s", diff ? "true" : "false");
}

int main(int argc, char **argv) { 
	//init node, nodehandle, serviceclient, and srv object
    ros::init(argc, argv, "action_client");
    ros::NodeHandle n;
    my_action_server::serverGoal goal;
    actionlib::SimpleActionClient<my_action_server::serverAction> action_client("my_action", true);
    ROS_INFO("waiting for server: ");
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
    //bool server_exists = action_client.waitForServer(); //wait forever

    if (!server_exists) {
        ROS_WARN("could not connect to server; halting");
        return 0; // bail out; optionally, could print a warning message and retry
    }

    ROS_INFO("connected to action server"); 

    cout << "Please enter an real value for amplitude: ";
    cin >> goal.amplitude;
    cout << "Please enter an real value for frequency: ";
    cin >> goal.frequency;
    cout << "Please enter an real integer for cycle number: ";

    action_client.sendGoal(goal, &doneCb);

    bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
    if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result");
            return 0;
        }
        else {
            ROS_INFO("Goal Succeeded!");
            ROS_INFO("Amplitude reached: %f", goal.amplitude);
            ROS_INFO("Frequency reached: %f", goal.frequency);
            ROS_INFO("Goal Succeeded!");
        }
        
        

    /*
    while(true)
    {
    
    ROS_INFO("Amplitude: %f",srv.request.amplitude);//print out values for ampltiude and frequency
    ROS_INFO("Frequency: %f",srv.request.frequency);
	}
	else
	//ROS_ERROR("Something bad happened");
    //if here, server returned a result to client
    */
    return 0;
}