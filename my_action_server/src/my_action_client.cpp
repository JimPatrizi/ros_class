//Jim Patrizi
//ROS action client for use with action_server_commander,:
// first, setup minimal controller, simulator, rqt plot with: roslaunch my_action_server ps3_simulator.launch 
// next run: rosrun my_action_server my_action_server_commander
// then start this node
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <my_action_server/serverAction.h>
#include <iostream>
#include <string>
using namespace std;

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
//waits until the result is returned from the action server to activate
void doneCb(const actionlib::SimpleClientGoalState& state,
        const my_action_server::serverResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    bool diff = result->succeed;
    //ROS_INFO("got result output = %s", diff ? "true" : "false");
}

//main of action client
//prompts user for amplitude, frequency, and num of cycles to be commanded to action server
//these values are then sent to the action server as goals
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
   
    if (!server_exists) {
        ROS_WARN("could not connect to server; halting");
        return 0; // bail out; optionally, could print a warning message and retry
    }

    ROS_INFO("connected to action server"); 

    //gets user prompted values sent as goals to server
    cout << "Please enter an real value for amplitude: ";
    cin >> goal.amplitude;
    cout << "Please enter an real value for frequency: ";
    cin >> goal.frequency;
    cout << "Please enter an real integer for cycle number: ";
    cin >> goal.cycle_num;

    //sends the goal and goes into the doneCb, then waits for the server to do the work
    action_client.sendGoal(goal, &doneCb);

    /** This will have the client wait for 205 seconds before timeout, input num of cycles so that it
    does not take longer than 205 seconds **/
    bool finished_before_timeout = action_client.waitForResult(ros::Duration(205.0));
    if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result");
            return 0;
        }
        else {
            ROS_INFO("Goal Succeeded!");
            ROS_INFO("Amplitude reached: %f", goal.amplitude);
            ROS_INFO("Frequency reached: %f", goal.frequency);
            ROS_INFO("Cycles reached: %d", goal.cycle_num);
            ROS_INFO("Goal Succeeded!");
        }
     
    return 0;
}