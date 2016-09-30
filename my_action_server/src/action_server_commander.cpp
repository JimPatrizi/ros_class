// my_action_server: a simple action server for commanding sinusoids
// Jim Patrizi

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Float64.h> 
#include <math.h>
#include <iostream>
#include <string>
#include <sstream>

//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../my_action_server/action/server.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (server) and appended name (Action)
#include <my_action_server/serverAction.h>
using namespace std;

int g_count = 0;
bool g_count_failure = false;
std_msgs::Float64 g_vel_cmd;
std_msgs::Float64 zero;
double amplitude_;
double frequency_;
int cycle_num_;

//callback function for vel_cmd topic
void myCallbackVelCmd(const std_msgs::Float64& message_holder) {
    // check for data on topic "vel_cmd" 
    //ROS_INFO("received velocity command value is: %f", message_holder.data);
    g_vel_cmd.data = message_holder.data; // post the received data in a global var for access by 
    //main prog. 
}


class MyActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<my_action_server::serverAction> as_;
    ros::Publisher my_publisher_object;
    
    // here are some message types to communicate with our client(s)
    my_action_server::serverGoal goal_; // goal message, received from client
    my_action_server::serverResult result_; // put results here, to be sent back to the client when done w/ goal
    my_action_server::serverFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client



public:
    MyActionServer(); //define the body of the constructor outside of class definition

    ~MyActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<my_action_server::serverAction>::GoalConstPtr& goal);
};

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  

MyActionServer::MyActionServer() :
   as_(nh_, "my_action", boost::bind(&MyActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "my_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of MyActionServer...");
    // do any other desired initializations here...specific to your implementation

   // ros::NodeHandle nh; // node handle 
    
    as_.start(); //start the server running
}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <example_action_server::serverAction> customizes the simple action server to use our own "action" message 
// defined in our package, "my_action_server", in the subdirectory "action", called "server.action"
// The name "server" is prepended to other message types created automatically during compilation.
// e.g.,  "serverAction" is auto-generated from (our) base name "server" and generic name "Action"
void MyActionServer::executeCB(const actionlib::SimpleActionServer<my_action_server::serverAction>::GoalConstPtr& goal) {
    //ROS_INFO("in executeCB");
    //ROS_INFO("goal input is: %d", goal->input);
    //do work here: this is where your interesting code goes

   // result_.output = g_count; // we'll use the member variable result_, defined in our class
   // result_.goal_stamp = goal->input;
    amplitude_ = goal ->amplitude;
    frequency_ = goal ->frequency;
    cycle_num_ = goal ->cycle_num;
    
    ros::Publisher my_publisher_object = nh_.advertise<std_msgs::Float64>("vel_cmd", 1);

    
    //initilize 
    double dt_sin = 0.001;
    g_vel_cmd.data = 0.0;
    double loop_stop = (cycle_num_/frequency_);
    ros::Rate naptime(1/dt_sin); 
    double rad_per_sec = 2*M_PI*frequency_;
    

    double t = 0.0;
    while (t <= loop_stop)
        {
            //do sin stuff here
            g_vel_cmd.data = amplitude_*sin(rad_per_sec*t);//calculates sine wave with user designated freq and amp
            t = t + dt_sin;
            //dt_sin = dt_sin+0.01; //increments delt t 
            my_publisher_object.publish(g_vel_cmd); // publish the control effort computed by this 
            //sin_commander
            ROS_INFO("vel_cmd command = %f", g_vel_cmd.data);
            
            naptime.sleep(); // wait for remainder of specified period; 
            // for debug, induce a halt if we ever get our client/server communications out of sync
        }
    zero.data = 0.0;
    
    
    my_publisher_object.publish(zero);    
    
        
    as_.setSucceeded(result_);
    //frequency_
    //cycle_num_
    // the class owns the action server, so we can use its member methods here
   
    // DEBUG: if client and server remain in sync, all is well--else whine and complain and quit
    // NOTE: this is NOT generically useful code; server should be happy to accept new clients at any time, and
    // no client should need to know how many goals the server has serviced to date
    /*if (g_count != goal->input) {
        ROS_WARN("hey--mismatch!");
        ROS_INFO("g_count = %d; goal_stamp = %d", g_count, result_.goal_stamp);
        g_count_failure = true; //set a flag to commit suicide
        ROS_WARN("informing client of aborted goal");
        as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
    }
    else {
         as_.setSucceeded(result_); // tell the client that we were successful acting on the request, and return the "result" message
    }
    */
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "action_server_node"); // name this node
    
    //double startTime = System.currentTimeMillis();//whoops this is java, look up C++

    ROS_INFO("instantiating the action server commander: ");

    MyActionServer as_object; // create an instance of the class "ExampleActionServer"
    
    ROS_INFO("going into spin");
    ROS_INFO("waiting on action client...");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    
    while (ros::ok())
    {
        ros::spin();
    }
    return 0;
}

