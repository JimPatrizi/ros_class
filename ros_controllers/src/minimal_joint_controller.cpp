/**
James Patrizi
EECS 397: Modern Robotics Programming
PS4 Minimal Joint Controller
See ReadMe for example usage, designed with use with sine_commander, and the model designated
in minimal_robot_description.urdf
**/
#include <ros/ros.h> //ALWAYS need to include this
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <sensor_msgs/JointState.h>
#include <string.h>
#include <stdio.h>  
#include <std_msgs/Float64.h>
#include <math.h>

using namespace std;

//a simple saturation function; provide saturation threshold, sat_val, and arg to be saturated, val
double sat(double val, double sat_val) {
    if (val>sat_val)
        return (sat_val);
    if (val< -sat_val)
        return (-sat_val);
    return val;
    
}

double g_pos_cmd=0.0; //position command input, for joint 1-- global var

//callback for both topics to control both joints defined 
void posCmdCB(const std_msgs::Float64& pos_cmd_msg) 
{ 
  ROS_INFO("received value of pos_cmd is: %f",pos_cmd_msg.data); 
  g_pos_cmd = pos_cmd_msg.data;
} 

double g_pos_cmd_2=0.0; //position command input, for joint 2-- global var
void posCmdCB_2(const std_msgs::Float64& pos_cmd_msg) 
{ 
  ROS_INFO("received value of pos_cmd is: %f",pos_cmd_msg.data); 
  g_pos_cmd_2 = pos_cmd_msg.data;
} 


//main idea was to copy Newman's code and logic for a second joint, required making a new objects for the additional joint
int main(int argc, char **argv) {
    ros::init(argc, argv, "minimal_joint_controller");
    ros::NodeHandle nh;
    ros::Duration half_sec(0.5);
    
    // make sure service is available before attempting to proceed, else node will crash
    bool service_ready = false;
    while (!service_ready) {
      service_ready = ros::service::exists("/gazebo/apply_joint_effort",true);
      ROS_INFO("waiting for apply_joint_effort service");
      half_sec.sleep();
    }
    ROS_INFO("apply_joint_effort service exists");

    ros::ServiceClient set_trq_client = 
       nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
    
    service_ready = false;
    while (!service_ready) {
      service_ready = ros::service::exists("/gazebo/get_joint_properties",true);
      ROS_INFO("waiting for /gazebo/get_joint_properties service");
      half_sec.sleep();
    }
    ROS_INFO("/gazebo/get_joint_properties service exists");// gets here just fine
    
    ros::ServiceClient get_jnt_state_client = 
    nh.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");

    gazebo_msgs::ApplyJointEffort effort_cmd_srv_msg_1;
    gazebo_msgs::GetJointProperties get_joint_state_srv_msg_1;
    gazebo_msgs::ApplyJointEffort effort_cmd_srv_msg_2;
    gazebo_msgs::GetJointProperties get_joint_state_srv_msg_2;
    
    ros::Publisher trq_publisher_1 = nh.advertise<std_msgs::Float64>("jnt_trq_1", 1); 
    ros::Publisher vel_publisher_1 = nh.advertise<std_msgs::Float64>("jnt_vel_1", 1);     
    ros::Publisher pos_publisher_1 = nh.advertise<std_msgs::Float64>("jnt_pos_1", 1);  
    ros::Publisher joint_state_publisher_1 = nh.advertise<sensor_msgs::JointState>("joint1_states", 1); 

    ros::Publisher trq_publisher_2 = nh.advertise<std_msgs::Float64>("jnt_trq_2", 1); 
    ros::Publisher vel_publisher_2 = nh.advertise<std_msgs::Float64>("jnt_vel_2", 1);     
    ros::Publisher pos_publisher_2 = nh.advertise<std_msgs::Float64>("jnt_pos_2", 1);  
    ros::Publisher joint_state_publisher_2 = nh.advertise<sensor_msgs::JointState>("joint2_states", 1); 


    //designate subscribers for use with callback
    ros::Subscriber pos_cmd_subscriber = nh.subscribe("pos_cmd",1,posCmdCB); 
    ros::Subscriber pos_cmd_subscriber_2 = nh.subscribe("pos_cmd_2",1,posCmdCB_2);
    
    //std_msgs for use with topics 
    std_msgs::Float64 trq_msg_1;
    std_msgs::Float64 q1_msg,q1dot_msg;
    std_msgs::Float64 trq_msg_2;
    std_msgs::Float64 q2_msg,q2dot_msg;
    //msgs for joint state, and interaction with gazebo service
    sensor_msgs::JointState joint_state_msg_1;
    sensor_msgs::JointState joint_state_msg_2;

    //variables to determine torque applied and error calulations
    double q1, q1dot;
    double q2, q2dot;
    double dt = 0.01;
    ros::Duration duration(dt);
    ros::Rate rate_timer(1/dt);
    
    //set joint names to be requested and initlize to a default state '0.0'
    effort_cmd_srv_msg_1.request.joint_name = "joint1";
    effort_cmd_srv_msg_1.request.effort = 0.0;
    effort_cmd_srv_msg_1.request.duration= duration;
    effort_cmd_srv_msg_2.request.joint_name = "joint2";
    effort_cmd_srv_msg_2.request.effort = 0.0;
    effort_cmd_srv_msg_2.request.duration= duration;

    get_joint_state_srv_msg_1.request.joint_name = "joint1";
    get_joint_state_srv_msg_2.request.joint_name = "joint2";//here my mistake was get_joint_state_srv_msg_1 in both places. 
    //double q1_des = 1.0;
    double q1_err;
    double q2_err;
    double Kp = 10.0;
    double Kv = 3;
    double trq_cmd_1;
    double trq_cmd_2;

    
    

    // set up the joint_state_msg_1 fields to define a single joint,
    // called joint1, and initial position and vel values of 0. Same for joint 2 and joint_state_msg_2
	joint_state_msg_1.header.stamp = ros::Time::now();
	joint_state_msg_1.name.push_back("joint1");//look into joint_state_msg_1
    joint_state_msg_2.header.stamp = ros::Time::now();
    joint_state_msg_2.name.push_back("joint2");//look into joint_state_msg_2
    
    joint_state_msg_1.position.push_back(0.0);
    joint_state_msg_1.velocity.push_back(0.0);
    joint_state_msg_2.position.push_back(0.0);
    joint_state_msg_2.velocity.push_back(0.0);

    q1 = 0.0;
    q2 = 0.0;
    bool success;
    while(ros::ok()) {    
	    
        get_jnt_state_client.call(get_joint_state_srv_msg_1);
       
        q1 = get_joint_state_srv_msg_1.response.position[0];
        

        q1_msg.data = q1;
        if(get_jnt_state_client.call(get_joint_state_srv_msg_2)){
            
            ROS_INFO("Service call was a success");
        }
        else{
            ROS_WARN("Service failed");
        }
        q2 = get_joint_state_srv_msg_2.response.position[0];
        //cout << "Size2: " << get_joint_state_srv_msg_2.response.position.size() << endl;
        q2_msg.data = q2;

        pos_publisher_1.publish(q1_msg);
        pos_publisher_2.publish(q2_msg);
        
        q1dot = get_joint_state_srv_msg_1.response.rate[0];
        q2dot = get_joint_state_srv_msg_2.response.rate[0];
        q1dot_msg.data = q1dot;
        q2dot_msg.data = q2dot;
        vel_publisher_1.publish(q1dot_msg);
        vel_publisher_2.publish(q2dot_msg);

	    joint_state_msg_1.header.stamp = ros::Time::now();
        joint_state_msg_1.position[0] = q1; 
        joint_state_msg_1.velocity[0] = q1dot;

        joint_state_msg_2.header.stamp = ros::Time::now();
        joint_state_msg_2.position[0] = q2; 
        joint_state_msg_2.velocity[0] = q2dot;

	    joint_state_publisher_1.publish(joint_state_msg_1);
        joint_state_publisher_2.publish(joint_state_msg_2);

        
        
        //ROS_INFO("q1 = %f;  q1dot = %f",q1,q1dot);
        //watch for periodicity
        q1_err= g_pos_cmd-q1;
        q2_err= g_pos_cmd_2-q2;
        if (q1_err>M_PI) {
            q1_err -= 2*M_PI;
        }
        if (q1_err< -M_PI) {
            q1_err += 2*M_PI;
        }

        if (q2_err>M_PI) {
            q2_err -= 2*M_PI;
        }
        if (q2_err< -M_PI) {
            q2_err += 2*M_PI;
        }           
            
        trq_cmd_1 = Kp*(q1_err)-Kv*q1dot;
        trq_cmd_2 = Kp*(q2_err)-Kv*q2dot;
        //trq_cmd_1 = sat(trq_cmd_1, 10.0); //saturate at 1 N-m
        trq_msg_1.data = trq_cmd_1;
        trq_msg_2.data = trq_cmd_2;
        trq_publisher_1.publish(trq_msg_1);
        trq_publisher_2.publish(trq_msg_2);
        // send torque command to Gazebo
        effort_cmd_srv_msg_1.request.effort = trq_cmd_1;
        effort_cmd_srv_msg_2.request.effort = trq_cmd_2;
        set_trq_client.call(effort_cmd_srv_msg_1);
        set_trq_client.call(effort_cmd_srv_msg_2);
        //make sure service call was successful
        bool result = effort_cmd_srv_msg_1.response.success;
        bool result2 = effort_cmd_srv_msg_2.response.success;
        if (!result && !result2)
            ROS_WARN("service call to apply_joint_effort failed!");
            
    ros::spinOnce();
	rate_timer.sleep();
  }
}
