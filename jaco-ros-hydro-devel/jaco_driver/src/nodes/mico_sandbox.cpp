//============================================================================
// Name        : jaco_arm_driver.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS node that subscribes the original mico joint state messages and re-publishes them so that 
//				they are compatible with the old URDF model for the robot
//============================================================================


#include <signal.h> 
#include <vector>
#include <string.h>
   
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>

#include <jaco_msgs/FingerPosition.h>

#include <tf2_msgs/TFMessage.h>

#define PI 3.14159265359

ros::Publisher joint_state_pub;

/*
 * name: ['Shoulder_Joint', 'Arm_Joint', 'Forearm_Joint', 'Wrist_1_Joint', 'Wrist_2_Joint', 'Hand_Joint', 'Finger_1_Proximal_Joint', 'Finger_1_Distal_Joint', 'Finger_2_Proximal_Joint', 'Finger_2_Distal_Joint']

 */
 
float finger_positions[2];
sensor_msgs::JointState current_jpos;


sensor_msgs::JointState current_efforts;
sensor_msgs::JointState last_efforts;
double total_grav_free_effort = 0;
double delta_effort[6];

ros::Publisher c_vel_pub_;

bool heard_efforts = false;

/* CALLBACK functions */

void finger_state_cb (const jaco_msgs::FingerPositionConstPtr& input)
{
	finger_positions[0]=input->finger1;
	finger_positions[1]=input->finger2;
}

void joint_state_cb (const sensor_msgs::JointStateConstPtr& input)
{
	current_jpos = *input;
}

void joint_effort_cb(const sensor_msgs::JointStateConstPtr& input){
	
	//compute the change in efforts if we had already heard the last one
	if (heard_efforts){
		for (int i = 0; i < 6; i ++){
			delta_effort[i] = input->effort[i]-current_efforts.effort[i];
		}
	}
	
	//store the current effort
	current_efforts = *input;
	
	total_grav_free_effort = 0.0;
	for (int i = 0; i < 6; i ++){
		if (current_efforts.effort[i] < 0.0)
			total_grav_free_effort -= (current_efforts.effort[i]);
		else 
			total_grav_free_effort += (current_efforts.effort[i]);
	}
	
	heard_efforts=true;
}


void tf_cb(const tf2_msgs::TFMessageConstPtr& input){
	//ROS_INFO("%i",input->transforms.size());
}




/* behaviors */

void establish_contact(double v_x, double v_y, double v_z){
	
	double timeout_duration = 30.0;
	
	ros::Rate r(100);
	for (int i = 0; i < (int)timeout_duration*100; i ++){
		
		
		ros::spinOnce();
		
		geometry_msgs::TwistStamped T; 
		T.twist.linear.x=v_x;//0.4f;
		T.twist.linear.y=v_y;
		T.twist.linear.z=v_z;//-40.0;
		
		T.twist.angular.x=0.0;
		T.twist.angular.y=0.0;
		T.twist.angular.z=0.0;
		

		c_vel_pub_.publish(T);

		r.sleep();
		
		if (heard_efforts){
			ROS_INFO("Total effort: %f",total_grav_free_effort);
			ROS_INFO("%f, %f, %f, %f, %f, %f",current_efforts.effort[0],current_efforts.effort[1],current_efforts.effort[2],
											current_efforts.effort[3],current_efforts.effort[4],current_efforts.effort[5]);
			
			ROS_INFO("%f, %f, %f, %f, %f, %f",delta_effort[0],delta_effort[1],delta_effort[2],
											delta_effort[3],delta_effort[4],delta_effort[5]);
											
			double total_delta = delta_effort[0]+delta_effort[1]+delta_effort[2]+delta_effort[3]+delta_effort[4]+delta_effort[5];
		
			ROS_INFO("%f",total_delta);
			
			if (total_grav_free_effort > 2.0){
				break;
			}
			
			if (total_delta > 0.3){
				break;
			}
		}
		
		
		
		
		
	}
	
	
	
}


void stroke(double d_x, double d_y, double d_z){
	double duration = 1.0;
	ros::Rate r(100);
	
	for (int i = 0; i < (int)duration*100; i++){
		ros::spinOnce();
		
		geometry_msgs::TwistStamped T; 
		T.twist.linear.x=d_x;//0.4f;
		T.twist.linear.y=d_y;
		T.twist.linear.z=d_z;//-40.0;
		
		T.twist.angular.x=0.0;
		T.twist.angular.y=0.0;
		T.twist.angular.z=0.0;
		
		c_vel_pub_.publish(T);

		r.sleep();
	}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mico_sandbox");
    ros::NodeHandle nh("~");
   
	//create subscriber to joint angles
	ros::Subscriber sub = nh.subscribe ("/mico_arm_driver/out/joint_state", 1, joint_state_cb);
  
	//create subscriber to finger angles
	ros::Subscriber sub_finger = nh.subscribe ("/mico_arm_driver/out/finger_position", 1, finger_state_cb);
	
	//create subscriber to joint torques
	ros::Subscriber sub_torques = nh.subscribe ("/mico_arm_driver/out/joint_efforts", 1, joint_effort_cb);
  
	//subscribe to tf
	ros::Subscriber sub_tf = nh.subscribe ("/tf", 1, tf_cb);
  
	//publisher for cartesian velocity
	c_vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 2);
  
  
	/*establish_contact(0.0,0.0,-0.05f);
  
	stroke(0.1,-0.0,-0.015);
	stroke(0.0,0.1,-0.015);
	stroke(-0.1,0.0,-0.015);
	stroke(0.0,-0.1,-0.015);
	
	//retract
	stroke(0.0,0.0,0.2);*/
	
	
	
	establish_contact(0.05f,0.0,0.00f);
  
	stroke(0.015,0.1,0);
	stroke(0.015,0.0,-0.1);
	stroke(0.015,-0.1,0);
	stroke(0.015,0.0,0.1);
	
	//retract
	stroke(-0.2,0.0,0.0);
	
	/*ros::Rate r(100);
	for (int i = 0; i < 300; i ++){
		
		
		ros::spinOnce();
		
		geometry_msgs::TwistStamped T; 
		T.twist.linear.x=0.5;//0.4f;
		T.twist.linear.y=0;
		T.twist.linear.z=-0.05;//-40.0;
		
		T.twist.angular.x=0.0;
		T.twist.angular.y=0.0;
		T.twist.angular.z=0.0;
		
		ROS_INFO("Publishing vel. command...");
		
		c_vel_pub_.publish(T);
	
		
		r.sleep();
	}*/
  
  
  
	
  
    
    return 0;
}
