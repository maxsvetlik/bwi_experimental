//============================================================================
// Name        : jaco_arm.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS driver for controlling the Kinova Jaco robotic manipulator arm
//============================================================================

#include "jaco_driver/jaco_arm.h"
#include <string>
#include <vector>

#define PI 3.14159265359


namespace jaco
{

JacoArm::JacoArm(JacoComm &arm, const ros::NodeHandle &nodeHandle)
    : jaco_comm_(arm), node_handle_(nodeHandle)
{
    /* Set up Services */
    stop_service_ = node_handle_.advertiseService("in/stop", &JacoArm::stopServiceCallback, this);
    start_service_ = node_handle_.advertiseService("in/start", &JacoArm::startServiceCallback, this);
    homing_service_ = node_handle_.advertiseService("in/home_arm", &JacoArm::homeArmServiceCallback, this);

    /* Set up Publishers */
    joint_angles_publisher_ = node_handle_.advertise<jaco_msgs::JointAngles>("out/joint_angles", 2);
    joint_state_publisher_ = node_handle_.advertise<sensor_msgs::JointState>("out/joint_state", 2);
    tool_position_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("out/tool_position", 2);
	cartesian_position_publisher_ = node_handle_.advertise<geometry_msgs::TwistStamped>("out/cartesian_position", 2);
    finger_position_publisher_ = node_handle_.advertise<jaco_msgs::FingerPosition>("out/finger_position", 2);
	joint_effort_publisher_ = node_handle_.advertise<sensor_msgs::JointState>("out/joint_efforts", 2);
	joint_accel_publisher_ = node_handle_.advertise<jaco_msgs::JointAcceleration>("out/actuator_acceleration", 2);

    /* Set up Subscribers*/
    joint_velocity_subscriber_ = node_handle_.subscribe("in/joint_velocity", 1,
                                                      &JacoArm::jointVelocityCallback, this);
    cartesian_velocity_subscriber_ = node_handle_.subscribe("in/cartesian_velocity", 1,
                                                          &JacoArm::cartesianVelocityCallback, this);

    node_handle_.param<double>("status_interval_seconds", status_interval_seconds_, 0.1);
    node_handle_.param<double>("joint_angular_vel_timeout", joint_vel_timeout_seconds_, 0.25);
    node_handle_.param<double>("cartesian_vel_timeout", cartesian_vel_timeout_seconds_, 0.25);
    node_handle_.param<double>("joint_angular_vel_timeout", joint_vel_interval_seconds_, 0.1);
    node_handle_.param<double>("cartesian_vel_timeout", cartesian_vel_interval_seconds_, 0.01);

    status_timer_ = node_handle_.createTimer(ros::Duration(status_interval_seconds_),
                                           &JacoArm::statusTimer, this);

    joint_vel_timer_ = node_handle_.createTimer(ros::Duration(joint_vel_interval_seconds_),
                                              &JacoArm::jointVelocityTimer, this);
    joint_vel_timer_.stop();
    joint_vel_timer_flag_ = false;

    cartesian_vel_timer_ = node_handle_.createTimer(ros::Duration(cartesian_vel_interval_seconds_),
                                                  &JacoArm::cartesianVelocityTimer, this);
    cartesian_vel_timer_.stop();
    cartesian_vel_timer_flag_ = false;

    ROS_INFO("The arm is ready to use.");
}


JacoArm::~JacoArm()
{
}


bool JacoArm::homeArmServiceCallback(jaco_msgs::HomeArm::Request &req, jaco_msgs::HomeArm::Response &res)
{
    jaco_comm_.homeArm();
    jaco_comm_.initFingers();
    res.homearm_result = "JACO ARM HAS BEEN RETURNED HOME";
    return true;
}


void JacoArm::jointVelocityCallback(const jaco_msgs::JointVelocityConstPtr& joint_vel)
{
    if (!jaco_comm_.isStopped())
    {
        joint_velocities_.Actuator1 = joint_vel->joint1;
        joint_velocities_.Actuator2 = joint_vel->joint2;
        joint_velocities_.Actuator3 = joint_vel->joint3;
        joint_velocities_.Actuator4 = joint_vel->joint4;
        joint_velocities_.Actuator5 = joint_vel->joint5;
        joint_velocities_.Actuator6 = joint_vel->joint6;
        last_joint_vel_cmd_time_ = ros::Time().now();

        if (joint_vel_timer_flag_ == false)
        {
            joint_vel_timer_.start();
            joint_vel_timer_flag_ = true;
        }
    }
}


/*!
 * \brief Handler for "stop" service.
 *
 * Instantly stops the arm and prevents further movement until start service is
 * invoked.
 */
bool JacoArm::stopServiceCallback(jaco_msgs::Stop::Request &req, jaco_msgs::Stop::Response &res)
{
    jaco_comm_.stopAPI();
    res.stop_result = "Arm stopped";
    ROS_DEBUG("Arm stop requested");
    return true;
}


/*!
 * \brief Handler for "start" service.
 *
 * Re-enables control of the arm after a stop.
 */
bool JacoArm::startServiceCallback(jaco_msgs::Start::Request &req, jaco_msgs::Start::Response &res)
{
    jaco_comm_.startAPI();
    res.start_result = "Arm started";
    ROS_DEBUG("Arm start requested");
    return true;
}


void JacoArm::cartesianVelocityCallback(const geometry_msgs::TwistStampedConstPtr& cartesian_vel)
{
    ROS_INFO("Heard vel. command...");
    
    if (!jaco_comm_.isStopped())
    {
        cartesian_velocities_.X = cartesian_vel->twist.linear.x;
        cartesian_velocities_.Y = cartesian_vel->twist.linear.y;
        cartesian_velocities_.Z = cartesian_vel->twist.linear.z;
        cartesian_velocities_.ThetaX = cartesian_vel->twist.angular.x;
        cartesian_velocities_.ThetaY = cartesian_vel->twist.angular.y;
        cartesian_velocities_.ThetaZ = cartesian_vel->twist.angular.z;

        last_cartesian_vel_cmd_time_ = ros::Time().now();

        if (cartesian_vel_timer_flag_ == false)
        {
            ROS_INFO("Starting cart. vel. timer...");
           
            cartesian_vel_timer_.start();
            cartesian_vel_timer_flag_ = true;
        }
    }
}


void JacoArm::cartesianVelocityTimer(const ros::TimerEvent&)
{
    double elapsed_time_seconds = ros::Time().now().toSec() - last_cartesian_vel_cmd_time_.toSec();

    if (elapsed_time_seconds > cartesian_vel_timeout_seconds_)
    {
        ROS_INFO("Cartesian vel timed out: %f", elapsed_time_seconds);
        cartesian_vel_timer_.stop();
        cartesian_vel_timer_flag_ = false;
    }
    else
    {
        ROS_INFO("Cart vel timer (%f): %f, %f, %f, %f, %f, %f", elapsed_time_seconds,
                 cartesian_velocities_.X, cartesian_velocities_.Y, cartesian_velocities_.Z,
                 cartesian_velocities_.ThetaX, cartesian_velocities_.ThetaY, cartesian_velocities_.ThetaZ);
        jaco_comm_.setCartesianVelocities(cartesian_velocities_);
    }
}


void JacoArm::jointVelocityTimer(const ros::TimerEvent&)
{
    double elapsed_time_seconds = ros::Time().now().toSec() - last_joint_vel_cmd_time_.toSec();

    if (elapsed_time_seconds > joint_vel_timeout_seconds_)
    {
        ROS_INFO("Joint vel timed out: %f", elapsed_time_seconds);
        joint_vel_timer_.stop();
        joint_vel_timer_flag_ = false;
    }
    else
    {
        ROS_INFO("Joint vel timer (%f): %f, %f, %f, %f, %f, %f", elapsed_time_seconds,
                 joint_velocities_.Actuator1, joint_velocities_.Actuator2, joint_velocities_.Actuator3,
                 joint_velocities_.Actuator4, joint_velocities_.Actuator5, joint_velocities_.Actuator6);
        jaco_comm_.setJointVelocities(joint_velocities_);
    }
}



void JacoArm::publishAcceleration(void){
	AngularAcceleration current_accel;
	jaco_comm_.getActuatorAcceleration(current_accel);
	
	jaco_msgs::JointAcceleration msg;
	
	geometry_msgs::Vector3 j1_a;
	j1_a.x = current_accel.Actuator1_X;
	j1_a.y = current_accel.Actuator1_Y;
	j1_a.z = current_accel.Actuator1_Z;
	msg.values.push_back(j1_a);
	
	geometry_msgs::Vector3 j2_a;
	j2_a.x = current_accel.Actuator2_X;
	j2_a.y = current_accel.Actuator2_Y;
	j2_a.z = current_accel.Actuator2_Z;
	msg.values.push_back(j2_a);
	
	geometry_msgs::Vector3 j3_a;
	j3_a.x = current_accel.Actuator3_X;
	j3_a.y = current_accel.Actuator3_Y;
	j3_a.z = current_accel.Actuator3_Z;
	msg.values.push_back(j3_a);
	
	geometry_msgs::Vector3 j4_a;
	j4_a.x = current_accel.Actuator4_X;
	j4_a.y = current_accel.Actuator4_Y;
	j4_a.z = current_accel.Actuator4_Z;
	msg.values.push_back(j4_a);
	
	geometry_msgs::Vector3 j5_a;
	j5_a.x = current_accel.Actuator5_X;
	j5_a.y = current_accel.Actuator5_Y;
	j5_a.z = current_accel.Actuator5_Z;
	msg.values.push_back(j5_a);
	
	geometry_msgs::Vector3 j6_a;
	j6_a.x = current_accel.Actuator6_X;
	j6_a.y = current_accel.Actuator6_Y;
	j6_a.z = current_accel.Actuator6_Z;
	msg.values.push_back(j6_a);
	
	joint_accel_publisher_.publish(msg);
}

/*!
 * \brief Publishes the current joint efforts (gravity-free).
 *
 */
void JacoArm::publishJointEfforts(void)
{
    // Qurry efforts
    AngularPosition current_efforts_gfree;
    jaco_comm_.getEfforts(current_efforts_gfree);
	
	/*ROS_INFO("Efforts:\t%f,%f,%f,%f,%f,%f",
		current_efforts_gfree.Actuators.Actuator1,current_efforts_gfree.Actuators.Actuator2,current_efforts_gfree.Actuators.Actuator3,
		current_efforts_gfree.Actuators.Actuator4,current_efforts_gfree.Actuators.Actuator5,current_efforts_gfree.Actuators.Actuator6);*/

	//TO DO: publish these efforts
	
	sensor_msgs::JointState joint_state;
    const char* nameArgs[] = {"jaco_joint_1", "jaco_joint_2", "jaco_joint_3", "jaco_joint_4", "jaco_joint_5", "jaco_joint_6"};
    std::vector<std::string> joint_names(nameArgs, nameArgs + 6);
    joint_state.name = joint_names;
    
    joint_state.effort.resize(6);
    joint_state.effort[0] = current_efforts_gfree.Actuators.Actuator1;
    joint_state.effort[1] = current_efforts_gfree.Actuators.Actuator2;
    joint_state.effort[2] = current_efforts_gfree.Actuators.Actuator3;
    joint_state.effort[3] = current_efforts_gfree.Actuators.Actuator4;
    joint_state.effort[4] = current_efforts_gfree.Actuators.Actuator5;
    joint_state.effort[5] = current_efforts_gfree.Actuators.Actuator6;
    
    joint_effort_publisher_.publish(joint_state);
}


/*!
 * \brief Publishes the current joint angles.
 *
 * Joint angles are published in both their raw state as obtained from the arm
 * (JointAngles), and transformed & converted to radians (joint_state) as per
 * the Jaco Kinematics PDF.
 *
 * JointState will eventually also publish the velocity and effort for each
 * joint, when this data is made available by the C++ API.  Currenty velocity
 * and effort are reported as being zero (0.0) for all joints.
 */
void JacoArm::publishJointAngles(void)
{

	
	/*ROS_INFO("Efforts:\t%f,%f,%f,%f,%f,%f",
		current_efforts_gfree.Actuators.Actuator1,current_efforts_gfree.Actuators.Actuator2,current_efforts_gfree.Actuators.Actuator3,
		current_efforts_gfree.Actuators.Actuator4,current_efforts_gfree.Actuators.Actuator5,current_efforts_gfree.Actuators.Actuator6);*/

	//TO DO: publish these efforts

    
    // Query arm for current joint angles
    JacoAngles current_angles;
    jaco_comm_.getJointAngles(current_angles);
    jaco_msgs::JointAngles jaco_angles = current_angles.constructAnglesMsg();

    jaco_angles.joint1 = current_angles.Actuator1;
    jaco_angles.joint2 = current_angles.Actuator2;
    jaco_angles.joint3 = current_angles.Actuator3;
    jaco_angles.joint4 = current_angles.Actuator4;
    jaco_angles.joint5 = current_angles.Actuator5;
    jaco_angles.joint6 = current_angles.Actuator6;

    sensor_msgs::JointState joint_state;
    const char* nameArgs[] = {"jaco_joint_1", "jaco_joint_2", "jaco_joint_3", "jaco_joint_4", "jaco_joint_5", "jaco_joint_6"};
    std::vector<std::string> joint_names(nameArgs, nameArgs + 6);
    joint_state.name = joint_names;

    // Transform from Kinova DH algorithm to physical angles in radians, then place into vector array
    joint_state.position.resize(6);

    joint_state.position[0] = (180- jaco_angles.joint1) * (PI / 180);
    joint_state.position[1] = (jaco_angles.joint2 - 270) * (PI / 180);
    joint_state.position[2] = (90-jaco_angles.joint3) * (PI / 180);
    joint_state.position[3] = (180-jaco_angles.joint4) * (PI / 180);
    joint_state.position[4] = (180-jaco_angles.joint5) * (PI / 180);
    joint_state.position[5] = (260-jaco_angles.joint6) * (PI / 180);

    // TODO: Add joint velocity
    // joint_state.velocity.resize(6);

    // TODO: Place the arm actuator forces into the array
    // JacoForces arm_forces;
    // arm_.GetForcesInfo(arm_forces);
    // joint_state.effort.resize(6);
    // joint_state.effort[0] = arm_forces.Actuator1;
    // joint_state.effort[1] = arm_forces.Actuator2;
    // joint_state.effort[2] = arm_forces.Actuator3;
    // joint_state.effort[3] = arm_forces.Actuator4;
    // joint_state.effort[4] = arm_forces.Actuator5;
    // joint_state.effort[5] = arm_forces.Actuator6;

	//JacoForces arm_forces;
	//arm_.GetForcesInfo(arm_forces);

    joint_angles_publisher_.publish(jaco_angles);
    joint_state_publisher_.publish(joint_state);
}


/*!
 * \brief Publishes the current cartesian coordinates
 */
void JacoArm::publishToolPosition(void)
{
    JacoPose pose;
    geometry_msgs::PoseStamped current_position;

    jaco_comm_.getCartesianPosition(pose);
    current_position.pose = pose.constructPoseMsg();
   
    //set the frame_id (addition by Jivko)
    current_position.header.frame_id = "base_footprint";

	//publish tool pose as PoseStamped
    tool_position_publisher_.publish(current_position);
    
    //publish tool post as (x,y,z) and (ThetaX, ThetaY, ThetaZ)
    geometry_msgs::TwistStamped current_pos;
    current_pos.header = current_position.header;
    current_pos.twist.linear.x = pose.X;
    current_pos.twist.linear.y = pose.Y;
    current_pos.twist.linear.z = pose.Z;
    current_pos.twist.angular.x = pose.ThetaX;
    current_pos.twist.angular.y = pose.ThetaY;
    current_pos.twist.angular.z = pose.ThetaZ;
   
    cartesian_position_publisher_.publish(current_pos);
   
    
}


/*!
 * \brief Publishes the current finger positions.
 */
void JacoArm::publishFingerPosition(void)
{
    FingerAngles fingers;
    jaco_comm_.getFingerPositions(fingers);
    finger_position_publisher_.publish(fingers.constructFingersMsg());
}


void JacoArm::statusTimer(const ros::TimerEvent&)
{
    publishJointAngles();
    publishToolPosition();
    publishFingerPosition();
    publishJointEfforts();
    publishAcceleration();
}

}  // namespace jaco
