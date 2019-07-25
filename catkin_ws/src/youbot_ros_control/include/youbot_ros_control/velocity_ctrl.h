#ifndef VELOCITY_H_
#define VELOCITY_H_

#include <iostream>
#include <ros/ros.h>
#include <signal.h>
#include <math.h>

#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointTorques.h>
#include <brics_actuator/JointValue.h>
#include <sensor_msgs/JointState.h>
#include <boost/units/systems/si/io.hpp>

#include <youbot_driver/generic/ConfigFile.hpp>

#define NUMBER_ARM_JOINTS 5

class SpeedController
{
    public:
    ros::Publisher torque_cmd_pub;
    
    SpeedController(int dof, int fst_jnt, ros::NodeHandle n);
	
    bool isRampOn(int i) {return m_ramp_on[i];};
    void setRampOn(bool ramp_on, int i) {m_ramp_on[i] = ramp_on;};	
    float getKvGain(int i) {return m_Kv[i];};
    void setKvGain(float kv, int i) {m_Kv[i] = kv;};
    float getClipVmax(int i) {return m_clip_vmax[i];};
    void setClipVmax(float clip_vmax, int i) {m_clip_vmax[i] = clip_vmax;};
    float getAccmax(int i) {return m_acceleration_max[i];};
    void setAccmax(float acc_max, int i) {m_acceleration_max[i] = acc_max / (m_gear_ratio[i]*2.0*M_PI) *60.0;};
	
    brics_actuator::JointTorques getJointTorquesMsg() {return m_joint_trq_msg;};
	
    //getClipImax(int i) {return m_clip_imax[i];};
    //setClipImax(float clip_imax, int i) {m_clip_imax[i] = clip_imax;};
    void proportionalController(const int dof, const int fst_jnt);	
    void overSampling(int dof, int fst_jnt);
    void checkJointsLimit(int dof, int fst_jnt);
	
    private:
	// topic subscriber
    ros::Subscriber speed_cmd_sub;
    ros::Subscriber joint_state_sub;
    // debug
    ros::Publisher spld_speed_cmd_pub;
    ros::Publisher ramp_speed_cmd_pub;
	// callback functions
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void jointVelocityCmdCallback(const brics_actuator::JointVelocities::ConstPtr& msg);
    // 
	brics_actuator::JointTorques initializeJointTorqueMsg(int dof, int fst_jnt);
	// Velocities setpoints are meant to be sent at an lower rate
    // typicaly 100Hz, while this loop should run at a rate 10 times greater
    void generateRamp(const int i);
    bool IsJointLimitCritical(int i); // if joint limit is going to be reached
    int IsJointLimit(int i); // if joint limit has been reached (and specify min, or max lim)

    sensor_msgs::JointState m_joint_state; // joints position, velocity and effort
    brics_actuator::JointTorques m_joint_trq_msg; // command signal generated (ampere) 
    brics_actuator::JointVelocities m_joint_vel_cmd; // velocity set point (input)
    brics_actuator::JointVelocities m_joint_vel_cmd_spld; // over sampled signal (typ 1kHz)
    brics_actuator::JointVelocities m_joint_vel_cmd_ramp; // if ramp is used, n-1 setpoint
	
    ros::Time last_t[NUMBER_ARM_JOINTS];	
    bool m_ramp_on[NUMBER_ARM_JOINTS];
    float m_Kv[NUMBER_ARM_JOINTS]; // proportional gains
    float m_clip_vmax[NUMBER_ARM_JOINTS];
    //float m_clip_imax[NUMBER_ARM_JOINTS];
    float m_acceleration_max[NUMBER_ARM_JOINTS];
    float m_gear_ratio[NUMBER_ARM_JOINTS];
	
	const float m_joint_limit_angle[2][NUMBER_ARM_JOINTS] = {
	{0.0100692, 0.0100692, -5.02655, 0.0221239, 0.110619},
	{5.84014, 2.61799, -0.015708, 3.4292, 5.64159}};
};

#endif /* VELOCITY_H_ */
