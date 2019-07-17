/*****************
 *    INCLUDES   *
 *****************/

#include <math.h>
#include <unistd.h>
#include <vector>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <signal.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointValue.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>

#include "youbot_ros_control/jacobian.h"

/*****************
 *   FUNCTIONS   *
 *****************/
 
#define DEBUG 1
// for the jacobian function
#define A 0.228
#define B 0.135
#define C 0.155
#define NUMBER_ARM_JOINTS 5

geometry_msgs::WrenchStamped sensor_data;
float thetas[NUMBER_ARM_JOINTS] = {0.,0.,0.,0.,0.};
// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

void getJointStateCallback(const sensor_msgs::JointState::ConstPtr& joint) 
{
    for (int i = 0; i < NUMBER_ARM_JOINTS; i++)
    {
        thetas[i] = joint->position[i];
    }
}

void getForceCallback(const geometry_msgs::WrenchStamped::ConstPtr& data)
{
    //header
    sensor_data.header.seq = data->header.seq;
    sensor_data.header.stamp = data->header.stamp;
    sensor_data.header.frame_id = data->header.frame_id;
    //wrench
    sensor_data.wrench.force.x = data->wrench.force.x;
    sensor_data.wrench.force.y = data->wrench.force.y;
    sensor_data.wrench.force.z = data->wrench.force.z;
    sensor_data.wrench.torque.x = data->wrench.torque.x;
    sensor_data.wrench.torque.y = data->wrench.torque.y;
    sensor_data.wrench.torque.z = data->wrench.torque.z;
}

void copyWrenchData(const geometry_msgs::WrenchStamped data, const float jac_matrix[][6], float joints_torque[], int nb_joints, int fst_jnt)
{
    float ft_sensor_d[6] = {data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z};
    
    for (int i = 0; i < nb_joints; i++) {
        float temp = 0;
        for (int j = 0; j < 6; j++) {
            temp += jac_matrix[i][j]*ft_sensor_d[j]; 
        }
        joints_torque[fst_jnt + i] = temp;
    }
    //joints_torque[3] = -joints_torque[3]; // this axis is inverted
    //joints_torque[2] = -joints_torque[2]; // this axis is inverted
    //joints_torque[1] = -joints_torque[1]; // this axis is inverted
    joints_torque[0] = 0;
    joints_torque[4] = 0;
}
 
/******************
 *      MAIN      *
 ******************/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_loop", ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigIntHandler);
    ros::NodeHandle n;
    ros::NodeHandle n1("~");
    ros::Subscriber sub = n.subscribe("force_sensor/grav_comp", 1, getForceCallback);
    ros::Subscriber sub_js = n.subscribe("/joint_states", 1, getJointStateCallback);
    ros::Publisher pub = n.advertise<brics_actuator::JointVelocities>
        ("arm_1/arm_controller/velocity_command", 1);
    ros::Publisher pub_pos = n.advertise<brics_actuator::JointPositions>
        ("arm_1/arm_controller/position_command", 1);
    #if DEBUG == 1
    ros::Publisher pub_debug = n.advertise<std_msgs::Float32MultiArray>("debug/joint_torque_from_jacobian", 10);
    ros::Publisher pub_debug_j = n.advertise<std_msgs::Float32MultiArray>("debug/jacobian", 10);
    #endif

    float freq;
    float K[5];
    float Ki[5];
    float K_temp;
    float Ki_temp;
    bool single_axis;
    bool two_axis;
    bool three_axis;
    int joint_i;
    int nb_act_joints = NUMBER_ARM_JOINTS;
    
    n1.param<float>("rate", freq, 100.); 
    n1.param<float>("K_gain", K_temp, 100.); 
    n1.param<float>("Ki_gain", Ki_temp, 100.); 
    n1.param<bool>("Debug_single_axis", single_axis, true);
    n1.param<bool>("Two_axis", two_axis, false);
    n1.param<bool>("Three_axis", three_axis, false);
    n1.param<int>("Debug_sngl_axis_nb", joint_i, 4);
    joint_i--;
       
    float offset[NUMBER_ARM_JOINTS] = {169, 65, -146, 102.5-90, 167.5-110};
    float init_offset[NUMBER_ARM_JOINTS] = {0.011, 0.011, -0.016, 0.023, 0.12};
    
    if ((single_axis) && (joint_i > 4 || joint_i < 0)) {
        ROS_WARN("Single Axis Mode selected with a wrong joint nb (%i), joint 4 will be used", joint_i+1);
        joint_i = 3;
    }
    
    if(!single_axis) { joint_i = 0; }
    
    for (int i = 0; i < nb_act_joints; i++) 
    {
        K[i] = K_temp;
        Ki[i] = Ki_temp; 
    }
    Ki[1] = Ki_temp * 0.5;
    
    ros::Rate rate(freq);

    brics_actuator::JointVelocities velocities_cmd;
    brics_actuator::JointPositions positions_cmd;
    std::vector <brics_actuator::JointValue> vel;
    std::vector <brics_actuator::JointValue> pos;   
    pos.resize(NUMBER_ARM_JOINTS);
	
    if (single_axis) {
        nb_act_joints = 1;
    }
    else if (two_axis) {
        nb_act_joints = 2;
        joint_i = 2; // third joint
    }
    else if (three_axis) {
        nb_act_joints = 3;
        joint_i = 1; // second joint
    }    
    
    for (int i = 0; i < NUMBER_ARM_JOINTS; i++) offset[i] *= M_PI/180;
    // Set axis to required position
	for (int i = 0; i < NUMBER_ARM_JOINTS; i++) {
        std::stringstream jointNameStream;
        jointNameStream << "" << i + 1;
        pos[i].joint_uri = "arm_joint_" + jointNameStream.str();
        pos[i].unit = "rad";
        pos[i].value = offset[i];
    }
	
	vel.resize(nb_act_joints);
	
	for (int i = 0; i < nb_act_joints; i++) {
	    std::stringstream jointNameStream;
        jointNameStream << "" << joint_i + i + 1;
	    vel[i].joint_uri = "arm_joint_" + jointNameStream.str();
	    vel[i].unit = "s^-1 rad";
	    vel[i].value = 0;
	}

    velocities_cmd.velocities = vel;

    float joints_torque_feedback[NUMBER_ARM_JOINTS];
    float jacobian[nb_act_joints][6];
    float sum_err[5] = {0., 0., 0., 0., 0.};
    float Te = 1/freq;              // sampling time
    ros::Time t;
    ros::Time l_t = ros::Time::now();
    float delay = 0;

    #if DEBUG == 1
    std_msgs::Float32MultiArray joints_torque;
    std_msgs::Float32MultiArray jacobian_debug;
    for (int i = 0; i < NUMBER_ARM_JOINTS; i++) 
    {
        joints_torque.data.push_back(0.); // malloc ?
        jacobian_debug.data.push_back(0.0);
    }
    #endif
    positions_cmd.positions = pos;
    usleep(1.0*1e6); // this delay seems necessary..
    pub_pos.publish(positions_cmd);
    usleep(2.0*1e6); // wait 2 seconds for the end of the movement

    /************
     *   LOOP   *
     ************/
     
    while (!g_request_shutdown) {
    
        if (single_axis)
        {
            youBotJacobianTJoint4(thetas[joint_i], jacobian);
            copyWrenchData(sensor_data, jacobian, joints_torque_feedback, nb_act_joints, joint_i);
        }
        else if (two_axis)
        {
            youBotJacobianTJoints34(thetas, jacobian);
            copyWrenchData(sensor_data, jacobian, joints_torque_feedback, nb_act_joints, joint_i);
        }
        else if (three_axis)
        {
            youBotJacobianTJoints234(thetas, jacobian);
            copyWrenchData(sensor_data, jacobian, joints_torque_feedback, nb_act_joints, joint_i);
        }
        else
        {
            youBotJacobianT(thetas, jacobian);
            copyWrenchData(sensor_data, jacobian, joints_torque_feedback, nb_act_joints, 0);
        }

        #if DEBUG
        for (int i = 0; i < NUMBER_ARM_JOINTS; i++) {
            joints_torque.data[i] = joints_torque_feedback[i];
            jacobian_debug.data[i] = jacobian[0][i];
        }
        pub_debug.publish(joints_torque);
        pub_debug_j.publish(jacobian_debug);
        #endif
        
        delay = (t - l_t).toSec();
        l_t = t;
        if ( (delay - Te) > 0.01*Te) {
            ROS_WARN("Sampling time overpassed: %.5f, should be %.5f", delay, Te);
        }
        
        if (single_axis) {
            sum_err[joint_i] += joints_torque_feedback[joint_i];
            velocities_cmd.velocities[0].value = K[joint_i]*joints_torque_feedback[joint_i] + Ki[joint_i]*delay*sum_err[joint_i];
        }
        else {
            for (int i = 0; i < nb_act_joints; i++) {
                sum_err[i] += joints_torque_feedback[joint_i + i];
                velocities_cmd.velocities[i].value = K[i]*joints_torque_feedback[joint_i + i] + Ki[i]*delay*sum_err[i];
            }
        }

        t = ros::Time::now();
        for(int i = 0; i < nb_act_joints; i++){
            velocities_cmd.velocities[i].timeStamp = t;
        }

        pub.publish(velocities_cmd);
        ros::spinOnce();
        rate.sleep();
    }
    
    // Set axis to required position
	for (int i = 0; i < NUMBER_ARM_JOINTS; i++) {
        pos[i].value = init_offset[i];
    }
    positions_cmd.positions = pos;
    pub_pos.publish(positions_cmd);

    ros::shutdown();
}
