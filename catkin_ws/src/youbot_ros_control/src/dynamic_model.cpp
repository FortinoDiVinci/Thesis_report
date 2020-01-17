#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <brics_actuator/JointTorques.h>

#include "utils/utils.h"


/**********	
 * MACROS *
 **********/

#define NB_JOINT_YOUBOT     5
#define NB_ACTUATED_JOINTS  3
#define ALPHA               1

const bool ACTUATED_JOINTS[NB_JOINT_YOUBOT] = {false, true, true, true, false};
const float YB_JOINTS_OFFSET[NB_JOINT_YOUBOT] = 
{ 169*M_PI/180, 
   65*M_PI/180, 
 -146*M_PI/180, 
  102*M_PI/180, 
167.5*M_PI/180};

const double gear_ratio[NB_JOINT_YOUBOT] = {1./156, 1./156, 1./100, 1./71, 1./71};
const double torque_constant[NB_JOINT_YOUBOT] = {0.0335, 0.0335, 0.0335, 0.051, 0.049};
const double joint_torque_to_motor_current[NB_JOINT_YOUBOT] = 
{gear_ratio[0]/torque_constant[0],
 gear_ratio[1]/torque_constant[1],
 gear_ratio[2]/torque_constant[2],
 gear_ratio[3]/torque_constant[3],
 gear_ratio[4]/torque_constant[4]};

/**************
 * GLOBAL VAR *
 **************/
 
double thetas[NB_JOINT_YOUBOT] = {0,0,0,0,0};
double d_thetas[NB_JOINT_YOUBOT] = {0,0,0,0,0};
 
/*************
 * FUNCTIONS *
 *************/

void rosInit(ros::Publisher*, ros::Subscriber*, int, char**); 
brics_actuator::JointTorques initTorquesMsg(const bool*);
brics_actuator::JointTorques initCurrentMsg(const bool*);
void getJointStates(const sensor_msgs::JointState::ConstPtr& data);
std::vector <double> base_ccg_3dof();
void publishTorqueMsgs(ros::Publisher* pub, brics_actuator::JointTorques msg);
void compensateDynamicModel(ros::Publisher* pub, brics_actuator::JointTorques msg);

/********
 * MAIN *
 ********/

int main(int argc, char** argv)
{
    ros::Publisher pub_trq_msg; 
    ros::Subscriber sub_joint;
    brics_actuator::JointTorques trq_msg;
    
    rosInit(&pub_trq_msg, &sub_joint, argc, argv);
    //trq_msg = initTorquesMsg(ACTUATED_JOINTS);
    trq_msg = initCurrentMsg(ACTUATED_JOINTS);
    
    ros::Rate rate(1000);

    while (ros::ok())
    {
        //publishTorqueMsgs(&pub_trq_msg, trq_msg);
        compensateDynamicModel(&pub_trq_msg, trq_msg);
        
        ros::spinOnce();
        rate.sleep();
    }
}


void rosInit(ros::Publisher* pub, ros::Subscriber* sub, int argc, char** argv)
{

    ROS_INFO("youbot_dynamic_model node started\n");
    
    ros::init(argc, argv, "youbot_dynamic_model");
    ros::NodeHandle n;
    
    // Subscribers & Publishers
    
    std::string topic_name;

    topic_name = "/joint_states";
    *sub = n.subscribe(topic_name, 1, getJointStates);
    
    //topic_name = "arm_1/dynamic_model";
    topic_name = "/arm_1/arm_controller/torque_command";
    *pub = n.advertise<brics_actuator::JointTorques>(topic_name, 1);
    topic_name.clear();
    
    double frequency = 1000;
    
    ros::Rate rate(frequency);
}

brics_actuator::JointTorques initTorquesMsg(const bool joint_actuated[])
{
    brics_actuator::JointTorques trq_msg;
    trq_msg.torques.resize(NB_ACTUATED_JOINTS);
    int jnt = 0;

    for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++)
    {
        if (joint_actuated[ii] == true)
        {
            std::stringstream jointNameStream;
            jointNameStream << "arm_joint_" << ii + 1;        
            trq_msg.torques[jnt].joint_uri = jointNameStream.str();
            trq_msg.torques[jnt].unit = "N m";
            trq_msg.torques[jnt].value = 0.0;
            jnt++;
        }
        else continue;
    }
    return trq_msg;
}

brics_actuator::JointTorques initCurrentMsg(const bool joint_actuated[])
{
    brics_actuator::JointTorques trq_msg;
    trq_msg.torques.resize(NB_ACTUATED_JOINTS);
    int jnt = 0;

    for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++)
    {
        if (joint_actuated[ii] == true)
        {
            std::stringstream jointNameStream;
            jointNameStream << "arm_joint_" << ii + 1;        
            trq_msg.torques[jnt].joint_uri = jointNameStream.str();
            trq_msg.torques[jnt].unit = "A";
            trq_msg.torques[jnt].value = 0.0;
            jnt++;
        }
        else continue;
    }
    return trq_msg;
}

void getJointStates(const sensor_msgs::JointState::ConstPtr& data)
{
    for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++)
    {
        thetas[ii] = data->position[ii];
        d_thetas[ii] = data->velocity[ii];
    }
}

std::vector <double> base_ccg_3dof()
{
    double th1 = -thetas[1] + M_PI/2 + YB_JOINTS_OFFSET[1];
    double th2 = -thetas[2] + YB_JOINTS_OFFSET[2];
    double th3 = -thetas[3] + YB_JOINTS_OFFSET[3];  

    double QP1 = -d_thetas[1]*0.;
    double QP2 = -d_thetas[2]*0.;
    double QP3 = -d_thetas[3]*0.;

    double FX3 = 0;    
    double FY3 = 0;
    double CZ3 = 0;
    double MXR1 = 0.4124;
    double MXR2 = 0.2393;
    double MX3 = 0.0833;
    double FS1 = 1.0044;
    double FS2 = 0.5523;
    double FS3 = 0.2175;
    double FV1 = 1.2958;
    double FV2 = 0.6531;
    double FV3 = 0.1085;  

    double MY1 = 0;
    double MY2 = 0;
    double MY3 = 0;
    double GZ = -9.81;

    // Geometric parameters (m)
    double d3 = 0.155; // ok
    double d4 = 0.135; // ok

    // Equations:

    double C1 = cos(th1);
    double S1 = sin(th1);
    double C2 = cos(th2);
    double S2 = sin(th2);
    double C3 = cos(th3);
    double S3 = sin(th3);

    double DV61 = QP1*QP1;
    double VP11 = -GZ*S1;
    double VP21 = -C1*GZ;
    double W32 = QP1 + QP2;
    double DV62 = W32*W32;
    double VSP12 = -DV61*d3 + VP11;
    double VP12 = C2*VSP12 + S2*VP21;
    double VP22 = C2*VP21 - S2*VSP12;
    double W33 = QP3 + W32;
    double DV63 = W33*W33;
    double VSP13 = -DV62*d4 + VP12;
    double VP13 = C3*VSP13 + S3*VP22;
    double VP23 = C3*VP22 - S3*VSP13;

    double F12 = -DV62*MXR2;
    double F22 = -DV62*MY2;
    double F13 = -DV63*MX3;
    double F23 = -DV63*MY3;
    double E13 = F13 + FX3;
    double E23 = F23 + FY3;
    double N33 = CZ3 + MX3*VP23 - MY3*VP13;
    double FDI13 = C3*E13 - E23*S3;
    double FDI23 = C3*E23 + E13*S3;
    double E12 = F12 + FDI13;
    double E22 = F22 + FDI23;
    double N32 = FDI23*d4 + MXR2*VP22 - MY2*VP12 + N33;
    double FDI22 = C2*E22 + E12*S2;
    double N31 = FDI22*d3 + MXR1*VP21 - MY1*VP11 + N32;

    double GAM1 = 0.*FS1*sign(QP1) + 0.*FV1*QP1 + N31;
    double GAM2 = 0.*FS2*sign(QP2) + 0.*FV2*QP2 + N32;
    double GAM3 = 0.*FS3*sign(QP3) + 0.*FV3*QP3 + N33;
    
    std::vector <double> tau;    
    tau.push_back(-GAM1);
    tau.push_back(-GAM2);
    tau.push_back(-GAM3);
    
    return tau;
}

void publishTorqueMsgs(ros::Publisher* pub, brics_actuator::JointTorques msg)
{
    std::vector<double> tau = base_ccg_3dof();

    msg.torques[0].value = tau[0];
    msg.torques[1].value = tau[1];
    msg.torques[2].value = tau[2];
    
    pub->publish(msg);
}

void compensateDynamicModel(ros::Publisher* pub, brics_actuator::JointTorques msg)
{
    std::vector<double> tau = base_ccg_3dof();

    msg.torques[0].value = ( fabs(d_thetas[1]) < 0.8 ) ? tau[0]*joint_torque_to_motor_current[1]*ALPHA : 0;
    msg.torques[1].value = ( fabs(d_thetas[2]) < 0.8 ) ? tau[1]*joint_torque_to_motor_current[2]*ALPHA : 0;
    msg.torques[2].value = ( fabs(d_thetas[3]) < 0.8 ) ? tau[2]*joint_torque_to_motor_current[3]*ALPHA : 0;
    
    pub->publish(msg);
}
