#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <signal.h>
//#include <eigen3/Eigen/Dense>

#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointValue.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>

#include <boost/scoped_ptr.hpp>

//#include "youbot_ros_control/robot.h"
#include "robot.cpp"
#include "youbot_driver/generic/ConfigFile.hpp"

/**********
 * MACROS *
 **********/

#define WITH_VIRTUAL_MECH       false
// setting the following macro with nullspace ctrl will have no effect
#define WITH_Y_ORIENTATION      false
#define NULLSPACE_CTRL_LOOP     true
#define X_L_CTRL_LOOP           false
#define INV_JAC_CTRL_LOOP       false
#define NB_JOINT_YOUBOT     	5
#define NB_ACTUATED_JOINTS  	3
#define DOF                 	3
#define REF_FRAME_ID        	"base_link"

const float TH_MAX[NB_JOINT_YOUBOT] = {5.7401, 25179, -0.1157, 3.3292, 5.5415}; //rad
const float TH_MIN[NB_JOINT_YOUBOT] = {1.101e-1, 1.101e-1, -4.9266, 1.221e-1, 2.106e-1}; //rad
const float TH_ON_D[NB_JOINT_YOUBOT] = {169, 65, -146, 102-90, 167.5-110}; //degree
const float TH_OFF_D[NB_JOINT_YOUBOT] = {0.011, 0.011, -0.016, 0.023, 0.12}; //degree
const bool ACTUATED_JOINTS[NB_JOINT_YOUBOT] = {false, true, true, true, false};

/**************
 * GLOBAL VAR *
 **************/

Robot* kuka_youBot;
Pose force_torque_sensor(Point(0,0,0,"N"), Point(0,0,0,"N m"));
sig_atomic_t volatile g_request_shutdown = 0;

/*************
 * FUNCTIONS *
 *************/

brics_actuator::JointVelocities initVelocitiesCmd(const bool*);
brics_actuator::JointPositions initPositionsCmd();
void getForces(const geometry_msgs::WrenchStamped::ConstPtr& data);
void getJointStates(const sensor_msgs::JointState::ConstPtr& data);
void sigIntHandler(int sig);

/********
 * MAIN *
 ********/

int main(int argc, char** argv)
{
    
    //
    // ROS
    //
    
    ROS_INFO("start\n");
    
    ros::init(argc, argv, "control_loop", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::NodeHandle n1("~");
    signal(SIGINT, sigIntHandler);
    
    // Subscribers
    
    std::string topic_name;
    ros::Subscriber sub_force;
    ros::Subscriber sub_joint;

    // Config File in youbot driver package for youbot hw params
    
    boost::scoped_ptr<youbot::ConfigFile> config_file;
    std::string youbot_driver_path = ros::package::getPath("youbot_driver");
    config_file.reset(new youbot::ConfigFile("youbot-manipulator.cfg", youbot_driver_path + "/config"));
    
    // Parameter server : PID, rate, ...
    
    float Kp[NB_JOINT_YOUBOT];
    float Ki[NB_JOINT_YOUBOT];
    float frequency, Kx_vm, Bx_vm, x0, Kry_vm, Bry_vm, ry0, alpha, Kq;
    
    std::string tmp_str = "Kx0_gain";
    std::stringstream joint_pid_data;
    
    #if WITH_VIRTUAL_MECH
    alpha = 0.35; // lower PID gain for stability purpose
    #else
    alpha = 1.0;
    #endif
    
    for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++)
    {
        std::stringstream jointNameStream;
        jointNameStream << "" << ii + 1;
        tmp_str = "Kp" + jointNameStream.str() + "_gain";
        n1.getParam(tmp_str, Kp[ii]);
        
        tmp_str = "Ki" + jointNameStream.str() + "_gain";
        n1.getParam(tmp_str, Ki[ii]);
        
        joint_pid_data << "Kp: " << Kp[ii] << "\n" << "Ki: " << Ki[ii] << "\n";  
    }
    ROS_INFO_STREAM(joint_pid_data.str());
    tmp_str.clear();

    n1.getParam("rate", frequency);
    n1.getParam("Stiffness_x", Kx_vm);
    n1.getParam("Damping_x", Bx_vm);
    n1.getParam("Stiffness_ry", Kry_vm);
    n1.getParam("Damping_ry", Bry_vm);
    n1.getParam("Equilibrium_x", x0);
    n1.getParam("Equilibrium_ry", ry0);
    n1.getParam("Joint_equilibrium_gain", Kq);
    
    ROS_INFO_STREAM("freq: " << frequency << "\n" << "Stiffness x: " << Kx_vm << "\n"
    	<< "Damping x: " << Bx_vm << "\n" << "Stiffness ry: " << Kry_vm << "\n"
    	<< "Damping ry: " << Bry_vm << "\n");
    
    //
    // ROBOT & VIRTUAL FIXTURE
    //
    
    std::vector<Joint> joints;
    joints.reserve(NB_JOINT_YOUBOT);
    float tmp_max_vel;
    std::string joint_name;
    
    for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++)
    {
        PID joint_pid(Kp[ii]*alpha, Ki[ii]*alpha, 0.0);
        joints.push_back(Joint(TH_MAX[ii], TH_MIN[ii], joint_pid));
        
        std::stringstream jointNameStream;
        jointNameStream << "" << ii + 1;
        joint_name = "Joint_" + jointNameStream.str();
        config_file->readInto(tmp_max_vel, joint_name, "MaxVelocity");
        joints[ii].setMaxVelocity(tmp_max_vel);
    }
    
    Jacobian youBot_jacobian(DOF, NB_ACTUATED_JOINTS);
    kuka_youBot = new Robot(joints, ACTUATED_JOINTS, youBot_jacobian, initPositionsCmd(), &n);
    
    joint_name.clear();
    joints.clear();
    
    // nullspace control method requires endpoint limit definition, 
    // extra ctrl gain & joints prefered position
    #if NULLSPACE_CTRL_LOOP
    std::vector <Endpoint> ep_limits;
    std::vector <float> qi_0;
    float x_lim_min, z_lim_min, x_lim_max, z_lim_max;
    x_lim_min = -0.21;
    x_lim_max = -0.19;
    z_lim_min = 0.226;
    z_lim_max = 0.410;
    
    ep_limits.resize(2);
    qi_0.resize(3);
    
    ep_limits[0].setEndpointPose(Pose(Point(x_lim_min,0,z_lim_min), Point(0,0,0)));
    ep_limits[1].setEndpointPose(Pose(Point(x_lim_max,0,z_lim_max), Point(0,0,0)));
    
    qi_0[0] = 1.676;
    qi_0[1] = -4.363;
    qi_0[2] = 1.497;
    
    kuka_youBot->setNullspaceCtrlGains(Kx_vm, Kq);
    kuka_youBot->setEndpointLimits(ep_limits);
    kuka_youBot->setEndpointPID(PID(Kp[1], Ki[1], 0));
    #endif
    
    // Start listening to youBot msgs
    
    topic_name = "force_sensor/grav_comp";
    sub_force = n.subscribe(topic_name, 1, getForces);
    topic_name = "/joint_states";
    sub_joint = n.subscribe(topic_name, 1, getJointStates);
    topic_name.clear();
    
    // Virtual Mechanism init
    
    #if WITH_VIRTUAL_MECH
    
    float K_vm[6] = {Kx_vm, 0, 0, 0, Kry_vm, 0};
    float B_vm[6] = {Bx_vm, 0, 0, 0, Bry_vm, 0};
    float I_vm[6] = {0, 0, 0, 0, 0, 0};
    Endpoint equilibrium;
    equilibrium.setEndpointPose(Pose(Point(x0, 0, 0, "m"), Point(0, ry0, 0, "rad")));
    std::vector<Endpoint> limits_vm;
    for (int i = 0; i < 2; i++) {Endpoint tmp_ep; limits_vm.push_back(tmp_ep);}
    limits_vm[0].setEndpointPose(Pose(Point(0.05,0,0,"m"), Point()));
    limits_vm[1].setEndpointPose(Pose(Point(0.06,0,0,"m"), Point()));
    
    VirtualMechanism vm(K_vm, B_vm, I_vm, equilibrium, true, limits_vm);
      
    #endif
    
    Pose force_torque_vm(Point(0,0,0,"N"), Point(0,0,0,"N m"));
    
    // youBot init position
    
    float init_angle[NB_JOINT_YOUBOT] = {169, 65, -146, 102.5-90, 167.5-110}; // °
    float off_angle[NB_JOINT_YOUBOT] = {0.011, 0.011, -0.016, 0.023, 0.12}; // rad
    brics_actuator::JointPositions init_off_pos;
    init_off_pos.positions.resize(NB_JOINT_YOUBOT);
    
    for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++)
    {
        std::stringstream jointNameStream;
        jointNameStream << "" << ii + 1;
        joint_name = "arm_joint_" + jointNameStream.str();
        //init_off_pos.positions[ii].timeStamp = ros::Time::now();
        init_off_pos.positions[ii].joint_uri = joint_name;
        init_off_pos.positions[ii].unit = "rad";
        init_off_pos.positions[ii].value = init_angle[ii] * M_PI/180;
    }
    
    #if NULLSPACE_CTRL_LOOP
    // initial position must be in the workspace for this loop
    init_off_pos.positions[1].value = qi_0[0];
    init_off_pos.positions[2].value = qi_0[1];
    init_off_pos.positions[3].value = qi_0[2]; 
    #endif
    
    kuka_youBot->sendPositionCmd(init_off_pos);
    usleep(1.0*1e6); // this delay seems necessary..
    kuka_youBot->publishPositionsCmd();
    usleep(2.0*1e6); // wait 2 seconds for the end of the movement
    
    // Time & frequency
    
    float Te = 1/frequency;
    float delay = Te;           //the ideal is: delay = Te

    ros::Rate rate(frequency);
    kuka_youBot->updateTimeSample();
    
    ROS_INFO("Beginning of the main loop\n");
    
    /************
     *   LOOP   *
     ************/

    while (!g_request_shutdown)
    {
    	#if WITH_VIRTUAL_MECH || NULLSPACE_CTRL_LOOP
    	// kinematics
    	kuka_youBot->computeEnpointPosition();
    	#if WITH_Y_ORIENTATION  && !NULLSPACE_CTRL_LOOP
    	kuka_youBot->computeEnpointOrientation(false, true, false); // only get rotation about y   	
        // compute VM
        force_torque_vm = vm.verticalXLineFixture(kuka_youBot->getEndpoint().getPose().getPosition().x, kuka_youBot->getEndpoint().getPose().getOrientation().y, kuka_youBot->getEndpoint().getVelocities().getPosition().x, kuka_youBot->getEndpoint().getVelocities().getOrientation().y);
        #elif !NULLSPACE_CTRL_LOOP
        force_torque_vm = vm.verticalXLineFixture(kuka_youBot->getEndpoint().getPose().getPosition().x, kuka_youBot->getEndpoint().getVelocities().getPosition().x);
        #endif
        #endif
        // forces to joint torques
        #if X_L_CTRL_LOOP
        kuka_youBot->updateJacobianTranspose();
        kuka_youBot->setInputError(kuka_youBot->computeJointTorquesFromWrench(force_torque_sensor + force_torque_vm));
        kuka_youBot->computeVelocityCollaborativeCmd(); 
        #elif INV_JAC_CTRL_LOOP
        kuka_youBot->updateJacobianInverse();
        kuka_youBot->setInputError(kuka_youBot->computeJointVelocitiesFromEndpointVelocity());
        #elif NULLSPACE_CTRL_LOOP
        kuka_youBot->updateJacobianInverse();
        kuka_youBot->updateJacobianTranspose();
        kuka_youBot->computeEnpointPosition();
        kuka_youBot->computeNullspaceCollaborativeCmd(x0, force_torque_sensor.getPosition().z, qi_0);
        #endif
        // PI       
        kuka_youBot->publishVelocitiesCmd();

	//ROS_INFO_STREAM_THROTTLE(0.2, "VM forces:\n" << force_torque_vm.getPoseVector());

        ros::spinOnce();
        rate.sleep();
    }
    
    kuka_youBot->sendPositionCmd(init_off_pos);
    kuka_youBot->publishPositionsCmd();
}

/*************
 * FUNCTIONS *
 *************/

/* Initialize joint velocities msg to be send through ros topic,
 * only actuated joints are controlled by velocity msgs */

brics_actuator::JointVelocities initVelocitiesCmd(const bool joint_actuated[])
{
    brics_actuator::JointVelocities vel_cmd;
    vel_cmd.velocities.reserve(NB_JOINT_YOUBOT);
    brics_actuator::JointValue jnt_val;

    for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++)
    {
        if (joint_actuated[ii] == true)
        {
            std::stringstream jointNameStream;
            jointNameStream << "" << ii + 1;        
            jnt_val.joint_uri = "arm_joint_" + jointNameStream.str();
            jnt_val.unit = "s^-1 rad";
            jnt_val.value = 0.0;
            vel_cmd.velocities.push_back(jnt_val);
        }
        else continue;
    }
    return vel_cmd;
}

/* Initialize joint positions msg to be send through ros topic */

brics_actuator::JointPositions initPositionsCmd()
{
    brics_actuator::JointPositions pos_cmd;
    pos_cmd.positions.resize(NB_JOINT_YOUBOT);
    
    for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++)
    {
        std::stringstream jointNameStream;
        jointNameStream << "" << ii + 1;
        pos_cmd.positions[ii].joint_uri = "arm_joint_" + jointNameStream.str();
        pos_cmd.positions[ii].unit = "rad";
        pos_cmd.positions[ii].value = 0.0;
    }
    return pos_cmd;
}

void getForces(const geometry_msgs::WrenchStamped::ConstPtr& data)
{
    //header
    //TODO : check that headers are consistant
    if (data->header.frame_id != REF_FRAME_ID)
    {
        ROS_WARN_STREAM_THROTTLE(5, "Reference frame of the force torque msg seems wrong, " << REF_FRAME_ID << " is expected.");
    }
    // wrench
    force_torque_sensor.setPositionX(data->wrench.force.x);
    force_torque_sensor.setPositionY(data->wrench.force.y);
    force_torque_sensor.setPositionZ(data->wrench.force.z);
    force_torque_sensor.setOrientationX(data->wrench.torque.x);
    force_torque_sensor.setOrientationY(data->wrench.torque.y);
    force_torque_sensor.setOrientationZ(data->wrench.torque.z);
}

void getJointStates(const sensor_msgs::JointState::ConstPtr& data)
{
    for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++)
    {
        kuka_youBot->updateJointData(ii, data->position[ii], data->velocity[ii], data->effort[ii]);
    }
}

void sigIntHandler(int sig)
{
    g_request_shutdown = 1;
}
