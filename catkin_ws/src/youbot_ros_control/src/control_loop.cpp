#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <signal.h>
#include <time.h>
//#include <eigen3/Eigen/Dense>

#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointValue.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
//#include "youbot_ros_control/StampedBool.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <boost/scoped_ptr.hpp>

//#include "youbot_ros_control/robot.h"
#include "robot.cpp"
#include "youbot_driver/generic/ConfigFile.hpp"
#include "utils/utils.h"

/**********    
 * MACROS *
 **********/

#define BALL_IMPACT_FORCE       true
#define RANDOM_PERTURBATIONS    true
// CTRL MODE
#define WITH_VIRTUAL_MECH       false
// setting the following macro with nullspace ctrl will have no effect
#define WITH_Y_ORIENTATION      false
#define NULLSPACE_CTRL_LOOP     true
#define X_L_CTRL_LOOP           false
#define NB_JOINT_YOUBOT         5
#define NB_ACTUATED_JOINTS      3
#define DOF                     3
#define REF_FRAME_ID            "base_link"

const float TH_MAX[NB_JOINT_YOUBOT] = {5.7401, 25179, -0.1157, 3.3292, 5.5415}; //rad
const float TH_MIN[NB_JOINT_YOUBOT] = {1.101e-1, 1.101e-1, -4.9266, 1.221e-1, 2.106e-1}; //rad
const float TH_ON_D[NB_JOINT_YOUBOT] = {169, 65, -146, 102-90, 167.5-110}; //degree
const float TH_OFF_D[NB_JOINT_YOUBOT] = {0.011, 0.011, -0.016, 0.023, 0.12}; //degree
const bool ACTUATED_JOINTS[NB_JOINT_YOUBOT] = {false, true, true, true, false};

/**************
 * GLOBAL VAR *
 **************/

Robot* kuka_youBot;
float joint_effort_set_point[NB_JOINT_YOUBOT] = {0., 0., 0., 0., 0.};
float joint_velocity_set_point[NB_JOINT_YOUBOT] = {0., 0., 0., 0., 0.};
Pose force_torque_sensor(Point(0,0,0,"N"), Point(0,0,0,"N m"));
sig_atomic_t volatile g_request_shutdown = 0;
bool unlocked = false;
bool perturbation_lock = false;

/*************
 *  CLASSES  *
 *************/

class DisturbanceTimer
{

public:
    
    DisturbanceTimer(ros::NodeHandle*, float);
    
    // timers for perturbation introduction
    void trigger(const ros::TimerEvent&);
    void stop_curr(const ros::TimerEvent&);
    void stop_vel(const ros::TimerEvent&);
    void off(const ros::TimerEvent&);
    void unlockDist(const ros::TimerEvent&);
    
    // timer for ball/paddle impact
    void stopImpact(const ros::TimerEvent&);
    
    float getDisturbance() {return dist;};
    float getImpactMagnitude() {return force_impulse;};
    bool isDisturbance() {return dist_msg.data;};
    bool isTransitionning() {return torque_speed_transition;};
    bool isImpact() {return is_impact;};
    
    std::vector<float> getLastEffCmd() {return last_eff_cmd;};
    
    void getImpulse(const std_msgs::Float64::ConstPtr& data);
    void computePaddleFreq();    
    
private:
    float dist_magnitude;
    float dist;
    std::vector<float> last_eff_cmd;
    ros::NodeHandle* nh;
    
    ros::Timer stop_dist_timer;
    ros::Timer next_dist_timer;

    ros::Timer stop_impact_timer;
    ros::Timer next_impact_timer;
    
    //youbot_ros_control::StampedBool dist_msg;
    bool torque_speed_transition;
    bool is_impact = false;
    bool disturbance_unlocked = true;

    double force_impulse = 0;   // ball/paddle impact force
    float paddle_period;
    std::vector<float> lastImpactsTime;
    float cycle_delay = 0;      // given as pourcentage of the time period
                                // this delay starts with the ball impact
    
    std_msgs::Bool dist_msg;
    std_msgs::Float32 dist_val_msg;
    
    ros::Publisher pub_time_dist;
    ros::Publisher pub_val_dist;
    
    ros::Subscriber sub_impulse;
};

/*************
 * FUNCTIONS *
 *************/

std::vector <float> nullSpaceCtrlInit(Robot* youBot, float, float, float, float);
brics_actuator::JointPositions youBotInitializePosition(Robot*, std::vector<float>);
brics_actuator::JointPositions youBotInitializePosition(Robot*);

void youBotInitializationNSCtrl(Robot* &, boost::scoped_ptr<youbot::ConfigFile>&, ros::NodeHandle*);
void youBotInitializationXLCtrl(Robot*, boost::scoped_ptr<youbot::ConfigFile>&, float*, float*, ros::NodeHandle*);
void nullSpaceCtrlLoop(Robot*, Pose, float, std::vector<float>, float);
void velocityRamp(Robot*, int);
void xavierLamyCtrlLoop(Robot*, Pose, Pose);
Pose virtualLineGuide(Robot*, VirtualMechanism, bool);

brics_actuator::JointVelocities initVelocitiesCmd(const bool*);
brics_actuator::JointPositions initPositionsCmd();

void getForces(const geometry_msgs::WrenchStamped::ConstPtr& data);
void getJointStates(const sensor_msgs::JointState::ConstPtr& data);
void getJointSetpoints(const sensor_msgs::JointState::ConstPtr& data);

bool checkXLimits(const Robot youBot, const float max_x, const float min_x, const float max_z, const float min_z);
void safeStop(Robot* youBot);
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
    ros::Subscriber sub_joint_set_point;

    // Config File in youbot driver package for youbot hw params
    
    boost::scoped_ptr<youbot::ConfigFile> config_file;
    std::string youbot_driver_path = ros::package::getPath("youbot_driver");
    config_file.reset(new youbot::ConfigFile("youbot-manipulator.cfg", youbot_driver_path + "/config"));
    
    // Parameter server : PID, rate, ...
    
    float Kp[NB_JOINT_YOUBOT];
    float Ki[NB_JOINT_YOUBOT];
    float frequency, Kx_vm, Bx_vm, x0, Kry_vm, Bry_vm, ry0, Kq;
    
    std::string tmp_str = "Kx0_gain";
    std::stringstream joint_pid_data;
    
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
        << "Damping ry: " << Bry_vm << "\n" <<  "Joint eq gain: " << Kq << "\n");
    
    float dist_magnitude;
    n1.getParam("disturbance_magnitude", dist_magnitude);
    
    //
    // ROBOT & VIRTUAL FIXTURE
    //

    #if X_L_CTRL_LOOP
    
        youBotInitializationXLCtrl(kuka_youBot, config_file, Kp, Ki, &n);
        
    #elif NULLSPACE_CTRL_LOOP
        youBotInitializationNSCtrl(kuka_youBot, config_file, &n);
        std::vector<float> qi_0;
        qi_0 = nullSpaceCtrlInit(kuka_youBot, Kx_vm, Kq, Kp[1], Ki[1]);
    #endif
    
    // Start listening to youBot msgs
    
    topic_name = "force_sensor/grav_comp";
    sub_force = n.subscribe(topic_name, 1, getForces);
    topic_name = "/joint_states";
    sub_joint = n.subscribe(topic_name, 1, getJointStates);
    topic_name = "/arm_1/joint_set_points";
    sub_joint_set_point = n.subscribe(topic_name, 1, getJointSetpoints);
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
    
    // youBot position initialization
    
    brics_actuator::JointPositions init_off_pos;
    
    #if NULLSPACE_CTRL_LOOP

        // initial position must be in the workspace for this loop
        init_off_pos = youBotInitializePosition(kuka_youBot, qi_0);
        
    #elif X_L_CTRL_LOOP
    
        init_off_pos = youBotInitializePosition(kuka_youBot);
        
    #endif
    
    // safety
    
    bool stop_robot = false;
    
    // Time & frequency
    
    float Te = 1/frequency;
    float delay = Te;           //the ideal is: delay = Te
    int it = 0;                 //used for ramp transition between trq & vel ctrl
    
    DisturbanceTimer dist_timer(&n, dist_magnitude);
    //ros::Timer timer = n.createTimer(ros::Duration(5.), &DisturbanceTimer::trigger, &dist_timer, true);

    ros::Rate rate(frequency);
    kuka_youBot->updateTimeSample();
    kuka_youBot->computeEnpointPosition();
    
    ROS_INFO("Beginning of the main loop\n");
    
    /************
     *   LOOP   *
     ************/

    while (!g_request_shutdown)
    {
        
        #if WITH_VIRTUAL_MECH
            force_torque_vm = virtualLineGuide(kuka_youBot, vm, WITH_Y_ORIENTATION);
        #endif
        
        #if X_L_CTRL_LOOP
            xavierLamyCtrlLoop(kuka_youBot, force_torque_sensor, force_torque_vm);
        #elif NULLSPACE_CTRL_LOOP
        
        if( dist_timer.isImpact() )  // ball impact
        {
        #if BALL_IMPACT_FORCE
            kuka_youBot->setTorqueDisturbanceCmd(dist_timer.getLastEffCmd(), dist_timer.getImpactMagnitude());
            kuka_youBot->publishTorquesCmd();
        #endif    
        }
        else if( !dist_timer.isDisturbance() ) // no disturbance
        {
            // disturbance is set to 0 (it will be set in the inner loop)
            nullSpaceCtrlLoop(kuka_youBot, force_torque_sensor, x0, qi_0, 0); //dist_timer.getDisturbance()
            #if RANDOM_PERTURBATIONS
            if ( dist_timer.isTransitionning() )
            {
                // avoid spike after returning to velocity control
                velocityRamp(kuka_youBot, it);
                it = min(it+1, 15);
                //ROS_INFO_STREAM("iteration: " << it);
            }
            #endif
            kuka_youBot->publishVelocitiesCmd();
        }
        else // disturbance 
        {
        #if RANDOM_PERTURBATIONS
            it = 0;
            kuka_youBot->setTorqueDisturbanceCmd(dist_timer.getLastEffCmd(), dist_timer.getDisturbance());
            kuka_youBot->publishTorquesCmd();
        #endif
        }
        
    #endif
    
        //ROS_INFO_STREAM_THROTTLE(0.2, "VM forces:\n" << force_torque_vm.getPoseVector());
        
        if ( checkXLimits(*kuka_youBot, x0 + 0.03, x0 - 0.03, 0.410 + 0.05, 0.226 - 0.05) && unlocked )
        {
            stop_robot = true;
            ROS_ERROR("The robot endpoint is out of bounds, the experiment has been terminated.");
            break;
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    if ( stop_robot )
    {
        safeStop(kuka_youBot);
    }
    
    kuka_youBot->setPositionCmd(init_off_pos);
    kuka_youBot->publishPositionsCmd();
}

/*************
 * FUNCTIONS *
 *************/

// Init functions

void youBotInitializationXLCtrl(Robot* youBot, boost::scoped_ptr<youbot::ConfigFile>& cfg_file, float* Kp, float* Ki, ros::NodeHandle* nh)
{
    std::vector<Joint> joints;
    joints.reserve(NB_JOINT_YOUBOT);
    float tmp_max_vel;
    std::string joint_name;
    float alpha;
    #if WITH_VIRTUAL_MECH
        alpha = 0.35; // lower PID gain for stability purpose
    #else
        alpha = 1.0;
    #endif
    
    for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++)
    {
        PID joint_pid(Kp[ii]*alpha, Ki[ii]*alpha, 0.0);
        joints.push_back(Joint(TH_MAX[ii], TH_MIN[ii], joint_pid));
        
        std::stringstream jointNameStream;
        jointNameStream << "" << ii + 1;
        joint_name = "Joint_" + jointNameStream.str();
        cfg_file->readInto(tmp_max_vel, joint_name, "MaxVelocity");
        joints[ii].setMaxVelocity(tmp_max_vel);
    }
    
    Jacobian youBot_jacobian(DOF, NB_ACTUATED_JOINTS);
    youBot = new Robot(joints, ACTUATED_JOINTS, youBot_jacobian, initPositionsCmd(), nh);
}

void youBotInitializationNSCtrl(Robot* &youBot, boost::scoped_ptr<youbot::ConfigFile>& cfg_file, ros::NodeHandle* nh)
{
    std::vector<Joint> joints;
    joints.reserve(NB_JOINT_YOUBOT);
    float tmp_max_vel;
    std::string joint_name;
    
    for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++)
    {
        joints.push_back(Joint(TH_MAX[ii], TH_MIN[ii], PID()));
        
        std::stringstream jointNameStream;
        jointNameStream << "" << ii + 1;
        joint_name = "Joint_" + jointNameStream.str();
        cfg_file->readInto(tmp_max_vel, joint_name, "MaxVelocity");
        joints[ii].setMaxVelocity(tmp_max_vel);
    }
    
    Jacobian youBot_jacobian(DOF, NB_ACTUATED_JOINTS);
    youBot = new Robot(joints, ACTUATED_JOINTS, youBot_jacobian, initPositionsCmd(), nh);
}

std::vector <float> nullSpaceCtrlInit(Robot* youBot, float Kx_gain, float Kq_gain, float Kz_gain, float Kiz_gain)
{
    std::vector <Endpoint> ep_limits;
    std::vector <float> qi_0;
    float x_lim_min, z_lim_min, x_lim_max, z_lim_max;
    
    // endpoint limits
    x_lim_min = -0.21;
    x_lim_max = -0.19;
    z_lim_min = 0.226;
    z_lim_max = 0.410;
    
    ep_limits.resize(2);
    qi_0.resize(3);
    
    ep_limits[0].setEndpointPose(Pose(Point(x_lim_min,0,z_lim_min), Point(0,0,0)));
    ep_limits[1].setEndpointPose(Pose(Point(x_lim_max,0,z_lim_max), Point(0,0,0)));
    
    // joints equilibrium angles
    qi_0[0] = 1.676;
    qi_0[1] = -4.363;
    qi_0[2] = 1.497;
    
    youBot->setNullspaceCtrlGains(Kx_gain, Kq_gain);
    youBot->setEndpointLimits(ep_limits);
    youBot->setEndpointPID(PID(Kz_gain, Kiz_gain, 0));
    
    return qi_0;
}

brics_actuator::JointPositions youBotInitializePosition(Robot* youBot, std::vector<float> qi_0)
{
    brics_actuator::JointPositions init_off_pos;
    init_off_pos.positions.resize(NB_JOINT_YOUBOT);
    
    std::string joint_name;
    
    for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++)
    {
        std::stringstream jointNameStream;
        jointNameStream << "" << ii + 1;
        joint_name = "arm_joint_" + jointNameStream.str();
        //init_off_pos.positions[ii].timeStamp = ros::Time::now();
        init_off_pos.positions[ii].joint_uri = joint_name;
        init_off_pos.positions[ii].unit = "rad";
        init_off_pos.positions[ii].value = TH_ON_D[ii] * M_PI/180;
    }
   
    // initial position must be in the workspace for this ctrl method
    init_off_pos.positions[1].value = qi_0[0];
    init_off_pos.positions[2].value = qi_0[1];
    init_off_pos.positions[3].value = qi_0[2]; 
    
    youBot->setPositionCmd(init_off_pos);
    usleep(1.0*1e6); // this delay seems necessary..
    youBot->publishPositionsCmd();
    usleep(3.0*1e6); // waits 3 seconds for the end of the movement
    
    return init_off_pos;
}

brics_actuator::JointPositions youBotInitializePosition(Robot* youBot)
{
    brics_actuator::JointPositions init_off_pos;
    init_off_pos.positions.resize(NB_JOINT_YOUBOT);
    
    std::string joint_name;
    
    for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++)
    {
        std::stringstream jointNameStream;
        jointNameStream << "" << ii + 1;
        joint_name = "arm_joint_" + jointNameStream.str();
        //init_off_pos.positions[ii].timeStamp = ros::Time::now();
        init_off_pos.positions[ii].joint_uri = joint_name;
        init_off_pos.positions[ii].unit = "rad";
        init_off_pos.positions[ii].value = TH_ON_D[ii] * M_PI/180;
    }
    
    youBot->setPositionCmd(init_off_pos);
    usleep(1.0*1e6); // this delay seems necessary..
    youBot->publishPositionsCmd();
    usleep(3.0*1e6); // waits 3 seconds for the end of the movement
    return init_off_pos;
}

// main functions

void nullSpaceCtrlLoop(Robot* youBot, Pose ft_sens, float x_eq, std::vector<float> qi_eq, float dist)
{
    youBot->updateJacobianInverse();
    youBot->updateJacobianTranspose();
    youBot->computeEnpointPosition();
    // to allow perturbation/disturbance replace 0. by dist
    if (unlocked) {
        youBot->computeNullspaceCollaborativeCmd(x_eq, ft_sens.getPosition().z, dist, qi_eq);
    }
}

void xavierLamyCtrlLoop(Robot* youBot, Pose ft_sens, Pose ft_virt_guide)
{
    youBot->updateJacobianTranspose();
    youBot->setInputError(youBot->computeJointTorquesFromWrench(ft_sens + ft_virt_guide));
    youBot->computeVelocityCollaborativeCmd(); 
}

void velocityRamp(Robot* youBot, int iteration)
{
    brics_actuator::JointVelocities dummy_vel_cmd;
    dummy_vel_cmd = youBot->getVelocityCmd();
    //ROS_INFO_STREAM("Before: "<< dummy_vel_cmd.velocities[2].value);
    int idx = 0;
    float alpha = iteration/15;
    for (int i = 0; i < NB_JOINT_YOUBOT; i++)
    {
        if ( ACTUATED_JOINTS[i] )
        {
            dummy_vel_cmd.velocities[idx].value = (dummy_vel_cmd.velocities[idx].value*alpha + youBot->getJoints()[i].getAngularVelocity()*(1 - alpha))/2 ;
            idx++;
        }
    }    
    //ROS_INFO_STREAM("After: "<< dummy_vel_cmd.velocities[2].value);
    youBot->setVelocityCmd(dummy_vel_cmd);
}

Pose virtualLineGuide(Robot* youBot, VirtualMechanism vm, bool orientation) 
{
    Pose ft_guide(Point(0,0,0,"N"), Point(0,0,0,"N m"));
    kuka_youBot->computeEnpointPosition();
    
    if (orientation)
    {
    youBot->computeEnpointOrientation(false, true, false);
    ft_guide = vm.verticalXLineFixture(youBot->getEndpoint().getPose().getPosition().x, youBot->getEndpoint().getPose().getOrientation().y, youBot->getEndpoint().getVelocities().getPosition().x, youBot->getEndpoint().getVelocities().getOrientation().y);
    }
    else
    {
    ft_guide = vm.verticalXLineFixture(youBot->getEndpoint().getPose().getPosition().x, youBot->getEndpoint().getVelocities().getPosition().x);
    }
    
    return ft_guide;
}

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

// Callback functions

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
    unlocked = true;
}

void getJointSetpoints(const sensor_msgs::JointState::ConstPtr& data)
{
    for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++)
    {
        joint_effort_set_point[ii] = data->effort[ii];
        joint_velocity_set_point[ii] = data->velocity[ii];
    }
}

// security

bool checkXLimits(const Robot youBot, const float max_x, const float min_x, const float max_z, const float min_z)
{
    float youbot_x = youBot.getEndpoint().getPose().getPosition().x;
    float youbot_z = youBot.getEndpoint().getPose().getPosition().z;
    
    if( (youbot_x > max_x) || (youbot_x < min_x) ) return true;
    
    else if ( (youbot_z > max_z) || (youbot_z < min_z) ) return true;
    
    return false;
}

// stop the robot to its current position

void safeStop(Robot* youBot)
{
    std::vector<Joint> joints = youBot->getJoints();

    if (NB_JOINT_YOUBOT != joints.size())
    {
        ROS_ERROR("The number of joints was not properly defined, safe stop cannot happen !");
        return;
    }
    
    brics_actuator::JointPositions pos_cmd;
    pos_cmd.positions.resize(NB_JOINT_YOUBOT);
    
    for (int i = 0; i < NB_JOINT_YOUBOT; i++)
    {
        std::stringstream jointNameStream;
        jointNameStream << "" << i + 1;
        pos_cmd.positions[i].joint_uri = "arm_joint_" + jointNameStream.str();
        pos_cmd.positions[i].unit = "rad";
        pos_cmd.positions[i].value = joints[i].getAngle();
    }
    
    youBot->setPositionCmd(pos_cmd);
    youBot->publishPositionsCmd();
    
    // wait for keyboard to return to initial position
    
    ROS_INFO("When ready, push Enter key while on the terminal, to let the robot return to its initial position");
    
    std::string s;
    while ( getline( std::cin, s ) && !s.empty() ) { }    
    
}

//

void sigIntHandler(int sig)
{
    g_request_shutdown = 1;
}

// timers

DisturbanceTimer::DisturbanceTimer(ros::NodeHandle* n, float magnitude)
{
    this->nh = n;
    
    dist_magnitude = magnitude;
    dist = 0.0;
    dist_msg.data = false;
    torque_speed_transition = false;
    cycle_delay = 0.25; // should be in the middle of decreasing phase
    
    last_eff_cmd.resize(NB_ACTUATED_JOINTS);
    lastImpactsTime.resize(5, ros::Time::now().toSec()); // 5 previous impact times are stored
    
    std::string topic_name = "";
    //topic_name = "arm_1/disturbance_time";
    //pub_time_dist = nh->advertise<youbot_ros_control::StampedBool>(topic_name, 1);
    //pub_time_dist = nh->advertise<std_msgs::Bool>(topic_name, 1);
    topic_name = "arm_1/disturbance_val";
    pub_val_dist = nh->advertise<std_msgs::Float32>(topic_name, 1);
    topic_name = "impulse";
    sub_impulse = nh->subscribe(topic_name, 1, &DisturbanceTimer::getImpulse, this); // ball impact force
    
    srand (time(NULL));
}

void DisturbanceTimer::trigger(const ros::TimerEvent& e)
{
    //ROS_INFO("TRIGGERED");
    
    int idx = 0;
    for (int i = 0; i < NB_JOINT_YOUBOT; i++)
    {
        if ( ACTUATED_JOINTS[i] )
        {
            last_eff_cmd[idx] = joint_effort_set_point[i];
            idx++;
        }
    }

    dist_msg.data = true;
    dist = dist_magnitude*( 2*( rand()%2 ) - 1 ); // sign is chosen randomly
    dist_val_msg.data = dist;
    //dist_msg.stamp = ros::Time::now();
    //pub_time_dist.publish(dist_msg);
    pub_val_dist.publish(dist_val_msg);
    
    stop_dist_timer = nh->createTimer(ros::Duration(0.03), &DisturbanceTimer::stop_curr, this, true);
}

void DisturbanceTimer::stop_curr(const ros::TimerEvent& e)
{
    //ROS_INFO("STOPED");
    dist_msg.data = true;
    dist = 0.;
    dist_val_msg.data = dist;
    //dist_msg.stamp = ros::Time::now();
    //pub_time_dist.publish(dist_msg);
    pub_val_dist.publish(dist_val_msg);
    
    next_dist_timer = nh->createTimer(ros::Duration(0.0015), &DisturbanceTimer::stop_vel, this, true);
}

void DisturbanceTimer::stop_vel(const ros::TimerEvent& e)
{
    //ROS_INFO("STOPED");
    dist_msg.data = false;
    torque_speed_transition = true;
  
    next_dist_timer = nh->createTimer(ros::Duration(0.015), &DisturbanceTimer::off, this, true);
}

void DisturbanceTimer::off(const ros::TimerEvent& e)
{
    dist_msg.data = false;
    torque_speed_transition = false;
    //dist_msg.stamp = ros::Time::now();
    //pub_time_dist.publish(dist_msg);
    
    disturbance_unlocked = false;
    
    next_dist_timer = nh->createTimer(ros::Duration((rand()%301)/100+1.5), &DisturbanceTimer::unlockDist, this, true);    
}

void DisturbanceTimer::unlockDist(const ros::TimerEvent& e)
{
    disturbance_unlocked = true; // free disturbance lock
}

void DisturbanceTimer::stopImpact(const ros::TimerEvent& e)
{
    is_impact = false; // free disturbance lock during impact
}

void DisturbanceTimer::getImpulse(const std_msgs::Float64::ConstPtr& data)
{
    ros::Time t_imp = ros::Time::now();
    force_impulse = data->data;
    
    if ( force_impulse != 0. )
    {
        //std::rotate(lastImpactsTime.rbegin(), lastImpactsTime.rbegin() + 1, lastImpactsTime.rend());
        
        if( !is_impact )
        {
            // avoid considering multiple consecutive impacts in chaotic bouncings
            lastImpactsTime.pop_back();
            lastImpactsTime.insert(lastImpactsTime.begin(), t_imp.toSec());
            
            this->computePaddleFreq();

            if (disturbance_unlocked)
            {
                next_dist_timer = nh->createTimer(ros::Duration(paddle_period*cycle_delay), &DisturbanceTimer::trigger, this, true);            
            }
            
            is_impact = true; // avoid perturbation during impact
            stop_impact_timer = nh->createTimer(ros::Duration(0.03), &DisturbanceTimer::stopImpact, this, true);
        }

    }
}

void DisturbanceTimer::computePaddleFreq()
{
    //ros::Duration sum;
    double average;
    
    for (std::vector<float>::iterator it=lastImpactsTime.begin(); it!=lastImpactsTime.end()-1; ++it)
    //for (int it=0; it<lastImpactsTime.size()-1; ++it)
    {
        average += *(it+1) - *it;
        //average += lastImpactsTime.at(it+1) - lastImpactsTime.at(it);
    }
    average = average / (lastImpactsTime.size() - 1);
    
    if (average > 0)
    {
        paddle_period = average;
    }
    else
    {
        // manage error
    }
}
