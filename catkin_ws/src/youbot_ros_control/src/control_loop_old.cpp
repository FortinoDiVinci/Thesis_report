/*****************
 *    INCLUDES   *
 *****************/

#include <math.h>
#include <unistd.h>
#include <vector>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointValue.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>

#include <boost/scoped_ptr.hpp>

#include "youbot_ros_control/jacobian.h"
#include "youbot_ros_control/forward_kinematic.h"

#include "youbot_driver/generic/ConfigFile.hpp"

/*****************
 *   FUNCTIONS   *
 *****************/
 
#define DEBUG 0
#define NUMBER_ARM_JOINTS 5
#define DOF 6
// set DOF to 6 to use both force & torque and to 3 to use only force feedback

geometry_msgs::WrenchStamped sensor_data;
float thetas[NUMBER_ARM_JOINTS] = {0.,0.,0.,0.,0.};
float omegas[NUMBER_ARM_JOINTS] = {0.,0.,0.,0.,0.};
// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handlerx
void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

// get variable sign (1, 0 or -1)
template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

void getJointStateCallback(const sensor_msgs::JointState::ConstPtr& joint) 
{
    for (int i = 0; i < NUMBER_ARM_JOINTS; i++)
    {
        thetas[i] = joint->position[i];
        omegas[i] = joint->velocity[i];
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

// compute linear and rotational velocity of the robot end effector
void getVelocityTensor(const float jac_transpose_matrix[][DOF], const float joint_velocity[], float velocity_tensor[][6], const int nb_joints)
{
    //shift old value, n-1 values are kept in second row
    for (int i = 0; i < DOF; i++)
    {
        velocity_tensor[1][i] = velocity_tensor[0][i];
    }
    // v = J(q)*q_d
    for (int i = 0; i < nb_joints; i++) {
        for (int j = 0; j < DOF; j++) {
            velocity_tensor[0][j] += joint_velocity[i]*jac_transpose_matrix[i][j];
        }
    }
}

// compute linear acceleration using 2 points derivation (non centered)
ros::Time getAcceleration(const float velocity_tensor[][6], const ros::Time last_time, float acc[6])
{
    ros::Time t = ros::Time::now();
    float df =  1/(t - last_time).toSec(); // inverse of dt
    
    // linear acceleration is unused right now, uncomment if necessary
    
    //float vx = velocity_tensor[0][0];
    //float vy = velocity_tensor[0][1];
    //float vz = velocity_tensor[0][2];
    float wx = velocity_tensor[0][3];
    float wy = velocity_tensor[0][4];
    float wz = velocity_tensor[0][5];
    
    //float old_vx = velocity_tensor[1][0];
    //float old_vy = velocity_tensor[1][1];
    //float old_vz = velocity_tensor[1][2];
    float old_wx = velocity_tensor[1][3];
    float old_wy = velocity_tensor[1][4];
    float old_wz = velocity_tensor[1][5];
    
    //acc[0] = (vx - old_vx)*ft;
    //acc[1] = (vy - old_vy)*ft;
    //acc[2] = (vz - old_vz)*ft;
    acc[3] = (wx - old_wx)*df;
    acc[4] = (wy - old_wy)*df;
    acc[5] = (wz - old_wz)*df;
    
    return t;
}

// computes Coriolis and Centrifugal force, considering that the center of mass of the handle is on the z axis
// of the end effector, which induces several simplification for the computation
geometry_msgs::WrenchStamped compensateCoriolisCentrifugalForces(const float mass, const float center_of_mass, const float acc[6], const float velocity_tensor[][6], geometry_msgs::WrenchStamped data)
{
    //in our simplified case the center of mass coordinates are along the z axis of the sensor
    //the cross product are strongly simplified for computation purpose
    
    float coriolis[3];
    float centrifugal[3];
    
    coriolis[0] = mass*acc[4]*center_of_mass;
    coriolis[1] = -mass*acc[3]*center_of_mass;
    //coriolis[2] = 0;
    
    float rot_vel[3] = {velocity_tensor[0][3], velocity_tensor[0][4], velocity_tensor[0][5]};
    
    centrifugal[0] = mass*rot_vel[2]*rot_vel[0]*center_of_mass;
    centrifugal[1] = mass*rot_vel[2]*rot_vel[1]*center_of_mass;
    centrifugal[3] = mass*(-rot_vel[0]*rot_vel[0] - rot_vel[1]*rot_vel[1])*center_of_mass;
    
    geometry_msgs::WrenchStamped comp_data;
    
    comp_data.wrench.force.x = data.wrench.force.x + coriolis[0] + centrifugal[0];
    comp_data.wrench.force.y = data.wrench.force.y + coriolis[1] + centrifugal[1];
    comp_data.wrench.force.z = data.wrench.force.z + centrifugal[3];
    
    comp_data.wrench.torque.x = data.wrench.torque.x;
    comp_data.wrench.torque.y = data.wrench.torque.y;
    comp_data.wrench.torque.z = data.wrench.torque.z;
    
    comp_data.header.seq = data.header.seq;
    comp_data.header.stamp = data.header.stamp;
    comp_data.header.frame_id = data.header.frame_id;
    
    return comp_data;    
}

// from endpoint force torque data to joint torque data (Ti = J_t(q) * F)
void copyWrenchData(const geometry_msgs::WrenchStamped data, const float jac_transpose_matrix[][DOF], float joints_torque[], int nb_joints, int fst_jnt)
{
    float ft_sensor_d[6] = {data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z};
    
    for (int i = 0; i < nb_joints; i++) {
        float temp = 0;
        for (int j = 0; j < DOF; j++) {
            temp += jac_transpose_matrix[i][j]*ft_sensor_d[j]; 
        }
        joints_torque[fst_jnt + i] = temp;
    }
}

float* virtualGuideFixture_VerticalLine(const float x, const float ry, const float vx, const float wy, const float stiffness, const float damping, const float x0, const float ry0, geometry_msgs::WrenchStamped force_setpoint, const float jac_transpose_matrix[][DOF], float joints_torque_setpoint[], const int nb_joints, const int fst_jnt)
{
    // for proper use of this function, the other forces and torques should be initialized to zero !
    
    float dx = x - x0;
    float dry = ry - ry0;
    float abs_dx = abs(dx);
    float x1 = 0.05;
    float x2 = 0.05;
    
    if (abs_dx >= x2) 
    {
        force_setpoint.wrench.force.x = 0;
        force_setpoint.wrench.torque.y = 0;
    }
    else if (abs_dx <= x1)
    {
        force_setpoint.wrench.force.x = (-dx*stiffness - vx*damping)*0.7; 
        force_setpoint.wrench.torque.y = (dry*stiffness + wy*damping)*0.05;
        cout << "virtual fixture: " << force_setpoint.wrench.force.x <<'\n';
        cout << "virtual fixture: " << force_setpoint.wrench.torque.y <<'\n';
    }
    else
    {
        float alpha = (-abs_dx + x2)/(x2 - x1);
        force_setpoint.wrench.force.x = (-dx*stiffness - vx*damping )*alpha*0.7;
        force_setpoint.wrench.torque.y = (dry*stiffness + wy*damping)*alpha*0.05;
        cout << "virtual fixture: " << force_setpoint.wrench.force.x <<'\n';
    }
    //cout << "speed x : " << vx << '\n';
    cout << "wy : " << wy << '\n';
    force_setpoint.wrench.force.y = 0;
    force_setpoint.wrench.force.z = 0;
    force_setpoint.wrench.torque.x = 0;
    force_setpoint.wrench.torque.z = 0;
    copyWrenchData(force_setpoint, jac_transpose_matrix, joints_torque_setpoint, nb_joints, fst_jnt);
    return joints_torque_setpoint;
}

bool reachLimits(float theta, int joint)
{
    float joint_upper_limits[5] = {5.7401, 2.5179, -0.1157, 3.3292, 5.5415}; // 0.1 rad margin
    float joint_lower_limits[5] = {1.101e-1, 1.101e-1, -4.9266, 1.221e-1, 2.106e-1}; // 0.1 rad margin
    if (theta >= joint_upper_limits[joint]) return true;
    else if (theta <= joint_lower_limits[joint]) return true;
    else return false;
}

void initializeWrenchStamped(geometry_msgs::WrenchStamped *f, const std::string reference_frame)
{
    f->wrench.force.x = 0;
    f->wrench.force.y = 0;
    f->wrench.force.z = 0;
    f->wrench.torque.x = 0;
    f->wrench.torque.y = 0;
    f->wrench.torque.z = 0;
    
    f->header.stamp = ros::Time::now();
    f->header.frame_id = reference_frame;
}

// debugging function
void dispJacobian(const float jac_transpose_matrix[][DOF], const int nb_joints)
{
    for(int i=0; i < nb_joints; i++)
    {
        for(int j=0; j < DOF; j++)
        {
            std::cout << jac_transpose_matrix[i][j] << '\t';
        }
        std::cout << '\n';
    }
    std::cout << '\n';
}

void dispWrenchStamped(geometry_msgs::WrenchStamped ft_data, bool dispForce, bool dispTorque)
{
    if(dispForce)
    {
        std::cout << "Fx: " << ft_data.wrench.force.x << '\t' << "Fy: " << ft_data.wrench.force.y << '\t' << "Fz: " << ft_data.wrench.force.z << '\n';
    }
    if(dispTorque)
    {
        std::cout << "Tx: " << ft_data.wrench.torque.x << '\t' << "Ty: " << ft_data.wrench.torque.y << '\t' << "Tz: " << ft_data.wrench.torque.z << '\n';
    }
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
    ros::Publisher pub_debug = n.advertise<std_msgs::Float32MultiArray>
        ("debug/joint_torque_from_jacobian", 10);
    ros::Publisher pub_debug_j = n.advertise<std_msgs::Float32MultiArray>
        ("debug/jacobian", 10);
        
    ros::Publisher pub_debug_force_comp = n.advertise<geometry_msgs::WrenchStamped> ("debug/force_comp",10);
    #endif

    boost::scoped_ptr<youbot::ConfigFile> configfile;
    std::string youbot_driver_path = ros::package::getPath("youbot_driver");
    configfile.reset(new youbot::ConfigFile("youbot-manipulator.cfg", youbot_driver_path + "/config"));
    float joint_max_velocity[5];

    for (unsigned int i = 0; i < NUMBER_ARM_JOINTS; i++) 
    {
        std::stringstream jointNameStream;
        jointNameStream << "Joint_" << i + 1;
        std::string jointName = jointNameStream.str();
        configfile->readInto(joint_max_velocity[i], jointName, "MaxVelocity");
	}

    float freq;
    float K[5];
    float Ki[5];
    float K_temp;
    float Ki_temp;
    float K_vm;         // virtual stiffness (N.m)
    float B_vm;         // virtual damping (N.m.s^-1)
    float x0;           // virtual equilibrium position (m)
    float ry0 = 1.5708; // virtual equilibrium orientation (rad)
    int nb_jnt_ctrl = NUMBER_ARM_JOINTS;
    int joint_i;
    float l;
    float m;    
    std::string sensor_l_param_name = "sensor_arm_lever";
    std::string sensor_m_param_name = "sensor_mass";
    
    if (n.searchParam("force_sensor_utils", sensor_l_param_name))
    {
        n.getParam("sensor_arm_lever", l);
    }
    else
    {  
        l = 0.0103;
        ROS_WARN("Could not locate arm lever parameter, value set to %f", l);
    }
    if (n.searchParam("force_sensor_utils", sensor_m_param_name))
    {
        n.getParam("sensor_mass", m);
    }
    else
    {
        m = 0.1096;
        ROS_WARN("Could not locate sensor mass parameter, value set to %f", m);
    }

    n1.param<float>("rate", freq, 100.); 
    n1.param<float>("K1_gain", K[0], 1.); 
    n1.param<float>("Ki1_gain", Ki[0], 1.); 
    n1.param<float>("K2_gain", K[1], 1.); 
    n1.param<float>("Ki2_gain", Ki[1], 1.); 
    n1.param<float>("K3_gain", K[2], 1.); 
    n1.param<float>("Ki3_gain", Ki[2], 1.); 
    n1.param<float>("K4_gain", K[3], 1.); 
    n1.param<float>("Ki4_gain", Ki[3], 1.); 
    n1.param<float>("K5_gain", K[4], 1.); 
    n1.param<float>("Ki5_gain", Ki[4], 1.); 
	
    n1.param<int>("Nb_joints_ctrl", nb_jnt_ctrl, 2);
    
    n1.param<float>("Stiffness", K_vm, 100.); 
    n1.param<float>("Damping", B_vm, 10.);
    n1.param<float>("Equilibrium", x0, -0.29);
    //n1.param<int>("First_axis_nb", joint_i, 2);
    joint_i--; // to adapt to c++
       
    float init_offset[NUMBER_ARM_JOINTS] = {169, 65, -146, 102.5-90, 167.5-110};
    float off_offset[NUMBER_ARM_JOINTS] = {0.011, 0.011, -0.016, 0.023, 0.12};
    
    if (nb_jnt_ctrl < 1 || nb_jnt_ctrl > 3) {
        ROS_WARN("The number of joint controlled must now be between 1 and 3, it will be set to 3");
        nb_jnt_ctrl = 3;
		joint_i = 1;
    }
    
    ros::Rate rate(freq);

    brics_actuator::JointVelocities velocities_cmd;
    brics_actuator::JointPositions positions_cmd;
    std::vector <brics_actuator::JointValue> vel;
    std::vector <brics_actuator::JointValue> pos;   
    pos.resize(NUMBER_ARM_JOINTS);
    
    if (nb_jnt_ctrl == 1) {
        joint_i = 3; // only controls joint 4
    }
    else if (nb_jnt_ctrl == 2) {
        joint_i = 2; // controls joints 3 & 4
    }
    else if (nb_jnt_ctrl == 3) {
        joint_i = 1; // controls joints 2, 3 & 4

        for (int i=1; i < 3; i++)
        {
            Ki[joint_i + i] = Ki[joint_i + i]*0.35;
            K[joint_i + i] = K[joint_i + i]*0.35;
        }
    }    
    // convert offset to radians
    for (int i = 0; i < NUMBER_ARM_JOINTS; i++) init_offset[i] *= M_PI/180;
    // Set axis to required position
    for (int i = 0; i < NUMBER_ARM_JOINTS; i++) {
        std::stringstream jointNameStream;
        jointNameStream << "" << i + 1;
        pos[i].joint_uri = "arm_joint_" + jointNameStream.str();
        pos[i].unit = "rad";
        pos[i].value = init_offset[i];
    }
    
    vel.resize(nb_jnt_ctrl);
    
    for (int i = 0; i < nb_jnt_ctrl; i++) {
        std::stringstream jointNameStream;
        jointNameStream << "" << joint_i + i + 1;
        vel[i].joint_uri = "arm_joint_" + jointNameStream.str();
        vel[i].unit = "s^-1 rad";
        vel[i].value = 0;
    }

    velocities_cmd.velocities = vel;

    float joints_torque_feedback[NUMBER_ARM_JOINTS];
    float jacobian_t[nb_jnt_ctrl][DOF];
    float sum_err[5] = {0., 0., 0., 0., 0.};
    float Te = 1/freq;              // sampling time
    ros::Time t;
    ros::Time l_t;
    ros::Time l_t_acc;
    float delay = Te; // in the ideal case, delay = Te

    #if DEBUG == 1
        std_msgs::Float32MultiArray joints_torque;
        std_msgs::Float32MultiArray jacobian_debug;
        
        jacobian_debug.layout.data_offset = 0;
        jacobian_debug.layout.dim.push_back(std_msgs::MultiArrayDimension());
        jacobian_debug.layout.dim.push_back(std_msgs::MultiArrayDimension());
        //jacobian_debug.layout.dim[0].label = "rows";
        //jacobian_debug.layout.dim[0].size = nb_jnt_ctrl;
        //jacobian_debug.layout.dim[0].stride = nb_jnt_ctrl*DOF;
        //jacobian_debug.layout.dim[1].label = "columns";
        //jacobian_debug.layout.dim[1].size = DOF;
        //jacobian_debug.layout.dim[1].stride = DOF;
        
        for (int i = 0; i < NUMBER_ARM_JOINTS; i++) 
        {
            joints_torque.data.push_back(0.); // malloc ?
        }   
        for (int i = 0; i < DOF*nb_jnt_ctrl; i++) 
        {
            jacobian_debug.data.push_back(0.); // malloc ?
        }
        
    #endif
    positions_cmd.positions = pos;
    usleep(1.0*1e6); // this delay seems necessary..
    pub_pos.publish(positions_cmd);
    usleep(2.0*1e6); // wait 2 seconds for the end of the movement
    
    float robot_endpoint_xyz[3] = {0,0,0};
    float old_robot_endpoint_xyz[3] = {0,0,0};
    float robot_endpoint_angle[3] = {0,0,0};     // enpoint orientation angle
    float old_robot_endpoint_angle[3] = {0,0,0}; 
    float vel_tensor[2][6] = {{0,0,0,0,0,0}, {0,0,0,0,0,0}};
    float acc[6] = {0,0,0,0,0,0};
    geometry_msgs::WrenchStamped comp_force;
    geometry_msgs::WrenchStamped force_setpoint;
    initializeWrenchStamped(&force_setpoint, "base_link");
    
    float joints_torque_setpoint[NUMBER_ARM_JOINTS];
    for (int i = 0; i < NUMBER_ARM_JOINTS; i++) joints_torque_setpoint[i] = 0;
    
    l_t = ros::Time::now();
    /************
     *   LOOP   *
     ************/
     
    while (!g_request_shutdown) {
    
        if (nb_jnt_ctrl == 1)
        {
            #if DOF == 6
                youBotJacobianTJoint3(thetas[joint_i], jacobian_t);
            #endif
            //getVelocityTensor(jacobian_t, omegas, vel_tensor, nb_jnt_ctrl);
            //l_t_acc = getAcceleration(vel_tensor, l_t_acc, acc);
            //comp_force = compensateCoriolisCentrifugalForces(m, l, acc, vel_tensor, sensor_data);
            copyWrenchData(sensor_data, jacobian_t, joints_torque_feedback, nb_jnt_ctrl, joint_i);
        }
        else if (nb_jnt_ctrl == 2)
        {
            #if DOF == 6
                youBotJacobianTJoints34(thetas, jacobian_t);
            #elif DOF == 3
                youBotJacobianTJoints34XYZDOF(thetas, jacobian_t);
            #endif
            //non contact forces du to speed and acceleration can be neglected
            //getVelocityTensor(jacobian_t, omegas, vel_tensor, nb_jnt_ctrl);
            //l_t_acc = getAcceleration(vel_tensor, l_t_acc, acc);
            //comp_force = compensateCoriolisCentrifugalForces(m, l, acc, vel_tensor, sensor_data);
            //forwardKinematicTranslationOnly(thetas, robot_endpoint_xyz);
            //std::cout << "x : " << robot_endpoint_xyz[0] << std::endl;
            //std::cout << "vx : " << vel_tensor[0][0] << std::endl;
            //virtualGuideFixture_VerticalLine(robot_endpoint_xyz[0], vel_tensor[0][0], K_vm, B_vm, x0, force_setpoint, jacobian_t, joints_torque_setpoint, nb_jnt_ctrl, joint_i);
            copyWrenchData(sensor_data, jacobian_t, joints_torque_feedback, nb_jnt_ctrl, joint_i);
        }
        else if (nb_jnt_ctrl == 3)
        {
            #if DOF == 6
                youBotJacobianTJoints234(thetas, jacobian_t);
            #elif DOF == 3
                youBotJacobianTJoints234ZDOF(thetas, jacobian_t);
            #endif
            //dispJacobian(jacobian_t, nb_jnt_ctrl);
            //getVelocityTensor(jacobian_t, omegas, vel_tensor, nb_jnt_ctrl);
            
            // Cartesian Pose computation
            old_robot_endpoint_xyz[0] = robot_endpoint_xyz[0];
            forwardKinematicTranslationOnly(thetas, robot_endpoint_xyz);
            vel_tensor[0][0] = (robot_endpoint_xyz[0] - old_robot_endpoint_xyz[0]) / delay;
            std::cout << "x : " << robot_endpoint_xyz[0] << std::endl;
            old_robot_endpoint_angle[1] = robot_endpoint_angle[1];
            getYEndPointAngle(thetas, &robot_endpoint_angle[1]);
            vel_tensor[4][0] = (robot_endpoint_angle[1] - old_robot_endpoint_angle[1]) / delay; 
            std::cout << "ry : " << robot_endpoint_angle[1] << std::endl;
            
            virtualGuideFixture_VerticalLine(robot_endpoint_xyz[0], robot_endpoint_angle[1], vel_tensor[0][0], vel_tensor[4][0], K_vm, B_vm, x0, ry0, force_setpoint, jacobian_t, joints_torque_setpoint, nb_jnt_ctrl, joint_i);
            copyWrenchData(sensor_data, jacobian_t, joints_torque_feedback, nb_jnt_ctrl, joint_i);
        }
        else
        {
            #if DOF == 6
                youBotJacobianT(thetas, jacobian_t);
            #endif
            copyWrenchData(sensor_data, jacobian_t, joints_torque_feedback, nb_jnt_ctrl, 0);
        }

        #if DEBUG == 1
            for (int i = 0; i < NUMBER_ARM_JOINTS; i++) {
                joints_torque.data[i] = joints_torque_feedback[i];
                //jacobian_debug.data[i] = jacobian_t[0][i];
            }
            //reshaping the data to ros multiarray
            for (int i = 0; i < nb_jnt_ctrl; i++)
            {
                for (int j = 0; j < DOF; j++)
                {
                    jacobian_debug.data[i*DOF + j] = jacobian_t[i][j];
                }
            }
            pub_debug.publish(joints_torque);
            pub_debug_j.publish(jacobian_debug);
            pub_debug_force_comp.publish(comp_force);
        #endif

        t = ros::Time::now();
        delay = (t - l_t).toSec();
        l_t = t; 
		
        if ( (delay - Te) > 0.05*Te) {
            ROS_WARN("Sampling time overpassed: %.5f, should be %.5f", delay, Te);
        }
		
        for (int i = 0; i < nb_jnt_ctrl; i++) {
        
            if (reachLimits(thetas[joint_i + i], joint_i + i)) sum_err[joint_i + i] = 0; //anti windup
            else sum_err[joint_i + i] += (joints_torque_feedback[joint_i + i] + joints_torque_setpoint[joint_i + i])*delay;
            
            velocities_cmd.velocities[i].value = K[joint_i + i]*(joints_torque_feedback[joint_i + i] + joints_torque_setpoint[joint_i + i]) + Ki[joint_i + i]*sum_err[joint_i + i];
            
            // recompute integral action val to reach max velocity
            if (abs(velocities_cmd.velocities[i].value) >= joint_max_velocity[joint_i + i])
            {
                sum_err[joint_i + i] = (sign(velocities_cmd.velocities[i].value)*joint_max_velocity[joint_i + i] - K[joint_i + i]*(joints_torque_feedback[joint_i + i] + joints_torque_setpoint[joint_i + i]))/Ki[joint_i + i];
                velocities_cmd.velocities[i].value = sign(velocities_cmd.velocities[i].value)*joint_max_velocity[joint_i + i];
            }
        }
        
        for(int i = 0; i < nb_jnt_ctrl; i++){
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
