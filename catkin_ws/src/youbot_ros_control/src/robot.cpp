#include "youbot_ros_control/robot.h"

/********
 * PID
 ********/

ros::Time PID::t;
ros::Time PID::t_old;

PID::PID()
{
    this->Kp = 0;
    this->Ki = 0;
    this->Kd = 0;
    
    this->error_sum = 0;
}

PID::PID(float Kp, float Ki, float Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    
    this->error_sum = 0;
}

// time
void PID::setLastTimeSample()
{
    t_old = ros::Time::now();
}

void PID::setCurrentTimeSample()
{
    t = ros::Time::now();
}

void PID::setLastTimeSample(const ros::Time time)
{
    t_old = time;
}
void PID::setCurrentTimeSample(const ros::Time time)
{
    t = time;
}

void PID::antiWindup(const float maxVal, const float error)
{
    error_sum = maxVal - Kp/Ki*error;
}

float PID::compute(const float error, const bool limitReached)
{
    float cmd; // pid output
    
    limitReached ? error_sum = 0 : error_sum += error; // anti windup
    cmd = Kp * error + Ki * error_sum * (t - t_old).toSec();
    
    return cmd;
}

/********
 * JOINT
 ********/

Joint::Joint(float max_th, float min_th, PID val)
{
    max_angle = max_th;
    min_angle = min_th;
    
    impedance_control = val;
}

// joint setters & getters
float Joint::getAngle() const
{
    return angle;
}

float Joint::getAngularVelocity() const
{
    return angular_velocity;
}

float Joint::getEffort() const
{
    return effort;
}

void Joint::setAngle(const float val)
{
    angle = val;
}

void Joint::setAngularVelocity(const float val)
{
    angular_velocity = val;
}

void Joint::setEffort(const float val)
{
    effort = val;
}

Joint& Joint::operator = (const Joint &val)
{
    angle = val.angle;
    angular_velocity = val.angular_velocity;
    effort = val.effort;
        
    impedance_control = val.impedance_control;
        
    max_angle = val.max_angle;
    min_angle = val.min_angle;
    
    max_velocity = val.max_velocity;
}

/***********
 * ENDPOINT
 ***********/

Endpoint::Endpoint()
{
    pose.setPositionUnit("m");
    pose.setOrientationUnit("rad");
    
    velocities.setPositionUnit("s^-1 m");
    velocities.setOrientationUnit("s^-1 rad");
    
    accelerations.setPositionUnit("s^-2 m");
    accelerations.setOrientationUnit("s^-2 rad");
    
    forces.setPositionUnit("N");
    forces.setOrientationUnit("N m");
}

/***********
 * JACOBIAN
 ***********/

Jacobian::Jacobian()
{
    degree_of_freedom = 0;
    actuated_joints = 0;
        
}

Jacobian::Jacobian(int dof, int nb_joints, bool onlyM = false, bool onlyT = false, bool onlyI = false)
{
    if ((onlyM && onlyT) || (onlyM && onlyI) || (onlyI && onlyT))
    {
        std::cout << "Inappropriate argument, please chose either only matrix, only transpose matrix or neither" << '\n';
    }

    degree_of_freedom = dof;
    actuated_joints = nb_joints;
    
    if(onlyM)
    {
        matrix.resize(degree_of_freedom, actuated_joints);
    }
    else if(onlyT)
    {
        t_matrix.resize(actuated_joints, degree_of_freedom);
    }
    else if(onlyI)
    {
        inv_matrix.resize(actuated_joints, degree_of_freedom);
    }
    else
    {
        matrix.resize(degree_of_freedom, actuated_joints);
        t_matrix.resize(actuated_joints, degree_of_freedom);
        inv_matrix.resize(actuated_joints, degree_of_freedom);
    }
}

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Jacobian::getTransposeMatrix() const
{
    return t_matrix;
}

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Jacobian::getMatrix() const
{
    return matrix;
}

void Jacobian::setTransposeMatrix()
{
    /*
    for (int h = 0; h < degree_of_freedom; h++)
    {
        for (int w = 0; w < actuated_joints; w++)
        {
            t_matrix[w][h] = matrix[h][w];
        }
    }
     */
    t_matrix = matrix.transpose();
}

void Jacobian::setTransposeMatrix(float **t_jac)
{
    /* TODO Check input */   
    
    for (int h = 0; h < actuated_joints; h++)
    {
        for (int w = 0; w < degree_of_freedom; w++)
        {
            t_matrix(h, w) = t_jac[h][w];
        }
    }
}

void Jacobian::setMatrix(float **jac)
{
    for (int h = 0; h < degree_of_freedom; h++)
    {
        for (int w = 0; w < actuated_joints; w++)
        {
            matrix(h, w) = jac[h][w];
        }
    }
}

void Jacobian::computeTransposeMatrix(std::vector<Joint> joints)
{
    if (t_matrix.rows() != actuated_joints)
    {
        std::cout << "error in jacobian transpose matrix computation, please check dimensions.\n" << "There are " << actuated_joints << " actuated joints, and jacobian matrix dimensions are supose to be " << t_matrix.rows() << "x" << t_matrix.cols() << "\n";
        return;
    }

    switch (actuated_joints) {
        case 3:
            if (degree_of_freedom == 6)
            {
                youBotTJoints234(joints);
            }
            else if (degree_of_freedom == 3)
            {
                youBotTJoints234_3DOF(joints);
            }
            else
            {
                std::cout << "err msg" << '\n';
            }
            break;
        case 1:
            // TODO
            break;
        case 2:
            //TODO
            break;
        default:
            break;
    }
}

void Jacobian::computeInverseMatrix(std::vector<Joint> joints)
{
    if (inv_matrix.rows() != actuated_joints)
    {
        std::cout << "error in jacobian inverse matrix computation, please check dimensions.\n" << "There are " << actuated_joints << " actuated joints, and jacobian matrix dimensions are supose to be " << inv_matrix.rows() << "x" << inv_matrix.cols() << "\n";
        return;
    }

    switch (actuated_joints) {
        case 3:
            if (degree_of_freedom == 6)
            {
                youBotInvJoints234(joints);
            }
            else if (degree_of_freedom == 3)
            {
                youBotInvJoints234_3DOF(joints);
            }
            else
            {
                std::cout << "err msg" << '\n';
            }
            break;
        case 1:
            // TODO
            break;
        case 2:
            //TODO
            break;
        default:
            break;
    }
}

void Jacobian::computeMatrix(std::vector<Joint> joints)
{
    if (matrix.cols() != actuated_joints)
    {
        std::cout << "error in jacobian matrix computation, please check dimensions.\n" << "There are " << actuated_joints << " actuated joints, and jacobian matrix dimensions are supose to be " << matrix.rows() << "x" << matrix.cols() << "\n";
        return;
    }
        
    switch (actuated_joints) {
        case 1:
            // TODO
            break;
        case 2:
            //TODO
            break;
        case 3:
            if (degree_of_freedom == 3)
            {
                // TODO
            }
            else if (degree_of_freedom == 6)
            {
                // TODO
            }
            else
            {
                std::cout << "err msg" << '\n';
            }
            break;
        default:
            break;
    }
}

void Jacobian::youBotTJoints234(const std::vector<Joint> joints) 
{
    float th2 = joints[1].getAngle();
    float th3 = joints[2].getAngle();
    float th4 = joints[3].getAngle();
    
    float t2 = M_PI*(0.45); 
    float t3 = t2 + th2 + th3; 
    float t7 = M_PI*(4.3e1/3.6e2); 
    float t4 = -t7 + th2 + th3 + th4; 
    float t5 = M_PI*(5.0/3.6e1); 
    float t6 = t5 + th2; 
    float t8 = sin(t4)*(-0.123); 
    float t9 = cos(t6); 
    float t10 = cos(t3); 
    float t11 = t10*(0.135); 
    float t12 = cos(t4);
    float t13 = t12*(0.123); 
    float t15 = th2 + th3 - M_PI*(0.05);
    float t16 = cos(t15)*(0.135);

    t_matrix(0, 0) = t11 + t13 + sin(t6)*(0.155);
    t_matrix(0, 1) = 0;
    t_matrix(0, 2) = t8 + t9*(0.155) - t16;
    t_matrix(0, 3) = 0.0;
    t_matrix(0, 4) = -1.0;
    t_matrix(0, 5) = 0;
    
    t_matrix(1, 0) = t11 + t13;
    t_matrix(1, 1) = 0;
    t_matrix(1, 2) = t8 - t16;    
    t_matrix(1, 3) = 0.0;    
    t_matrix(1, 4) = -1.0;    
    t_matrix(1, 5) = 0;

    t_matrix(2, 0) = t13;
    t_matrix(2, 1) = 0;
    t_matrix(2, 2) = t8;
    t_matrix(2, 3) = 0.0;
    t_matrix(2, 4) = -1.0;
    t_matrix(2, 5) = 0;
}

void Jacobian::youBotTJoints234_3DOF(const std::vector<Joint> joints) 
{
    float th2 = joints[1].getAngle();
    float th3 = joints[2].getAngle();
    float th4 = joints[3].getAngle();
    
    float t2 = M_PI*(0.45); 
    float t3 = t2 + th2 + th3; 
    float t7 = M_PI*(4.3e1/3.6e2); 
    float t4 = -t7 + th2 + th3 + th4; 
    float t5 = M_PI*(5.0/3.6e1); 
    float t6 = t5 + th2; 
    float t8 = sin(t4)*(-0.123); 
    float t9 = cos(t6); 
    float t10 = cos(t3); 
    float t11 = t10*(0.135); 
    float t12 = cos(t4);
    float t13 = t12*(0.123); 
    float t15 = th2 + th3 - M_PI*(0.05);
    float t16 = cos(t15)*(0.135);

    t_matrix(0, 0) = t11 + t13 + sin(t6)*(0.155);
    t_matrix(0, 1) = t8 + t9*(0.155) - t16;
    t_matrix(0, 2) = -1.0;
    
    t_matrix(1, 0) = t11 + t13;
    t_matrix(1, 1) = t8 - t16;      
    t_matrix(1, 2) = -1.0;    

    t_matrix(2, 0) = t13;
    t_matrix(2, 1) = t8;
    t_matrix(2, 2) = -1.0;
}

void Jacobian::youBotInvJoints234(const std::vector<Joint> joints) 
{
    float th2 = joints[1].getAngle();
    float th3 = joints[2].getAngle();
    float th4 = joints[3].getAngle();
	
    float t2 = M_PI*(9.0/2.0e1);
    float t3 = t2 + th2 + th3;
    float t4 = M_PI*(1.4e1/4.5e1);
    float t5 = t4 + th3;
    float t6 = cos(t5);
    float t7 = 1.0/t6;
    float t8 = sin(t3);
    float t9 = cos(t3);
    float t10 = M_PI*(5.0/3.6e1);
    float t11 = t10 + th2;
    float t12 = M_PI*(3.1e1/7.2e1);
    float t13 = t12 + th4;
    float t14 = sin(t13);
    float t15 = cos(t11);
    float t16 = sin(t11);
    float t17 = M_PI*(2.9e1/1.2e2);
    float t18 = t17 + th3 + th4;
    float t19 = sin(t18);
    
    // TODO
    
    
    inv_matrix(0, 0) = t7*t8*(2.0e2/3.1e1);
    inv_matrix(0, 1) = 0.;
    inv_matrix(0, 2) = t7*(t8*2.7e1 - t15*3.1e1)*(-2.0e2/8.37e2);
    inv_matrix(0, 3) = 0.;
    inv_matrix(0, 4) = t7*t15*(-2.0e2/2.7e1);
    inv_matrix(0, 5) = 0.;

    inv_matrix(1, 0) = t7*t9*(2.0e2/3.1e1);
    inv_matrix(0, 1) = 0.;
    inv_matrix(1, 2) = t7*(t9*2.7e1 + t16*3.1e1)*(-2.0e2/8.37e2);
    inv_matrix(0, 3) = 0.;
    inv_matrix(1, 4) = t7*t16*(2.0e2/2.7e1);
    inv_matrix(0, 5) = 0.;

    inv_matrix(2, 0) = t7*t14*(1.84e2/1.55e2);
    inv_matrix(0, 1) = 0.;
    inv_matrix(2, 2) = t7*(t14*2.7e1 - t19*3.1e1)*(-4.396654719235364e-2);
    inv_matrix(0, 3) = 0.;
    inv_matrix(2, 4) = t7*(t6*1.35e2 + t19*1.84e2)*(-1.0/1.35e2);
    inv_matrix(0, 5) = 0.;   
}

void Jacobian::youBotInvJoints234_3DOF(const std::vector<Joint> joints) 
{
    float th2 = joints[1].getAngle();
    float th3 = joints[2].getAngle();
    float th4 = joints[3].getAngle();
	
    float t2 = M_PI*(9.0/2.0e1);
    float t3 = t2 + th2 + th3;
    float t4 = M_PI*(1.4e1/4.5e1);
    float t5 = t4 + th3;
    float t6 = cos(t5);
    float t7 = 1.0/t6;
    float t8 = sin(t3);
    float t9 = cos(t3);
    float t10 = M_PI*(5.0/3.6e1);
    float t11 = t10 + th2;
    float t12 = M_PI*(3.1e1/7.2e1);
    float t13 = t12 + th4;
    float t14 = sin(t13);
    float t15 = cos(t11);
    float t16 = sin(t11);
    float t17 = M_PI*(2.9e1/1.2e2);
    float t18 = t17 + th3 + th4;
    float t19 = sin(t18);

    inv_matrix(0, 0) = t7*t8*(2.0e2/3.1e1);
    inv_matrix(0, 1) = t7*(t8*2.7e1 - t15*3.1e1)*(-2.0e2/8.37e2);
    inv_matrix(0, 2) = t7*t15*(-2.0e2/2.7e1);

    inv_matrix(1, 0) = t7*t9*(2.0e2/3.1e1);
    inv_matrix(1, 1) = t7*(t9*2.7e1 + t16*3.1e1)*(-2.0e2/8.37e2);
    inv_matrix(1, 2) = t7*t16*(2.0e2/2.7e1);

    inv_matrix(2, 0) = t7*t14*(1.84e2/1.55e2);
    inv_matrix(2, 1) = t7*(t14*2.7e1 - t19*3.1e1)*(-4.396654719235364e-2);
    inv_matrix(2, 2) = t7*(t6*1.35e2 + t19*1.84e2)*(-1.0/1.35e2);
}

void Jacobian::dispTransposeMatrix()
{
    for(int i=0; i < actuated_joints; i++)
    {
        for(int j=0; j < degree_of_freedom; j++)
        {
            std::cout << t_matrix(i, j) << '\t';
        }
        std::cout << '\n';
    }
    std::cout << '\n';    
}

void Jacobian::dispMatrix()
{
    for(int i=0; i < degree_of_freedom; i++)
    {
        for(int j=0; j < actuated_joints; j++)
        {
            std::cout << matrix(i, j) << '\t';
        }
        std::cout << '\n';
    }
    std::cout << '\n';    
}

/********
 * ROBOT
 ********/
 
Robot::Robot(const std::vector<Joint> jnt, const bool act_jnts_tab[], const Jacobian jac, brics_actuator::JointPositions pos_msg, ros::NodeHandle *n)
{
    nb_joints = jnt.size();
    joints = jnt;
    
    endpoint_limits.resize(2);
    
    for (int i = 0; i < nb_joints; i++)
    {
        actuated_joints_table.push_back(act_jnts_tab[i]);
    }
    
    jacobian = jac;
    
    velocities_cmd_msg.velocities.resize(jacobian.actuated_joints);
    int idx = 0;

    for (int ii = 0; ii < nb_joints; ii++)
    {
        if (actuated_joints_table[ii] == true)
        {
            std::stringstream jointNameStream;
            jointNameStream << "" << ii + 1;        
            velocities_cmd_msg.velocities[idx].joint_uri = "arm_joint_" + jointNameStream.str();
            velocities_cmd_msg.velocities[idx].unit = "s^-1 rad";
            velocities_cmd_msg.velocities[idx].value = 0.0;
            idx++;
        }
        else continue;
    }
    
    //ROS_INFO_STREAM("\n" << velocities_cmd_msg << "\n");
    
    positions_cmd_msg = pos_msg;
    
    std::string topic_name = "";
    topic_name = "arm_1/arm_controller/velocity_command";
    pub_vel_cmd_msg = n->advertise<brics_actuator::JointVelocities>(topic_name, 1);
    topic_name = "arm_1/arm_controller/position_command";
    pub_pos_cmd_msg = n->advertise<brics_actuator::JointPositions>(topic_name, 1);
  
}

bool Robot::jointLimitReached(const int joint_nb)
{
    if (joints[joint_nb].angle >= joints[joint_nb].max_angle) return true;
    else if (joints[joint_nb].angle <= joints[joint_nb].min_angle) return true;
    else return false;
}

bool Robot::endpointLimitReached(const int i)
{
    if (endpoint.pose.getPoseVector()(i, 0) >= endpoint_limits[1].pose.getPoseVector()(i, 0)) return true;
    else if (endpoint.pose.getPoseVector()(i, 0) <= endpoint_limits[0].pose.getPoseVector()(i, 0)) return true;
    else return false;
}

// endpoint
void Robot::computeEnpointPosition()
{
    switch(jacobian.actuated_joints) 
    {
        case 3:
            old_endpoint.pose.setPosition(endpoint.pose.getPosition());
            endpoint.pose.setPosition(getEndpointPosition());
            break;
        case 1:
            break;
        case 2:
            break;
        default:
            break;
    }
}

void Robot::computeEnpointOrientation(const bool onlyX = false, const bool onlyY = false, const bool onlyZ = false)
{
    switch(jacobian.actuated_joints) 
    {
        case 3:
            if(onlyY) 
            {
                old_endpoint.pose.setOrientationY(endpoint.pose.getOrientation().y);
                endpoint.pose.setOrientationY(getYEndpointAngle());
                break;
            }
            else if(onlyX) 
            {
                //endpoint.pose.setOrientationX(getXEndpointAngle()); // TODO ...
                break;
            }
            else if(onlyZ) 
            {
                //endpoint.pose.setOrientationZ(getZEndpointAngle()); // TODO ...
                break;
            }
            else
            {
                //endpoint.pose.setOrientation(getEndpointAngle()); // TODO ...
            }
            break;
        case 1:
            break;
        case 2:
            break;
        default:
            break;
    }
}

/*
Point Robot::getEndpointPosition() 
{
    float t4 = M_PI*(1.1e1/1.8e2);
    float t5 = t4 + joints[0].getAngle();
    float t6 = M_PI*(5.0/3.6e1);
    float t7 = t6 + joints[1].getAngle();
    float t8 = sin(t5);
    float t9 = M_PI*(1.4e1/4.5e1);
    float t10 = t9 + joints[2].getAngle();
    float t11 = cos(t7);
    float t12 = cos(t5);
    float t13 = sin(t7);
    float t14 = M_PI*(3.1e1/7.2e1);
    float t15 = t14 + joints[3].getAngle();
    float t16 = cos(t10);
    float t18 = t12*t13;
    float t20 = sin(t10);
    float t26 = t11*t12;
    float t22 = -t26;
    float t24 = cos(t15);
    float t25 = t16*t18;
    float t33 = t20*t22;
    float t27 = t25 - t33;
    float t28 = sin(t15);
    float t29 = t16*t22;
    float t30 = t18*t20;
    float t31 = t29 + t30;
    float t39 = t8*t13;
    float t41 = t8*t11;
    float t45 = t16*t39;
    float t46 = t20*t41;
    float t47 = t45 + t46;
    float t48 = t16*t41;
    float t51 = t20*t39;
    float t49 = t48 - t51;
    float t57 = M_PI*(1.0/2.0e1);
    float t58 = joints[1].getAngle() + joints[2].getAngle() + joints[3].getAngle() - M_PI*(4.3e1/3.6e2);
    float t59 = cos(t58);
    
    float a1 = 2.7e1/2.0e2;
    float a2 = 3.3e1/1.0e3;
    float a3 = 3.1e1/2.0e2;
    float a4 = 1.09e2/5.0e2;
    
    float tx = -t12*(a2) + t11*t12*(a3) - t16*t18*(a1) + t24*t27*(a4) - t28*t31*(a4) + t20*(-t26)*(a1); 
    float ty = t8*(a2) - t8*t11*(a3) + t20*t41*(a1) - t24*t47*(a4) - t28*t49*(a4)+t16*(t39)*(a1); 
    float tz = t13*(a3) + t59*(a4) - sin(-t57 + joints[1].getAngle() + joints[2].getAngle())*(a1) + 1.47e-1;
    
    ROS_INFO_STREAM(Point(tx, ty, tz, "m"));
    
    return Point(tx, ty, tz, "m");
} */

Point Robot::getEndpointPosition() 
{
    float th2 = joints[1].getAngle();
    float th3 = joints[2].getAngle();
    float th4 = joints[3].getAngle();

    float t2 = M_PI*(1.1e1/1.8e2);
    float t3 = t2 + joints[0].getAngle();
    float t4 = cos(t3);
    float t5 = M_PI*(5.0/3.6e1);
    float t6 = t5 + th2;
    float t7 = sin(t3);
    float t8 = cos(t6);
    float t9 = sin(t6);
    float t10 = M_PI*(1.4e1/4.5e1);
    float t11 = t10 + th3;
    float t12 = cos(t11);
    float t14 = t4*t9;
    float t16 = sin(t11);
    float t17 = t4*t8;
    float t19 = M_PI*(3.1e1/7.2e1);
    float t20 = t19 + th4;
    float t22 = cos(t20);
    float t29 = -t7*t9;
    float t25 = t7*t8;
    float t28 = sin(t20);

    float tx = t4*(-0.033) + + t4*t8*(0.155) - t12*t14*(0.135) - t16*t17*(0.135) + t22*(t12*t14 + t16*t17)*(0.184) + t28*(t12*t17 - t14*t16)*(0.184);
    float ty = t7*(0.033) - t7*t8*(0.155) - t12*t29*(0.135) + t16*t25*(0.135) + t22*(t12*t29 - t16*t25)*(0.184) - t28*(t12*t25 + t16*t29)*(0.184);
    float tz = t9*(0.155) - sin(th2 + th3 - M_PI*(0.05))*(0.135) + cos(th2 + th3 + th4 - M_PI*(0.1194))*(0.184) + 1.47e-1;

    //ROS_INFO_STREAM(Point(tx, ty, tz, "m"));
    
    return Point(tx, ty, tz, "m");
}

float Robot::getYEndpointAngle() 
{
    float t2 = M_PI*(5.0/7.2e1);
    float t3 = t2 + joints[4].getAngle(); 
    float t4 = M_PI*(1.1e1/1.8e2);
    float t5 = t4 + joints[0].getAngle();
    float t6 = M_PI*(5.0/3.6e1);
    float t7 = t6 + joints[1].getAngle();
    float t8 = sin(t5);
    float t9 = M_PI*(1.4e1/4.5e1);
    float t10 = t9 + joints[2].getAngle();
    float t11 = cos(t7);
    float t12 = cos(t5);
    float t13 = sin(t7);
    float t14 = M_PI*(3.1e1/7.2e1);
    float t15 = t14 + joints[3].getAngle();
    float t16 = cos(t10);
    float t18 = t12*t13;
    float t19 = t18;
    float t20 = sin(t10);
    float t26 = t11*t12;
    float t22 = -t26; 
    float t23 = sin(t3);
    float t24 = cos(t15);
    float t25 = t16*t19;
    float t29 = t16*t22;
    float t30 = t19*t20;
    float t31 = t29 + t30;
    float t33 = t20*t22;
    float t27 = t25 - t33;
    float t28 = sin(t15);
    float t32 = cos(t3);
    float t36 = t24*t31;
    float t37 = t27*t28;
    
    float R11 = t32*(t36 + t37) - t8*t23;
    
    return acos(R11);
}

void Robot::computeEndpoint()
{
    this->computeEnpointPosition();
    this->computeEnpointOrientation();
}

// using derivative
void Robot::computeEndpointVelocities()
{
    float delay = joints[0].impedance_control.getTimeDelay().toSec();
    endpoint.velocities.setPositionX((endpoint.pose.getPosition().x - old_endpoint.pose.getPosition().x) / delay);
    endpoint.velocities.setPositionY((endpoint.pose.getPosition().y - old_endpoint.pose.getPosition().y) / delay);
    endpoint.velocities.setPositionZ((endpoint.pose.getPosition().z - old_endpoint.pose.getPosition().z) / delay);
    endpoint.velocities.setOrientationX((endpoint.pose.getOrientation().x - old_endpoint.pose.getOrientation().x) / delay);
    endpoint.velocities.setOrientationY((endpoint.pose.getOrientation().y - old_endpoint.pose.getOrientation().y) / delay);
    endpoint.velocities.setOrientationZ((endpoint.pose.getOrientation().z - old_endpoint.pose.getOrientation().z) / delay);
}

Eigen::VectorXf Robot::computeJointTorquesFromWrench(Pose force_torque)
{
    /*
    float torques[jacobian.actuated_joints];
    for (int act_jnt = 0; act_jnt < jacobian.actuated_joints; act_jnt++)
    {
        (*joints + act_jnt) = 0;
        for (int dof = 0; dof < jacobian.dof; dof++)
        {
            torques[act_jnt] += jacobian.t_matrix[act_jnt][dof]*force_torque[dof];
        }
    }
    */
    return jacobian.t_matrix * force_torque.getPoseVector();
}

Eigen::VectorXf Robot::computeJointVelocitiesFromEndpointVelocity(Pose endpoint_velocity)
{

    return jacobian.inv_matrix * endpoint_velocity.getPoseVector();
}

void Robot::setInputError(Eigen::VectorXf error)
{
    input_err = error;
}

void Robot::updateJacobianTranspose()
{
    jacobian.computeTransposeMatrix(joints);
}

void Robot::updateJacobianInverse()
{
    jacobian.computeInverseMatrix(joints);
}

void Robot::updateJacobian()
{
    jacobian.computeMatrix(joints);
}

void Robot::updateTimeSample() 
{
    joints[0].impedance_control.setLastTimeSample(joints[0].impedance_control.getTimeSample());
    joints[0].impedance_control.setCurrentTimeSample();
}

void Robot::updateJointData(int i, float th, float v_th = 0, float e_th = 0)
{
    //ROS_INFO_STREAM_THROTTLE(0.05, "Joint update");
    if (i < nb_joints)
    {
        joints[i].angle = th;
        joints[i].angular_velocity = v_th;
        joints[i].effort = e_th;
    }
    else
    {
        std::cout << "Joint index is out of bounds\n";
    }
}

void Robot::computeVelocityCollaborativeCmd()
{
    int i = 0;
    updateTimeSample();
    //ROS_INFO_STREAM_THROTTLE(0.2, "delay: " << joints[0].impedance_control.getTimeDelay() << "\n");
    for (int jnt = 0; jnt < nb_joints; jnt++)
    {
        if(actuated_joints_table[jnt])
        {
            velocities_cmd_msg.velocities[i].value = joints[jnt].impedance_control.compute(input_err(i, 0), jointLimitReached(jnt));
            //ROS_INFO_STREAM_THROTTLE(0.2, "torque error " << i << " : " << input_err(i, 0) << "\n");
            //ROS_INFO_STREAM_THROTTLE(0.2, "vel cmd" << velocities_cmd_msg.velocities[i].value << "\n");
           
            if (abs(velocities_cmd_msg.velocities[i].value) > joints[jnt].maxVelocity())
            {
                joints[jnt].impedance_control.antiWindup(joints[jnt].maxVelocity()*sign(velocities_cmd_msg.velocities[i].value), velocities_cmd_msg.velocities[i].value);
                velocities_cmd_msg.velocities[i].value = joints[jnt].maxVelocity()*sign(velocities_cmd_msg.velocities[i].value);
            }
            i++;
        }
        
    }
    
}

void Robot::computeNullspaceCollaborativeCmd(const float x0, const float Fz, const float Fr, const std::vector <float> q_i0)
{
    Eigen::Matrix<float, 3, 1> cartesian_cmd;
    Eigen::Matrix<float, 3, 1> joint_ctrl;
    bool wind_up = false;
    bool limit_reached = false;
    
    updateTimeSample();
    
    float z = endpoint.pose.getPosition().z;
    
    if( z > endpoint_limits[1].pose.getPosition().z || 
        z < endpoint_limits[0].pose.getPosition().z ) 
    {
        limit_reached = true;
        ROS_WARN_THROTTLE(0.2, "Z axis limit reached");
    }
    
    cartesian_cmd(0,0) = Kx * (x0 - endpoint.pose.getPosition().x);
    cartesian_cmd(1,0) = endpoint.cartesian_control.compute(Fr + Fz, false); //endpointLimitReached(2) 
    cartesian_cmd(2,0) = 0;
    
    //ROS_INFO_STREAM_THROTTLE(0.05, "Z ctrl: " << cartesian_cmd(1,0));
    //ROS_INFO_STREAM_THROTTLE(0.05, "X ctrl: " << cartesian_cmd(0,0));
    
    // Nullspace subtask in joint space
    joint_ctrl(0,0) = q_i0[0] - joints[1].angle;
    joint_ctrl(1,0) = q_i0[1] - joints[2].angle;
    joint_ctrl(2,0) = q_i0[2] - joints[3].angle;
 
    joint_ctrl = Kq*zNullSpaceProjector()*joint_ctrl;
    
    //ROS_INFO_STREAM_THROTTLE(0.05, "projector: \n: " << joint_ctrl << '\n');
    
    //jacobian must be defined as follow x,z,ry (robot in a 2D plane)
    joint_ctrl += jacobian.inv_matrix * cartesian_cmd;
    
    //ROS_INFO_STREAM_THROTTLE(0.05, "all: \n: " << joint_ctrl << '\n');
    
    for (int i = 0; i <3; i++)
    {
    	if( fabs(joint_ctrl(i,0)) > joints[i + 1].maxVelocity() )
    	{
            //joint_ctrl(i,0) = sign(joint_ctrl(i,0))*joints[i + 1].maxVelocity();
            wind_up = true;
        }
        velocities_cmd_msg.velocities[i].value = joint_ctrl(i,0);
    }
    if(wind_up)
    {
        
        //endpoint.cartesian_control.antiWindup(cartesian_cmd(1,0), Fz + Fr);
        //ROS_WARN("Windup !");
    }
}

Eigen::Matrix<float, 3, 3> Robot::zNullSpaceProjector()
{
    // extract jacobian for the task in z (vector)
    Eigen::Matrix<float, 3, 1> jac_z_task_t = jacobian.t_matrix.block<3,1>(0,2);
    
    return Eigen::Matrix<float, 3, 3>::Identity() - jac_z_task_t * ( jac_z_task_t * ( jac_z_task_t.transpose() * jac_z_task_t ).inverse() ).transpose();
    
    //Id - J_1^T*(J_1^#)^T //with superscripts # being pseudo inverse, and T transpose
}

void Robot::sendPositionCmd(const brics_actuator::JointPositions pos_vect)
{
    positions_cmd_msg = pos_vect;
}

void Robot::sendPositionCmd(const std::vector <float> joint_pos, const std::vector <std::string> joint_names)
{
    int cmd_pos_size = joint_pos.size();
    int cmd_name_size = joint_names.size();
    if (cmd_pos_size != cmd_name_size || cmd_pos_size > nb_joints || cmd_name_size > nb_joints)
    {
        ROS_WARN_STREAM("Error in sendPositionCmd input dimensions, " << cmd_pos_size << " positions given and " << cmd_name_size << " joint names given, while " << nb_joints << " are defined (max)");
    }
    else
    {
        // TODO
    }
}

void Robot::publishVelocitiesCmd()
{
    pub_vel_cmd_msg.publish(velocities_cmd_msg);
}

void Robot::publishPositionsCmd()
{
    pub_pos_cmd_msg.publish(positions_cmd_msg);
}

/*************************
 * VIRTUAL MECHANISM (VM)
 *************************/

VirtualMechanism::VirtualMechanism(float K[6], float B[6], float I[6], Endpoint ep, bool lim = false, std::vector<Endpoint> limits_e = std::vector<Endpoint>())
{
    equilibrium = ep;
    
    for (int i = 0; i < 6; i++)
    {
        this->K[i] = K[i];
        this->B[i] = B[i];
        this->I[i] = I[i];
    }
    
    if (lim)
    {
        if (limits_e.size() > 2)
        {
            ROS_WARN_STREAM("A limit vector with more than 2 elements was given while initializing virtual mechanism, only the first two elements will be used");
            for (int i = 0; i < 2; i++)
            {
                epsilon_limits.push_back(limits_e[i]);
            }
        }
        else if (limits_e.size() < 1)
        {
            ROS_WARN_STREAM("An empty limit vector was given while initializing virtual mechanism, no limits will be used");
            isLimited = lim;
        }
        else if (limits_e.size() == 1)
        {
            epsilon_limits.push_back(limits_e[0]);
            epsilon_limits.push_back(limits_e[0]);
        }
        else if (limits_e.size() == 2)
        {
            for (int i = 0; i < 2; i++)
            {
                epsilon_limits.push_back(limits_e[i]);
            }
        }
        //ROS_INFO_STREAM("First limit is:\n" << limits_e[0].getPose().getPoseVector() << "\nSecond limit is:\n" << limits_e[1].getPose().getPoseVector());
    }
}

void VirtualMechanism::setLimits(const Endpoint lim[])
{
    
}

Pose VirtualMechanism::trajectoryFixture(const Endpoint ep)
{
    Pose tmp;
    return tmp;
    // TODO
}

Pose VirtualMechanism::verticalXLineFixture(const float x, const float ry, const float vx, const float wy)
{
    Pose tmp(Point(0,0,0,"N"), Point(0,0,0,"N m"));
    float dx = x - equilibrium.getPose().getPosition().x;
    float dry = ry - equilibrium.getPose().getOrientation().y;
    float abs_dx = fabs(dx);
    float x2 = epsilon_limits[1].getPose().getPosition().x;
    float x1 = epsilon_limits[0].getPose().getPosition().x;
    
    //ROS_INFO_STREAM_THROTTLE(0.2, "dx: " << dx << ",\ndry: " << dry << ",\nabs(dx): " << abs_dx << ",\nry: " << ry);
    
    if(isLimited)
    {
        if(abs_dx >= x2)
        {
            return tmp; // no virtual mechanism force
        }
        else if (abs_dx <= x1)
        {
            tmp.setPositionX(-dx*K[0] - vx*B[0]);
            tmp.setOrientationY(dry*K[4] + wy*B[4]);
        }
        else
        {
            float alpha = (-abs_dx + x2)/(x2 - x1);
            
            tmp.setPositionX((-dx*K[0] - vx*B[0])*alpha);
            tmp.setOrientationY((dry*K[4] + wy*B[4])*alpha);
        }
    }
    else
    {
        tmp.setPositionX(-dx*K[0] - vx*B[0]);
        tmp.setOrientationY(dry*K[4] + wy*B[4]);
    }
    return tmp;
}

Pose VirtualMechanism::verticalXLineFixture(const float x, const float vx)
{
    Pose tmp(Point(0,0,0,"N"), Point(0,0,0,"N m"));
    float dx = x - equilibrium.getPose().getPosition().x;
    float abs_dx = fabs(dx);
    float x2 = epsilon_limits[1].getPose().getPosition().x;
    float x1 = epsilon_limits[0].getPose().getPosition().x;
    
    //ROS_INFO_STREAM_THROTTLE(0.2, "dx: " << dx << ",\nabs(dx): " << abs_dx);
    
    if(isLimited)
    {
        if(abs_dx >= x2)
        {
            return tmp; // no virtual mechanism force
        }
        else if (abs_dx <= x1)
        {
            tmp.setPositionX(-dx*K[0] - vx*B[0]);
        }
        else
        {
            float alpha = (-abs_dx + x2)/(x2 - x1);
            
            tmp.setPositionX((-dx*K[0] - vx*B[0])*alpha);
        }
    }
    else
    {
        tmp.setPositionX(-dx*K[0] - vx*B[0]);;
    }
    return tmp;
}
