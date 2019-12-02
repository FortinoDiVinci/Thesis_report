#ifndef ROBOT_H
#define ROBOT_H

#include <string.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointPositions.h>

#include "utils/point.h" 
#include "utils/utils.h"


class PID
{
    public:
    	PID();
        PID(float Kp, float Ki, float Kd);
        // time setters & getters
        static void setLastTimeSample();
        static void setLastTimeSample(const ros::Time);
        static void setCurrentTimeSample();
        static void setCurrentTimeSample(const ros::Time);
        static ros::Time getTimeSample() {return t;};
    
        void antiWindup(const float maxVal, const float error);
    
        float compute(const float error, const bool limitReached);

        PID& operator = (const PID &val) {
            Kp = val.Kp;
            Ki = val.Ki;
            Kd = val.Kd;
            error_sum = val.error_sum;
            t = val.t;
            t_old = val.t_old;
        };
    
        float Kp;
        float Ki;
        float Kd;
    
    protected:
        float error_sum;
        static ros::Time t;
        static ros::Time t_old;
};

class Joint
{
    friend class Robot; 
    public:
        //Joint();
        Joint(float max_th, float min_th, PID val);
    
        // setters & getters
        float getAngle() const;
        float getAngularVelocity() const;
        float getEffort() const;
    
        void setAngle(const float);
        void setAngularVelocity(const float);
        void setEffort(const float);
    
        float maxAngle() const {return max_angle;};
        float minAngle() const {return min_angle;};
        float maxVelocity() const {return max_velocity;};
        
        void setMaxVelocity(float value) {max_velocity = value;};

        Joint& operator = (const Joint &val);

    protected:
        float angle;
        float angular_velocity;
        float effort;
        
        PID impedance_control;
        
    private:
        float max_angle;
        float min_angle;
        
        float max_velocity;
};


class Endpoint
{
    friend class Robot;
    public:
        Endpoint();
    
        Pose getPose() const {return pose;};
        Pose getVelocities() const {return velocities;};
        Pose getAccelerations() const {return accelerations;};
        Pose getForces() const {return forces;};
    
        void setEndpointPose(const Pose p) {pose = p;};
        void setEndpointVelocities(const Pose v) {velocities = v;};
        void setEndpointAccelerations(const Pose a) {accelerations = a;};
        void setEndpointForces(const Pose f) {forces = f;};
    
        Endpoint& operator = (const Endpoint &val){
            pose = val.pose;
            velocities = val.velocities;
            accelerations = val.accelerations;
            forces = val.forces;
        };
   
    protected:
        Pose pose;
        Pose velocities;
        Pose accelerations;
        Pose forces;
};


class Jacobian
{
    friend class Robot;
    public:
    	Jacobian();
        Jacobian(int dof, int nb_joints, bool onlyM, bool onlyT);
    
        // Eigen::MatrixXf = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
        Eigen::MatrixXf getTransposeMatrix() const;
        Eigen::MatrixXf getMatrix() const;
        
        void setTransposeMatrix();	// if Matrix is defined
        void setTransposeMatrix(float **t_jac);
        void setMatrix(float **jac);
    
        void computeTransposeMatrix(std::vector<Joint>);
        void computeMatrix(std::vector<Joint>);

        void dispTransposeMatrix();
        void dispMatrix();
    
        //void Jacobian::operator = (const Jacobian &val);
        
    protected:
        int degree_of_freedom;
        int actuated_joints;
    
        //float** matrix;
        //float** t_matrix;
        Eigen::MatrixXf matrix;
        Eigen::MatrixXf t_matrix;
    private:
        void youBotJoints234(const std::vector<Joint>);
};

class Robot
{
    public:
        //Robot() {;};
        Robot(const std::vector<Joint>, const bool*, const Jacobian, brics_actuator::JointVelocities, brics_actuator::JointPositions, ros::NodeHandle*);
        bool jointLimitReached(const int joint_nb);
    
        Endpoint getEndpoint() const {return endpoint;};
        void setEndpoint(const Endpoint ep) {endpoint = ep;};
    
        Jacobian getJacobian() const {return jacobian;};
        //void setJacobian(const Jacobian jac) {jacobian = jac;}; //TODO: overload = operator
    
        void computeEnpointPosition();
        void computeEnpointOrientation(const bool onlyX, const bool onlyY, const bool onlyZ);
        void computeEndpoint();
        Eigen::VectorXf computeJointTorquesFromWrench(Pose);
        void setTorqueError(Eigen::VectorXf);

        // compute jacobian matrices
        void updateJacobianTranspose();
        void updateJacobian();
    
    	// set time
    	void updateTimeSample();
    	
    	// update joint data
    	void updateJointData(int, float, float, float);
    	
        // compute velocity cmd msgs
        void computeVelocityCollaborativeCmd();
        void sendPositionCmd(const brics_actuator::JointPositions);
        void sendPositionCmd(const std::vector <float>, const std::vector <std::string>);
    
        void publishVelocitiesCmd();
        void publishPositionsCmd();
    
    private:
        float getYEndpointAngle();
        Point getEndpointPosition();
    
        int nb_joints;
        std::vector<Joint> joints;
        std::vector<bool> actuated_joints_table;
    
        Endpoint endpoint;
    
        Jacobian jacobian;
    
        Eigen::VectorXf torque_ref;
        Pose wrench_feedback;
        Eigen::VectorXf torque_feedback;
        Eigen::VectorXf torque_err; // t_ref - t_feedback
        brics_actuator::JointVelocities velocities_cmd_msg;
        brics_actuator::JointPositions positions_cmd_msg;
    
        ros::Publisher pub_vel_cmd_msg;
        ros::Publisher pub_pos_cmd_msg;
};


class VirtualMechanism
{
    public:
        VirtualMechanism(float*, float*, float*, Endpoint, bool, std::vector<Endpoint>);
    
        void setLimits(const Endpoint*);
    
        Pose trajectoryFixture(const Endpoint);
        Pose verticalXLineFixture(const float x, const float ry);

    private:
        int DOF;    // degree of freedom
        // no interaction effect are considered here
        // otherwise, K, B, I should be change to 6x6 matrices
        float K[6]; // stiffness
        float B[6]; // damping
        float I[6]; // inertia
    
        Endpoint equilibrium;
        std::vector<Endpoint> epsilon_limits;
        bool isLimited;
};



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
    cmd = Kp * error + Ki * error_sum * (t_old - t).toSec();
    
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

Jacobian::Jacobian(int dof, int nb_joints, bool onlyM = true, bool onlyT = false)
{
    if (onlyM && onlyT)
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
    else
    {
        matrix.resize(degree_of_freedom, actuated_joints);
        t_matrix.resize(actuated_joints, degree_of_freedom);
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
        std::cout << "error\n"; //TODO
        return;
    }

    switch (actuated_joints) {
        case 3:
            if (degree_of_freedom == 6)
            {
                youBotJoints234(joints);
            }
            else if (degree_of_freedom == 3)
            {
                // TODO
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
        std::cout << "error\n"; //TODO
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

void Jacobian::youBotJoints234(const std::vector<Joint> joints) 
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
 
Robot::Robot(const std::vector<Joint> jnt, const bool act_jnts_tab[], const Jacobian jac, brics_actuator::JointVelocities vel_msg, brics_actuator::JointPositions pos_msg, ros::NodeHandle *n)
{
    nb_joints = jnt.size();
    joints = jnt;
    
    for (int i = 0; i < nb_joints; i++)
    {
        actuated_joints_table.push_back(act_jnts_tab[i]);
    }
    
    jacobian = jac;
    
    torque_ref.resize(jacobian.actuated_joints, 1);
    torque_feedback.resize(jacobian.actuated_joints, 1);
    velocities_cmd_msg = vel_msg;
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

// endpoint
void Robot::computeEnpointPosition()
{
    switch(jacobian.actuated_joints) 
    {
        case 3:
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

void Robot::setTorqueError(Eigen::VectorXf t_err)
{
    torque_err = t_err;
}

void Robot::updateJacobianTranspose()
{
    jacobian.computeTransposeMatrix(joints);
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
    for (int jnt = 0; jnt < nb_joints; jnt++)
    {
        if(actuated_joints_table[jnt])
        {
            velocities_cmd_msg.velocities[i].value = joints[jnt].impedance_control.compute(torque_err(i, 0), jointLimitReached(jnt));
            
            if (abs(velocities_cmd_msg.velocities[i].value) > joints[jnt].maxAngle())
            {
                joints[jnt].impedance_control.antiWindup(joints[jnt].maxAngle()*sign(velocities_cmd_msg.velocities[i].value), velocities_cmd_msg.velocities[i].value);
                velocities_cmd_msg.velocities[i].value = joints[jnt].maxAngle()*sign(velocities_cmd_msg.velocities[i].value);
            }
            i++;
        }
        
    }
    
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
        std::cout << "Error in sendPositionCmd input dimensions, " << cmd_pos_size << " positions given and " << cmd_name_size << " joint names given, while " << nb_joints << " are defined (max)";
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
            std::cout << "A limit vector with more than 2 elements was given while initializing virtual mechanism, only the first two elements will be used";
            for (int i = 0; i < 2; i++)
            {
                epsilon_limits.push_back(limits_e[i]);
            }
        }
        else if (limits_e.size() < 1)
        {
            std::cout << "An empty limit vector was given while initializing virtual mechanism, no limits will be used";
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

Pose VirtualMechanism::verticalXLineFixture(const float x, const float ry)
{
    Pose tmp;
    float dx = x - equilibrium.getPose().getPosition().x;
    float dry = ry - equilibrium.getVelocities().getOrientation().y;
    float abs_dx = abs(dx);
    
    if(isLimited)
    {
        if(abs_dx >= epsilon_limits[1].getPose().getPosition().x)
        {
            return tmp; // no virtual mechanism force
        }
        else if (abs_dx <= epsilon_limits[0].getPose().getPosition().x)
        {
            tmp.setPositionX(-abs_dx*K[0] - dry*B[0]);
            tmp.setOrientationY(-abs_dx*K[4] - dry*B[4]);
        }
        else
        {
            float alpha = (-abs_dx + epsilon_limits[1].getPose().getPosition().x)/(epsilon_limits[1].getPose().getPosition().x - epsilon_limits[0].getPose().getPosition().x);
            tmp.setPositionX((-abs_dx*K[0] - dry*B[0])*alpha);
            tmp.setOrientationY((-abs_dx*K[4] - dry*B[4])*alpha);
        }
    }
    else
    {
        tmp.setPositionX(-abs_dx*K[0] - dry*B[0]);
        tmp.setOrientationY(-abs_dx*K[4] - dry*B[4]);
    }
    return tmp;
}


#endif
