#ifndef ROBOT_H
#define ROBOT_H

#include <string.h>
#include <math.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <brics_actuator/JointTorques.h>
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
        static ros::Duration getTimeDelay() {return t - t_old;};
    
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
            
            cartesian_control = val.cartesian_control;
        };
   
    protected:
        Pose pose;
        Pose velocities;
        Pose accelerations;
        Pose forces;
        
        PID cartesian_control; // TODO: change to pid vector
};


class Jacobian
{
    friend class Robot;
    public:
    	Jacobian();
        Jacobian(int dof, int nb_joints, bool onlyM, bool onlyT, bool onlyI);
    
        // Eigen::MatrixXf = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
        Eigen::MatrixXf getTransposeMatrix() const;
        Eigen::MatrixXf getMatrix() const;
        
        void setTransposeMatrix();	// if Matrix is defined
        void setTransposeMatrix(float **t_jac);
        void setMatrix(float **jac);
    
        void computeTransposeMatrix(std::vector<Joint>);
        void computeInverseMatrix(std::vector<Joint>);
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
        Eigen::MatrixXf inv_matrix;
    private:
        void youBotTJoints234(const std::vector<Joint>);
        // transpose jacobian considering linear x, z and angular y
        void youBotTJoints234_3DOF(const std::vector<Joint>);
        void youBotInvJoints234(const std::vector<Joint> joints); // TODO pseudo inverse
        // inverse jacobian considering linear x, z and angular y, (3x3 matrix)
        void youBotInvJoints234_3DOF(const std::vector<Joint>);
};

class Robot
{
    public:
        //Robot() {;};
        Robot(const std::vector<Joint>, const bool*, const Jacobian, brics_actuator::JointPositions, ros::NodeHandle*);
        
        void dispRobot() {std::cout << "Nb jnts: " << nb_joints << '\n';};
        
        bool jointLimitReached(const int joint_nb);
        bool endpointLimitReached(const int dof_nb);
    
        Endpoint getEndpoint() const {return endpoint;};
        void setEndpoint(const Endpoint ep) {endpoint = ep;};
    
        Jacobian getJacobian() const {return jacobian;};
        //void setJacobian(const Jacobian jac) {jacobian = jac;}; //TODO: overload = operator
    
    	void setEndpointPID(PID pid) {endpoint.cartesian_control = pid;};
    	void setEndpointLimits(std::vector<Endpoint> lim) {endpoint_limits = lim;};
    
        void computeEnpointPosition();
        void computeEnpointOrientation(const bool onlyX, const bool onlyY, const bool onlyZ);
        void computeEndpoint();
        void computeEndpointVelocities();
        
        Eigen::VectorXf computeJointTorquesFromWrench(Pose);
        Eigen::VectorXf computeJointVelocitiesFromEndpointVelocity(Pose);
        void setInputError(Eigen::VectorXf);
        void setNullspaceCtrlGains(float kx, float kq) {Kx = kx; Kq = kq;};

        // compute jacobian matrices
        void updateJacobianTranspose();
        void updateJacobianInverse();
        void updateJacobian();
    
    	// set time
    	void updateTimeSample();
    	
    	// update joint data
    	void updateJointData(int, float, float, float);
    	std::vector<Joint> getJoints() {return joints;};
    	
        // compute velocity cmd msgs
        void computeVelocityCollaborativeCmd();
        void computeNullspaceCollaborativeCmd(const float, const float, const float, const std::vector <float>);
        void setPositionCmd(const brics_actuator::JointPositions);
        void setPositionCmd(const std::vector <float>, const std::vector <std::string>);
        
        void setTorqueDisturbanceCmd(std::vector<float>);
    
    	void publishTorquesCmd();
        void publishVelocitiesCmd();
        void publishPositionsCmd();

    private:
        float getYEndpointAngle();
        Point getEndpointPosition();
        // nullspace projector for z (using jacobian transpose, 3rd col must be Z)
        Eigen::Matrix<float, 3, 3> zNullSpaceProjector();
    
        int nb_joints;
        std::vector<Joint> joints;
        std::vector<bool> actuated_joints_table;
    
        Endpoint endpoint;
        Endpoint old_endpoint;
        std::vector<Endpoint> endpoint_limits; // lower limits should be first, then upper's
    
        Jacobian jacobian;

        float Kx; //gain for nullspace ctrl (equivalent to virtual guide in x)
        float Kq; //gain for nullspace ctrl (joint pose gain)

        Eigen::VectorXf input_err; // t_ref - t_feedback
        brics_actuator::JointTorques torques_cmd_msg;
        brics_actuator::JointVelocities velocities_cmd_msg;
        brics_actuator::JointPositions positions_cmd_msg;
    
    	ros::Publisher pub_tor_cmd_msg;
        ros::Publisher pub_vel_cmd_msg;
        ros::Publisher pub_pos_cmd_msg;
};


class VirtualMechanism
{
    public:
        VirtualMechanism(float*, float*, float*, Endpoint, bool, std::vector<Endpoint>);
    
        void setLimits(const Endpoint*);
    
        Pose trajectoryFixture(const Endpoint);
        Pose verticalXLineFixture(const float x, const float ry, const float vx, const float wy);
        Pose verticalXLineFixture(const float x, const float vx); // without orientation

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

#endif
