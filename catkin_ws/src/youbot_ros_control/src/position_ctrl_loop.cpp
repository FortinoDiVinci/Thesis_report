#include <ros/ros.h>
#include <ros/package.h>
#include <string.h>
#include <math.h>

#include <signal.h>
#include <time.h>

#include <brics_actuator/JointPositions.h>
#include "youbot_driver_ros_interface/YouBotPID.h"

//#include "robot.cpp"

/**********    
 * MACROS *
 **********/

#define NB_JOINT_YOUBOT         5
#define NB_ACTUATED_JOINTS      3
#define DOF                     3
#define REF_FRAME_ID            "base_link"
#define SINE_HZ_FREQ            1.0
 
/**************
 * GLOBAL VAR *
 **************/ 
 
const float TH_MAX[NB_JOINT_YOUBOT] = {5.7401, 25179, -0.1157, 3.3292, 5.5415}; //rad
const float TH_MIN[NB_JOINT_YOUBOT] = {1.101e-1, 1.101e-1, -4.9266, 1.221e-1, 2.106e-1}; //rad
const float TH_ON_D[NB_JOINT_YOUBOT] = {169, 65, -146, 102-90, 167.5-110}; //degree
const float TH_OFF_D[NB_JOINT_YOUBOT] = {0.011, 0.011, -0.016, 0.023, 0.12}; //degree
const bool ACTUATED_JOINTS[NB_JOINT_YOUBOT] = {false, true, true, true, false};
static const int KP_INITIAL_GAIN[NB_JOINT_YOUBOT] = {2,3,3,3,2};

//Robot* kuka_youBot;
float joint_position_set_point[NB_JOINT_YOUBOT] = {0., 0., 0., 0., 0.};

sig_atomic_t volatile g_request_shutdown = 0;
 
/*************
 *  CLASSES  *
 *************/
 
class PositionPID
{
    public:
        PositionPID(ros::NodeHandle *n, const int nb_joints);
        void publishConfig();
        void setInitialGains();
        std::vector<int> getKp() {return Kp;};
        std::vector<int> getKi() {return Ki;};
        std::vector<int> getKd() {return Kd;};
        // TODO: vector size should be equal to nb_joints
        void setKp(std::vector<int> Kp) {this->Kp = Kp;};
        void setKi(std::vector<int> Ki) {this->Ki = Ki;};
        void setKd(std::vector<int> Kd) {this->Kd = Kd;};
          
    private:
        std::vector<int> Kp;
        std::vector<int> Ki;
        std::vector<int> Kd;
        youbot_driver_ros_interface::YouBotPID youBot_pid_msg;
        ros::Publisher pub_pid_config;
        
        void checkPIDConfig();
};
 
/*************
 * FUNCTIONS *
 *************/

// Inverse kinematics
std::vector<double> youBotIk3jnts(const double x, const double z, const double th);
// ROS position message generation and publication
void initPositionMsg(brics_actuator::JointPositions* positions_cmd_msg, const int nb_joints);
void initPositionPublisher(ros::NodeHandle *n, ros::Publisher *pub_pos_cmd);
void publishPositionCmd(const brics_actuator::JointPositions positions_cmd_msg, ros::Publisher *pub_pos_cmd);
void initPosition(brics_actuator::JointPositions* positions_cmd_msg, ros::Publisher *pub_pos_cmd, std::vector<double>);
//void configPositionPIDGains(ros::NodeHandle *n, const unsigned long int Kp[], const unsigned long int Ki[], const unsigned long int Kd[], const int nb_joints);
// Utils
void getROSServerParam(ros::NodeHandle *n, int *rate,  PositionPID *pos_pid, double *f, double *x0, double *z0, double *z_mag);
void getROSServerParam(ros::NodeHandle *n, int *rate, double *f, double *x0, double *z0, double *z_mag);
void getROSServerParam(ros::NodeHandle *n, PositionPID *pos_pid);
void sigIntHandler(int sig);

/********
 * MAIN *
 ********/

int main(int argc, char** argv)
{
    ROS_INFO("start\n");
    
    ros::init(argc, argv, "position_control_loop", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    ros::NodeHandle n1("~");
    signal(SIGINT, sigIntHandler);
	
	ros::Publisher pub_position_cmd;
	brics_actuator::JointPositions positions_cmd_msg;
	
    std::vector< std::vector<double> > thetas;
    std::vector<double> z; // virtual trajectory
    double x, z0, sine_freq, sine_magn;    
    int frequency;
    
    PositionPID joint_pid_config(&n, NB_JOINT_YOUBOT);
    
    getROSServerParam(&n1, &frequency, &sine_freq, &x, &z0, &sine_magn);
     
    //ROS_INFO_STREAM("rate: " << frequency << ", sine_freq: " << sine_freq << ", x0: " << x << ", z0: " << z0 << ", magn: " << sine_magn);
    
    int nb_samples = int(std::floor(frequency/sine_freq));
    
    static const double arr[] = {0.,0.,0.};
    std::vector<double> dummy_vector(arr, arr + sizeof(arr) / sizeof(arr[0]) );
    thetas.resize(nb_samples, dummy_vector);
    z.resize(nb_samples, 0.);
    
    for (int i = 0; i < nb_samples; i++)
    {
        //z[i] = 0.274 + 0.04*sin(2*M_PI*SINE_HZ_FREQ);
        z[i] = z0 + sine_magn*sin(2*M_PI*sine_freq*i/frequency);
        thetas[i] = youBotIk3jnts(x, z[i], M_PI);   
    }
    
    //thetas[0] = youBotIk3jnts(x, z[0], M_PI); 
    //ROS_INFO_STREAM("th1: " << thetas[0][0] << ", th2: " << thetas[0][1] << ", th3: " << thetas[0][2]);     
    //ROS_INFO_STREAM("th1: " << thetas[100][0] << ", th2: " << thetas[100][1] << ", th3: " << thetas[100][2]);       
            
	initPositionMsg(&positions_cmd_msg, NB_JOINT_YOUBOT);
	initPositionPublisher(&n, &pub_position_cmd);
	usleep(1.0*1e6); // this delay seems necessary..
	joint_pid_config.setInitialGains();
	initPosition(&positions_cmd_msg, &pub_position_cmd, thetas[0]);
	usleep(5.0*1e6); // waits 5 seconds for the end of the movement
	
	// new position controller gain
	getROSServerParam(&n1, &joint_pid_config);
	joint_pid_config.publishConfig();
	
    ros::Rate rate(frequency);
    int ii = 0;
    ros::Time t;
    
    while (!g_request_shutdown)
    {
        
		positions_cmd_msg.positions[1].value = thetas[ii][0];
		positions_cmd_msg.positions[2].value = thetas[ii][1];
		positions_cmd_msg.positions[3].value = thetas[ii][2];
		t = ros::Time::now();
        positions_cmd_msg.positions[1].timeStamp = t;
        positions_cmd_msg.positions[2].timeStamp = t;
        positions_cmd_msg.positions[3].timeStamp = t;
		publishPositionCmd(positions_cmd_msg, &pub_position_cmd);
        
        ii = (ii + 1)%nb_samples;
        
        ros::spinOnce();
        rate.sleep();
    }
     
    positions_cmd_msg.positions[1].value = thetas[0][0];
	positions_cmd_msg.positions[2].value = thetas[0][1];
	positions_cmd_msg.positions[3].value = thetas[0][2];
	publishPositionCmd(positions_cmd_msg, &pub_position_cmd);
	
}

/*************
 * FUNCTIONS *
 *************/

//   Inverse kinematic for the youBot, when only joint 2,3and4 are actuated
// A geometric method is used to solve the inverse kinematic
std::vector<double> youBotIk3jnts(const double x, const double z, const double th)
{
    // Constants
    double l1 = 0.155;
    double l2 = 0.135;
    double l3 = 0.181; //0.1625;//
    double x1 = 0.033;
    double z1 = 0.147;
    
    //double x3 = x - l3*cos(th);
    //double z3 = z - l3*sin(th);
    
    double B = z - l3*sin(th) - z1;
    double A = x - l3*cos(th) - x1;
    
    //ROS_INFO_STREAM("A: " << A << ", B: " << B);
    
    double alpha = atan2(B,A);
    double C = A/cos(alpha);
    
    // Law of cosines (Al Kashi's theorem)
    double a = acos( (l1*l1 + C*C - l2*l2)/(2*l1*C) );
    
    // double D = C*sin(a);
    
    std::vector<double> thetas;
    double th1 = alpha - a;
    double th2 = asin(C*sin(a)/l2);
    
    thetas.push_back( (65+90)*M_PI/180 - th1 );
    thetas.push_back( (-146)*M_PI/180 - th2 );
    
    //double th3 = (th - thetas[0] - thetas[1]);
    
    thetas.push_back( (102.5)*M_PI/180 - (th - th1 - th2) );
    
    //thetas[0] = alpha - a;
    //thetas[1] = asin(C*sin(a)/l2);
    //thetas[2] = (th  - 1.571) - thetas[0] - thetas[1];
    
    return thetas;
}

void initPositionMsg(brics_actuator::JointPositions* positions_cmd_msg, const int nb_joints) 
{
	positions_cmd_msg->positions.resize(nb_joints);
	
	for (int ii = 0; ii < nb_joints; ii++)
    {
		std::stringstream jointNameStream;
		jointNameStream << "" << ii + 1;        
		positions_cmd_msg->positions[ii].joint_uri = "arm_joint_" + jointNameStream.str();
		positions_cmd_msg->positions[ii].unit = "rad";
		positions_cmd_msg->positions[ii].value = 0.0;
    }
	
}

void initPositionPublisher(ros::NodeHandle *n, ros::Publisher *pub_pos_cmd) 
{
	std::string topic_name = "";
	topic_name = "arm_1/arm_controller/position_command";
    *pub_pos_cmd = n->advertise<brics_actuator::JointPositions>(topic_name, 1); 	
}

void publishPositionCmd(const brics_actuator::JointPositions positions_cmd_msg, ros::Publisher *pub_pos_cmd)
{
	pub_pos_cmd->publish(positions_cmd_msg);
}

void initPosition(brics_actuator::JointPositions* positions_cmd_msg, ros::Publisher *pub_pos_cmd, std::vector<double> thetas)
{
	for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++)
    {
        if (ii >= 1 && ii <= 3)
        {
            positions_cmd_msg->positions[ii].value = thetas[ii-1];
        }
        else
        {
		    positions_cmd_msg->positions[ii].value = TH_ON_D[ii] * M_PI/180;
	    }
		positions_cmd_msg->positions[ii].timeStamp = ros::Time::now();
	}
	publishPositionCmd(*positions_cmd_msg, pub_pos_cmd);
}

// PID

/*
void configPositionPIDGains(ros::NodeHandle *n, const unsigned long int Kp[], const unsigned long int Ki[], const unsigned long int Kd[], const int nb_joints)
{
	// youBot PID msg init
	youbot_driver_ros_interface::YouBotPID youBotPID;
	youBotPID.positions.resize(nb_joints);
	
	for (int i = 0; i < nb_joints; i++)
	{
		std::stringstream jointNameStream;
        jointNameStream << "" << i + 1;
        
		youBotPID.positions[i].timeStamp = ros::Time::now();
		youBotPID.positions[i].joint_uri = "arm_joint_" + jointNameStream.str();
		youBotPID.positions[i].P_value = Kp[i];
		youBotPID.positions[i].I_value = Ki[i];
		youBotPID.positions[i].D_value = Kd[i];
	}
	// publisher
	std::string topic_name = "";
	topic_name = "arm_controller/PID_reconfiguration";
	ros::Publisher pub_pid_conf;
    pub_pid_conf = n->advertise<youbot_driver_ros_interface::YouBotPID>(topic_name, 1);
	pub_pid_conf.publish(youBotPID);
} */

PositionPID::PositionPID(ros::NodeHandle *n, const int nb_joints)
{
    this->youBot_pid_msg.positions.resize(nb_joints);
    this->Kp.resize(nb_joints);
    this->Ki.resize(nb_joints);
    this->Kd.resize(nb_joints);
    
    for (int i = 0; i < nb_joints; i++)
	{
		std::stringstream jointNameStream;
        jointNameStream << "" << i + 1;        
		youBot_pid_msg.positions[i].timeStamp = ros::Time::now();
		youBot_pid_msg.positions[i].joint_uri = "arm_joint_" + jointNameStream.str();
	}
	
	// publisher
	std::string topic_name = "arm_1/arm_controller/PID_reconfiguration";
    this->pub_pid_config = n->advertise<youbot_driver_ros_interface::YouBotPID>(topic_name, 1);	
}

void PositionPID::publishConfig() 
{
    
    for (int i = 0; i < youBot_pid_msg.positions.size(); i++)
	{
	    this->youBot_pid_msg.positions[i].P_value = this->Kp[i];
		this->youBot_pid_msg.positions[i].I_value = this->Ki[i];
		this->youBot_pid_msg.positions[i].D_value = this->Kd[i];
		this->youBot_pid_msg.positions[i].timeStamp = ros::Time::now();
    }
    
    this->pub_pid_config.publish(youBot_pid_msg);
    
}

void PositionPID::setInitialGains()
{
    static const int arr[] = {0.,0.,0.,0.,0.};
    std::vector<int> null_vector(arr, arr + sizeof(arr) / sizeof(arr[0]) );
    std::vector<int> kp_init_vector(KP_INITIAL_GAIN, KP_INITIAL_GAIN + sizeof(KP_INITIAL_GAIN) / sizeof(KP_INITIAL_GAIN[0]) );
    
    this->setKp(kp_init_vector);
    this->setKi(null_vector);
    this->setKd(null_vector);
    
    this->publishConfig();
}

//

void getROSServerParam(ros::NodeHandle *n, int *rate,  PositionPID *pos_pid, double *f, double *x0, double *z0, double *z_mag)
{
    std::vector<int> dummy_vect;   
    n->getParam("Kp", dummy_vect);  
    //ROS_INFO_STREAM("dummy_vect[0]: " << dummy_vect[0]);
    pos_pid->setKp(dummy_vect);
    n->getParam("Ki", dummy_vect);
    pos_pid->setKi(dummy_vect);
    n->getParam("Kd", dummy_vect);
    pos_pid->setKd(dummy_vect);
    
    n->getParam("rate", *rate);
    n->getParam("f", *f);
    n->getParam("x0", *x0);
    n->getParam("z0", *z0);
    n->getParam("magnitude", *z_mag);
    
}

void getROSServerParam(ros::NodeHandle *n, int *rate, double *f, double *x0, double *z0, double *z_mag)
{
    
    n->getParam("rate", *rate);
    n->getParam("f", *f);
    n->getParam("x0", *x0);
    n->getParam("z0", *z0);
    n->getParam("magnitude", *z_mag);
    
}

void getROSServerParam(ros::NodeHandle *n, PositionPID *pos_pid)
{
    std::vector<int> dummy_vect;   
    n->getParam("Kp", dummy_vect);  
    pos_pid->setKp(dummy_vect);
    n->getParam("Ki", dummy_vect);
    pos_pid->setKi(dummy_vect);
    n->getParam("Kd", dummy_vect);
    pos_pid->setKd(dummy_vect);
}
// Signal interruption handler

void sigIntHandler(int sig)
{
    g_request_shutdown = 1;
}
