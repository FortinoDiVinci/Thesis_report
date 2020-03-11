#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iterator>
#include <algorithm>
#include <signal.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointValue.h>

#include "utils/utils.h"

/*************
 *   MACRO   *
 *************/

#define NB_JOINT_CTRL       3
#define NB_JOINT_YOUBOT     5

sig_atomic_t volatile g_request_shutdown = 0;

/*************
 *   CLASS   *
 *************/

class CSVReader
{

public:
    CSVReader(std::string filepath, std::string delm = ",") :
                path(filepath), delimeter(delm) {;};
                
    void setFileName(std::string n_file_name) {file_name = n_file_name;};
    
    //Function to fetch data from a CSV File
    std::vector<std::vector<float> > getData();
    
private:
    std::vector<float> stringVector2FloatVector(const std::vector<std::string>& string_vector);    
    
    std::string path;
    std::string file_name;
    std::string delimeter;
    
};

class JointTrajectory
{

friend class Trajectory;

protected:
   std::string name;
   std::vector<float> angles;
public:
   JointTrajectory(std::string joint_name) :
                name(joint_name) {;}; 
   
   void setJointName(std::string n_name) {name = n_name;};
   void setJointTrajectory(std::vector<float> n_angles) {angles = n_angles;};
   
};

class Trajectory
{
    // global variable "g_request_shutdown" is used for interruption
private:
    std::vector<JointTrajectory> trajectory;
    ros::Publisher pub;
    float frequency;
public:
    Trajectory(ros::NodeHandle*, float rate = 1000);
    
    void setRate(float rate) {frequency = rate;};
    void setTrajectory(std::vector<JointTrajectory> jnts_tj) {trajectory = jnts_tj;};
    
    void addJointTrajectory(JointTrajectory jnt_tj) {trajectory.push_back(jnt_tj);};
    
    void sendTrajectory();

};

/*************
 * FUNCTIONS *
 *************/

void sigIntHandler(int sig);
brics_actuator::JointPositions initPositionsCmd();


/********
 * MAIN *
 ********/

int main(int argc, char** argv)
{

    ros::init(argc, argv, "trajectory_generator", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    
    signal(SIGINT, sigIntHandler);
    
    std::string path = ros::package::getPath("youbot_ros_control");
    
    Trajectory trajectory(&nh);
    
    // this version is for joint 2, 3 and 4 ctrl
    
    ROS_INFO("CSV Reader for trajectory generator");
    
    CSVReader csv_reader(path);
    std::string file_base_name = "traj_20s_110320_2_j";
    
    for(int i = 1; i < NB_JOINT_CTRL + 1; i++) // joint ctrl are 2,3&4
    {
        std::stringstream file_name_stream;
        file_name_stream << file_base_name << i + 1 << ".txt";
        csv_reader.setFileName(file_name_stream.str());

        std::stringstream joint_name_stream;
        joint_name_stream << "arm_joint_" << i + 1;
        JointTrajectory jt(joint_name_stream.str()); 

        jt.setJointTrajectory(csv_reader.getData()[0]);
        
        trajectory.addJointTrajectory(jt);
    }

    trajectory.sendTrajectory();
    
    ros::spinOnce(); 
}

std::vector<float> CSVReader::stringVector2FloatVector(const std::vector<std::string>& string_vector)
{
    std::vector<float> float_vector(string_vector.size());

    //std::transform(string_vector.begin(), string_vector.end(), std::back_inserter(float_vector), boost::lexical_cast<float>); 
    
    for(int i = 0; i < string_vector.size(); i++)
    {
        sscanf(string_vector.at(i).c_str(), "%f", &float_vector.at(i));
    }

    return float_vector;
}

std::vector<std::vector<float> > CSVReader::getData()
{
    std::ifstream file((path + "/data/20_03_11/" + file_name).c_str());
 
    //ROS_INFO_STREAM(file);
    ROS_INFO_STREAM((path + "/data/20_03_11/" + file_name).c_str());
 
    std::vector<std::vector<float> > temp;

    std::string line;
    // Iterate through each line and split the content using delimeter
    try
    {
        while (getline(file, line))
        {
            std::vector<std::string> vec;
            boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
            
            temp.push_back(stringVector2FloatVector(vec));
        }
    }
    catch(std::ios_base::failure err)
    {
        ROS_ERROR_STREAM(err.what());    
    }
    // Close the File
    file.close();
 
    //ROS_INFO_STREAM("Data size: " << data_list.size());
 
    std::vector<std::vector<float> > data_list(temp[0].size(), std::vector<float>(temp.size()));
 
    for(int i = 0; i < temp[0].size(); i++)
    {
        for(int j = 0; j < temp.size(); j++)
        {
            data_list[i][j] = temp[j][i];
        }
    }
     return data_list;
}

//

Trajectory::Trajectory(ros::NodeHandle* n, float rate)
{

    std::string topic_name = "arm_1/arm_controller/position_command";
    this->pub = n->advertise<brics_actuator::JointPositions>(topic_name, 1);
    
    frequency = rate;
}

void Trajectory::sendTrajectory()
{
    ros::Rate rate(this->frequency);
    
    std::vector<std::vector<float> > joint_data;
    joint_data.resize(NB_JOINT_CTRL);
    
    //std::vector<std::vector<float> >::iterator it = joint_data.begin();
    
    // arm_joint_x should be in order (2,3,4) no mapping is done here...
    for(int i = 0; i < NB_JOINT_CTRL; i ++)
    {
        float offset = 0;
        
        if (this->trajectory[i].name == "arm_joint_2")
        {
            offset = 65*M_PI/180;
        }
        else if (this->trajectory[i].name == "arm_joint_3")
        {
            offset = -146*M_PI/180;
        }
        else if (this->trajectory[i].name == "arm_joint_4")
        {
            offset = 102.5*M_PI/180;
        }
        else
        {
            // TODO: manage error...
            ROS_WARN_STREAM("Unexpected joint name, name is: " << this->trajectory[i].name);
            continue;
        }
            
        //std::transform (this->trajectory[i].angles.begin(), this->trajectory[i].angles.end(), it, bind2nd(std::plus<float>(), offset));
        //it++;
        
        ROS_INFO_STREAM("Trajectory n" << i << " size: " << this->trajectory[i].angles.size());
        
        for(int j = 0; j < this->trajectory[i].angles.size(); j ++)
        {
            joint_data[i].push_back(this->trajectory[i].angles[j] + offset); 
        }
    }
    
    // TODO: check joints trajectory size (must be equal)
    
    brics_actuator::JointPositions pos_cmd = initPositionsCmd();
    
    pos_cmd.positions[0].value = 1.101e-1;
    pos_cmd.positions[4].value = 2.106e-1;
    
    // Begining of the trial
    
    for(int i = 0; i < this->trajectory[0].angles.size(); i++)
    {
        for(int jnt = 1; jnt < 4; jnt++)
        {
            pos_cmd.positions[jnt].value = joint_data[jnt - 1][i];
            
            if(g_request_shutdown)
            {
                return;
            }
            
        }
        pub.publish(pos_cmd);
        
        //ROS_INFO_STREAM(pos_cmd);
        
        ros::spinOnce();
        rate.sleep();
    }
      
}

//

void sigIntHandler(int sig)
{
    g_request_shutdown = 1;
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
