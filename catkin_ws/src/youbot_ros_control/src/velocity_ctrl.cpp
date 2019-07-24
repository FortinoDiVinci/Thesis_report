#include "youbot_ros_control/velocity_ctrl.h"


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

SpeedController::SpeedController(int dof, int fst_jnt, ros::NodeHandle n1)
{
    m_joint_state.name.assign(1, "arm_joint_x");
    m_joint_state.position.resize(NUMBER_ARM_JOINTS);
    m_joint_state.velocity.resize(NUMBER_ARM_JOINTS);
    m_joint_state.effort.resize(NUMBER_ARM_JOINTS);
	
    m_joint_trq_msg = SpeedController::initializeJointTorqueMsg(dof, fst_jnt);
	
    ros::NodeHandle n;
	
    torque_cmd_pub = n.advertise<brics_actuator::JointTorques> ("arm_1/arm_controller/torque_command", 1);
	
	// for debug
	spld_speed_cmd_pub = n.advertise<brics_actuator::JointVelocities> ("arm_1/arm_controller/velocities/slpd", 1);
	ramp_speed_cmd_pub = n.advertise<brics_actuator::JointVelocities> ("arm_1/arm_controller/velocities/ramp", 1);
	
    speed_cmd_sub = n.subscribe("arm_1/arm_controller/velocity_command/ros", 1, &SpeedController::jointVelocityCmdCallback, this);
    joint_state_sub = n.subscribe("joint_states", 1, &SpeedController::jointStateCallback, this);
	
    std::string configFilePath;
    n1.param<std::string>("youBotConfigurationFilePath", configFilePath, "/home/youbot/catkin_ws/src/youbot_driver/config/");
    youbot::ConfigFile configfile("youbot-manipulator.cfg", configFilePath);
	
    std::string jointName;
	
	for (int i = 0; i < NUMBER_ARM_JOINTS; i++)
	{
		std::stringstream jointNameStream;
		jointNameStream << "Joint_" << i + 1;
		jointName = jointNameStream.str();
		
		float gr_den, gr_num, gear_ratio;
		configfile.readInto(gr_num, jointName, "GearRatio_numerator");
		configfile.readInto(gr_den, jointName, "GearRatio_denominator");
		m_gear_ratio[i] = gr_num/gr_den;
		
		configfile.readInto(m_Kv[i], jointName, "PParameterFirstParametersSpeedControl");
		configfile.readInto(m_ramp_on[i], jointName, "RampGenerator");
		configfile.readInto(m_clip_vmax[i], jointName, "MaxVelocity");
		configfile.readInto(m_acceleration_max[i], jointName, "MotorAcceleration");
		last_t[i] = ros::Time::now();		
		ROS_INFO("Joint %i: ramp is %d, maxVel is %f, max acc is %f", i+1, m_ramp_on[i], m_clip_vmax[i], m_acceleration_max[i]);
		// corrected value
		m_Kv[i] = m_Kv[i]/256;
		//m_acceleration_max[i] = m_acceleration_max[i] / (gear_ratio * 2.0 * M_PI) * 60.0;
	}
	
	m_joint_vel_cmd.velocities.resize(NUMBER_ARM_JOINTS);
	m_joint_vel_cmd_spld.velocities.resize(NUMBER_ARM_JOINTS);
	m_joint_vel_cmd_ramp.velocities.resize(NUMBER_ARM_JOINTS);

	for (int i = 0; i < NUMBER_ARM_JOINTS; i ++)
	{
	    std::stringstream jointNameStream;
		jointNameStream << "Joint_" << i + 1;
		jointName = jointNameStream.str();
		
		m_joint_vel_cmd.velocities[i].joint_uri = jointName;
		m_joint_vel_cmd.velocities[i].unit = "s^-1 rad";
		
		m_joint_vel_cmd_spld.velocities[i].joint_uri = jointName;
		m_joint_vel_cmd_spld.velocities[i].unit = "s^-1 rad";
		
		m_joint_vel_cmd_ramp.velocities[i].joint_uri = jointName;
		m_joint_vel_cmd_ramp.velocities[i].unit = "s^-1 rad";
		m_joint_vel_cmd_ramp.velocities[i].value = 0.;
	}	
}

brics_actuator::JointTorques SpeedController::initializeJointTorqueMsg(int DOF, int first_joint)
{
    brics_actuator::JointTorques m_joint_torques;
    std::stringstream jointName;
    //m_joint_torques.torques.clear();

    for (int i = first_joint; i < DOF + first_joint; i++)
    {
      brics_actuator::JointValue joint;
      //joint.value = arr[i + first_joint];
      joint.unit = boost::units::to_string(boost::units::si::ampere); // joints are in reality ctrl in current
      jointName.str("");
      jointName << "arm_joint_" << (i + 1);
      joint.joint_uri = jointName.str();

      m_joint_torques.torques.push_back(joint);
    }
    return m_joint_torques;
}

void SpeedController::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (int i = 0; i < NUMBER_ARM_JOINTS; i++)
	{
		m_joint_state.position[i] = msg->position[i];
		m_joint_state.velocity[i] = msg->velocity[i];
		m_joint_state.effort[i] = msg->effort[i];
	}
}

void SpeedController::jointVelocityCmdCallback(const brics_actuator::JointVelocities::ConstPtr& msg)
{
	
	if (msg->velocities.size() < 1)
	{
		ROS_WARN("youBot driver received an invalid joint velocities command.");
		return;
	}
	
	//youbot::JointVelocitySetpoint desiredAngularVelocity;
	string unit = boost::units::to_string(boost::units::si::radian_per_second);

	/* populate mapping between joint names and values  */
	std::map<string, double> jointNameToValueMapping;
	for (int i = 0; i < static_cast<int> (msg->velocities.size()); ++i)
	{
		if (unit == msg->velocities[i].unit)
		{
			jointNameToValueMapping.insert(make_pair(msg->velocities[i].joint_uri, msg->velocities[i].value));
		}
		else
		{
			ROS_WARN("Unit incompatibility. Are you sure you want to command %s instead of %s ?", msg->velocities[i].unit.c_str(), unit.c_str());
		}

	}
	/* loop over all youBot arm joints and check if something is in the received message that requires action */
	for (int i = 0; i < NUMBER_ARM_JOINTS; ++i)
	{
		/* check what is in map */
		std::stringstream jointName;
		jointName.str("");
		jointName << "arm_joint_" << (i + 1);
		map<string, double>::const_iterator jointIterator = jointNameToValueMapping.find(jointName.str());
		if (jointIterator != jointNameToValueMapping.end())
		{
			/* set the desired joint value */
			ROS_DEBUG("Trying to set joint %s to new velocity value %f", (jointName.str()).c_str(), jointIterator->second);
			m_joint_vel_cmd.velocities[i].value = jointIterator->second;
			//m_joint_vel_cmd.velocities[i].joint_uri = jointIterator->first;
			m_joint_vel_cmd.velocities[i].timeStamp = msg->velocities[i].timeStamp;
		}
		else
		{
			m_joint_vel_cmd.velocities[i].value = std::numeric_limits<double>::quiet_NaN();
		}	
	}
}

void SpeedController::proportionalController(const int dof, const int fst_jnt)
{
	for (int i = fst_jnt; i < dof + fst_jnt; i++)
    {
        //spld_speed_cmd_pub.publish(m_joint_vel_cmd_spld);
		if(m_ramp_on[i])
		{
			generateRamp(i);
			ramp_speed_cmd_pub.publish(m_joint_vel_cmd_spld);
		}
		// clip velocities
		m_joint_vel_cmd_spld.velocities[i].value = std::min(float(m_joint_vel_cmd_spld.velocities[i].value), m_clip_vmax[i]);
		
		float err = m_joint_vel_cmd_spld.velocities[i].value - m_joint_state.velocity[i];
		// clip according to max current (already done in the lower level)
		// m_joint_torques.torques[i - fst_jnt].value = std::min(m_Kv*err, m_clip_imax[i]);
		m_joint_trq_msg.torques[i - fst_jnt].value = m_Kv[i]*err; // only required torques are transmisted
	}
}

void SpeedController::overSampling(int dof, int first_joint_ctrl)
{
    if (m_joint_vel_cmd.velocities.size() < 1) 
    {
        ROS_ERROR_THROTTLE(1, "No joint velocity recieved");
        return;
    }

	ros::Time t = ros::Time::now();
	
	for (int i = 0; i < NUMBER_ARM_JOINTS; ++i)
	{
		// check if a joint meant to be controlled was not given a proper cmd signal
		if ((i >= first_joint_ctrl && i < first_joint_ctrl + dof) && isnan(m_joint_vel_cmd_spld.velocities[i].value))
		{
			ROS_WARN("Joint %d did not recieve a proper velocity command signal", i);
			m_joint_vel_cmd_spld.velocities[i].timeStamp = t;
			continue; // do not assign NaN val when a joint is meant to be controlled
		}
		m_joint_vel_cmd_spld.velocities[i].value = m_joint_vel_cmd.velocities[i].value;
		m_joint_vel_cmd_spld.velocities[i].timeStamp = t;
	}
}

void SpeedController::generateRamp(const int i)
{
	float dt = (m_joint_vel_cmd_spld.velocities[i].timeStamp - last_t[i]).toSec();
	last_t[i] = m_joint_vel_cmd_spld.velocities[i].timeStamp;
	//ROS_INFO_THROTTLE(1, "dt: %f", dt);
	float acc = (m_joint_vel_cmd_spld.velocities[i].value - m_joint_state.velocity[i]) / dt;
	//float acc = float(m_joint_vel_cmd.velocities[i].value - m_joint_vel_cmd_ramp.velocities[i].value) / dt;
	ROS_INFO("acc: %f", acc);
	if (abs(acc) > m_acceleration_max[i])
	{
	    ROS_INFO("acc (act): %f", acc);
		m_joint_vel_cmd_spld.velocities[i].value = sgn(acc) * m_acceleration_max[i]*dt + m_joint_vel_cmd_ramp.velocities[i].value;
	}
	m_joint_vel_cmd_ramp.velocities[i].value = m_joint_vel_cmd_spld.velocities[i].value; // n-1 val
}

bool SpeedController::IsJointLimit(int i)
{
    // if lower limit might be reached
    if(m_joint_state.position[i] - m_joint_limit_angle[0][i] < 0.3 && 
    m_joint_state.velocity[i] < -0.2)
    {
        ROS_ERROR("Critical position %f and velocity %f reached on joint %i", m_joint_state.position[i], m_joint_state.velocity[i], i + 1);
        return true;
    }
    // if upper limit might be reached
    else if(m_joint_state.position[i] - m_joint_limit_angle[1][i] > -0.3 && 
    m_joint_state.velocity[i] > 0.2)
    {
        ROS_ERROR("Critical position %f and velocity %f reached on joint %i", m_joint_state.position[i], m_joint_state.velocity[i], i + 1);
        return true;
    }
    else
        return false;
}

void SpeedController::checkJointsLimit(int dof, int fst_jnt)
{
    for(int i = 0; i < dof; i++)
    {
        if(IsJointLimit(i + fst_jnt))
        {
            // brake (torque in the opposite direction)
            m_joint_trq_msg.torques[i].value = m_joint_trq_msg.torques[i].value * (-0.2);
        }
    }
}

 sig_atomic_t volatile g_request_shutdown = 0;
 
 void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

/************
 *   MAIN   *
 ************/
 
 int main(int argc, char** argv)
{
	ros::init(argc, argv, "velocity_ctrl_node", ros::init_options::NoSigintHandler);
	signal(SIGINT, mySigIntHandler);
	
	ros::NodeHandle n1("~");
	
	int dof;
	int fst_jnt;
	int freq;
	
	n1.param<int>("degree_of_freedom", dof, 1);
	n1.param<int>("first_joint", fst_jnt, 5);
	n1.param<int>("rate", freq, 1000.);
	fst_jnt--;
	
	if ((fst_jnt + dof) > NUMBER_ARM_JOINTS)
	{
		// this error avoid launching the node with unconsistent input parameters
		ROS_ERROR("The number of joints controlled and the first joint to be controlled are not consistent");
		ROS_WARN("The first joint required to be controlled is the nb %d, and %d joint(s) were required to be controlled", fst_jnt, dof);
		return -1;
	}
	
	SpeedController vel_ctrl(dof, fst_jnt, n1);
	vel_ctrl.setRampOn(true, 4);
	//vel_ctrl.setAccmax(5, 4);
	ros::Rate rate(freq);
	
	while (!g_request_shutdown) 
	{
		vel_ctrl.overSampling(dof, fst_jnt);
		vel_ctrl.proportionalController(dof, fst_jnt);	
		vel_ctrl.checkJointsLimit(dof, fst_jnt);	
		vel_ctrl.torque_cmd_pub.publish(vel_ctrl.getJointTorquesMsg());
		
		ros::spinOnce();
        rate.sleep();
	}
	
	ros::shutdown();
}
