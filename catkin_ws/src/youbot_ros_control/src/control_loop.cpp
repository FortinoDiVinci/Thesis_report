#include <ros/package.h>
#include <ros/ros.h>
#include <signal.h>
#include <string>
#include <time.h>
//#include <eigen3/Eigen/Dense>

#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointValue.h>
#include <brics_actuator/JointVelocities.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
//#include "youbot_ros_control/StampedBool.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <boost/scoped_ptr.hpp>

//#include "youbot_ros_control/robot.h"
#include "robot.cpp"
#include "utils/utils.h"
#include "youbot_driver/generic/ConfigFile.hpp"

/**********
 * MACROS *
 **********/

// allows the haptic ball feedback
#define BALL_IMPACT_FORCE true
// allows the introduction of perturbation
#define TORQUE_PERTURBATIONS false
// after a perturbation is introduced a transition is done at the velocity level
#define VELOCITY_TRANSITION false
// the disturbance are introduced at precise ball/paddle timing if true
#define DIST_SYNC_WITH_IMPACT false
// the disturbance are introduced using the outter force loop reference
#define REF_PERTURBATIONS false

// CTRL MODE
#define WITH_VIRTUAL_MECH false
// setting the following macro with nullspace ctrl will have no effect
#define WITH_Y_ORIENTATION false
#define NULLSPACE_CTRL_LOOP true
#define X_L_CTRL_LOOP false
#define NB_JOINT_YOUBOT 5
#define NB_ACTUATED_JOINTS 3
#define DOF 3
#define REF_FRAME_ID "base_link"

const float TH_MAX[NB_JOINT_YOUBOT] = {5.7401, 25179, -0.1157, 3.3292,
                                       5.5415}; // rad
const float TH_MIN[NB_JOINT_YOUBOT] = {1.101e-1, 1.101e-1, -4.9266, 1.221e-1,
                                       2.106e-1}; // rad
const float TH_ON_D[NB_JOINT_YOUBOT] = {169, 65, -146, 102 - 90,
                                        167.5 - 110}; // degree
const float TH_OFF_D[NB_JOINT_YOUBOT] = {0.011, 0.011, -0.016, 0.023,
                                         0.12}; // degree
const bool ACTUATED_JOINTS[NB_JOINT_YOUBOT] = {false, true, true, true, false};

/**************
 * GLOBAL VAR *
 **************/

Robot *kuka_youBot;
float joint_effort_set_point[NB_JOINT_YOUBOT] = {0., 0., 0., 0., 0.};
float joint_velocity_set_point[NB_JOINT_YOUBOT] = {0., 0., 0., 0., 0.};
Pose force_torque_sensor(Point(0, 0, 0, "N"), Point(0, 0, 0, "N m"));
sig_atomic_t volatile g_request_shutdown = 0;
bool unlocked = false;
bool perturbation_lock = false;

/*************
 *  CLASSES  *
 *************/

class DisturbanceTimer {

public:
  DisturbanceTimer(ros::NodeHandle *);
  DisturbanceTimer(ros::NodeHandle *, float, float);

  // timers for perturbation introduction
  void trigger(const ros::TimerEvent &);
  void stop_curr(const ros::TimerEvent &);
  void stop_vel(const ros::TimerEvent &);
  void off(const ros::TimerEvent &);
  void unlockDist(const ros::TimerEvent &);

  // timer for ball/paddle impact
  void stopImpact(const ros::TimerEvent &);

  float getDisturbance() { return dist; };
  float getImpactMagnitude() { return force_impulse; };
  bool isDisturbance() { return dist_msg.data; };
  bool isTransitionning() { return torque_speed_transition; };
  bool isImpact() { return is_impact; };

  std::vector<float> getLastEffCmd() { return last_eff_cmd; };

  void getImpulse(const std_msgs::Float64::ConstPtr &data);
  void computePaddleFreq();

private:
  float dist_magnitude;
  float dist;
  float dist_duration;
  std::vector<float> last_eff_cmd;
  ros::NodeHandle *nh;

  ros::Timer stop_dist_timer;
  ros::Timer next_dist_timer;

  ros::Timer stop_impact_timer;
  ros::Timer next_impact_timer;

  // youbot_ros_control::StampedBool dist_msg;
  bool torque_speed_transition;
  bool is_impact;
  bool disturbance_unlocked;

  double force_impulse; // ball/paddle impact force
  unsigned int impacts_counts;
  float paddle_period;
  // std::vector<ros::Time> lastImpactsTime;
  std::vector<double> lastImpactsTime;
  float cycle_delay; // given as pourcentage of the time period
                     // this delay starts with the ball impact
  int cycle_phase;

  std_msgs::Bool dist_msg;
  std_msgs::Float32 dist_val_msg;
  std_msgs::Float32 fake_impulse_msg;

  ros::Publisher pub_time_dist;
  ros::Publisher pub_val_dist;
  ros::Publisher pub_fake_impulse;

  ros::Subscriber sub_impulse;
};

/*************
 * FUNCTIONS *
 *************/

std::vector<float> nullSpaceCtrlInit(Robot *youBot, float, float, float, float,
                                     float);
brics_actuator::JointPositions youBotInitializePosition(Robot *,
                                                        std::vector<float>);
brics_actuator::JointPositions youBotInitializePosition(Robot *);

void youBotInitializationNSCtrl(Robot *&,
                                boost::scoped_ptr<youbot::ConfigFile> &,
                                ros::NodeHandle *);
void youBotInitializationXLCtrl(Robot *,
                                boost::scoped_ptr<youbot::ConfigFile> &,
                                float *, float *, ros::NodeHandle *);
void updateRobotData(Robot *);
void nullSpaceCtrlLoop(Robot *, Pose, float, std::vector<float>, float);
void velocityRamp(Robot *, int);
void xavierLamyCtrlLoop(Robot *, Pose, Pose);
Pose virtualLineGuide(Robot *, VirtualMechanism, bool);

brics_actuator::JointVelocities initVelocitiesCmd(const bool *);
brics_actuator::JointPositions initPositionsCmd();

void getForces(const geometry_msgs::WrenchStamped::ConstPtr &data);
void getJointStates(const sensor_msgs::JointState::ConstPtr &data);
void getJointSetpoints(const sensor_msgs::JointState::ConstPtr &data);

bool checkXLimits(const Robot youBot, const float max_x, const float min_x,
                  const float max_z, const float min_z);
void safeStop(Robot *youBot);
void sigIntHandler(int sig);

/********
 * MAIN *
 ********/

int main(int argc, char **argv) {

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
  config_file.reset(new youbot::ConfigFile("youbot-manipulator.cfg",
                                           youbot_driver_path + "/config"));

  // Parameter server : PID, rate, ...

  float Kp[NB_JOINT_YOUBOT];
  float Ki[NB_JOINT_YOUBOT];
  float frequency, Kx_vm, Bx_vm, x0, Kry_vm, Bry_vm, ry0, Kq, Kd;

  std::string tmp_str = "Kx0_gain";
  std::stringstream joint_pid_data;

  for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++) {
    std::stringstream jointNameStream;
    jointNameStream << "" << ii + 1;
    tmp_str = "Kp" + jointNameStream.str() + "_gain";
    n1.getParam(tmp_str, Kp[ii]);

    tmp_str = "Ki" + jointNameStream.str() + "_gain";
    n1.getParam(tmp_str, Ki[ii]);

    joint_pid_data << "Kp: " << Kp[ii] << "\n"
                   << "Ki: " << Ki[ii] << "\n";
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
  n1.getParam("Joint_damping_gain", Kd);

  // ROS_INFO_STREAM("freq: " << frequency << "\n" << "Stiffness x: " << Kx_vm
  // << "\n"
  //    << "Damping x: " << Bx_vm << "\n" << "Stiffness ry: " << Kry_vm << "\n"
  //    << "Damping ry: " << Bry_vm << "\n" <<  "Joint eq gain: " << Kq <<
  //    "\n");

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
  qi_0 = nullSpaceCtrlInit(kuka_youBot, Kx_vm, Kq, Kd, Kp[1], Ki[1]);
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
  equilibrium.setEndpointPose(
      Pose(Point(x0, 0, 0, "m"), Point(0, ry0, 0, "rad")));
  std::vector<Endpoint> limits_vm;
  for (int i = 0; i < 2; i++) {
    Endpoint tmp_ep;
    limits_vm.push_back(tmp_ep);
  }
  limits_vm[0].setEndpointPose(Pose(Point(0.05, 0, 0, "m"), Point()));
  limits_vm[1].setEndpointPose(Pose(Point(0.06, 0, 0, "m"), Point()));

  VirtualMechanism vm(K_vm, B_vm, I_vm, equilibrium, true, limits_vm);

#endif

  Pose force_torque_vm(Point(0, 0, 0, "N"), Point(0, 0, 0, "N m"));

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

  float Te = 1 / frequency;
  float delay = Te; // the ideal is: delay = Te
  int it = 0;       // used for ramp transition between trq & vel ctrl

#if TORQUE_PERTURBATIONS
  DisturbanceTimer dist_timer(&n, dist_magnitude, 0.03); // 30ms perturbations
#elif REF_PERTURBATIONS
  DisturbanceTimer ref_timer(&n, 0, 0.1); // 100ms perturbations
#elif BALL_IMPACT_FORCE // case with no random perturbation, but with ball
                        // impacts
  DisturbanceTimer dist_timer(&n);
#endif
#if !DIST_SYNC_WITH_IMPACT && TORQUE_PERTURBATIONS
  ros::Timer timer = n.createTimer(
      ros::Duration(5.), &DisturbanceTimer::trigger, &dist_timer, true);
#endif
#if !DIST_SYNC_WITH_IMPACT && REF_PERTURBATIONS
  ros::Timer timer = n.createTimer(
      ros::Duration(10.), &DisturbanceTimer::trigger, &ref_timer, true);
#endif

  ros::Rate rate(frequency);
  kuka_youBot->updateTimeSample();
  kuka_youBot->computeEnpointPosition();

  ROS_INFO("Beginning of the main loop\n");

  /************
   *   LOOP   *
   ************/

  while (!g_request_shutdown) {
    updateRobotData(kuka_youBot);
#if WITH_VIRTUAL_MECH
    force_torque_vm = virtualLineGuide(kuka_youBot, vm, WITH_Y_ORIENTATION);
#endif

#if X_L_CTRL_LOOP
    xavierLamyCtrlLoop(kuka_youBot, force_torque_sensor, force_torque_vm);
#elif NULLSPACE_CTRL_LOOP
#if REF_PERTURBATIONS
    if (ref_timer.isImpact())
#elif TORQUE_PERTURBATIONS || BALL_IMPACT_FORCE
    if (dist_timer.isImpact()) // ball impact
#endif
    {
#if BALL_IMPACT_FORCE
#if REF_PERTURBATIONS
      kuka_youBot->setTorqueDisturbanceCmd(ref_timer.getLastEffCmd(),
                                           ref_timer.getImpactMagnitude());
#else
      kuka_youBot->setTorqueDisturbanceCmd(dist_timer.getLastEffCmd(),
                                           dist_timer.getImpactMagnitude());
#endif
      kuka_youBot->publishTorquesCmd();
#else
      // nullSpaceCtrlLoop(kuka_youBot, force_torque_sensor, x0, qi_0,
      // ref_timer.getDisturbance());
      nullSpaceCtrlLoop(kuka_youBot, force_torque_sensor, x0, qi_0, 0);
      kuka_youBot->publishVelocitiesCmd();
#endif
    }
#if REF_PERTURBATIONS
    else if (!ref_timer.isDisturbance())
    {
#elif TORQUE_PERTURBATIONS || BALL_IMPACT_FORCE
    else if (!dist_timer.isDisturbance()) // no disturbance
    {
#endif
      nullSpaceCtrlLoop(kuka_youBot, force_torque_sensor, x0, qi_0, 0);
#if TORQUE_PERTURBATIONS && VELOCITY_TRANSITION
      if (dist_timer.isTransitionning()) {
        // avoid spike after returning to velocity control
        velocityRamp(kuka_youBot, it);
        it = min(it + 1, 15);
        // ROS_INFO_STREAM("iteration: " << it);
      }
#endif
      kuka_youBot->publishVelocitiesCmd();
#if REF_PERTURBATIONS || TORQUE_PERTURBATIONS || BALL_IMPACT_FORCE
    }
#endif
#if TORQUE_PERTURBATIONS 
    else // disturbance
    {

      it = 0;
      kuka_youBot->setTorqueDisturbanceCmd(dist_timer.getLastEffCmd(),
                                           dist_timer.getDisturbance());
      kuka_youBot->publishTorquesCmd();

    }
#endif
#endif

    // ROS_INFO_STREAM_THROTTLE(0.2, "VM forces:\n" <<
    // force_torque_vm.getPoseVector());

    if (checkXLimits(*kuka_youBot, x0 + 0.03, x0 - 0.03, 0.410 + 0.05,
                     0.226 - 0.05) &&
        unlocked) {
      stop_robot = true;
      ROS_ERROR("The robot endpoint is out of bounds, the experiment has been "
                "terminated.");
      break;
    }
    // kuka_youBot->computeEnpointOrientation(false, true, false);
    // ROS_INFO_STREAM_THROTTLE(0.5, "Ry: " <<
    // kuka_youBot->getEndpoint().getPose().getOrientation().y);
    // ROS_INFO_STREAM_THROTTLE(0.5, "z: " <<
    // kuka_youBot->getEndpoint().getPose().getPosition().z);
    // ROS_INFO_STREAM_THROTTLE(0.5, "x: " <<
    // kuka_youBot->getEndpoint().getPose().getPosition().x);

    ros::spinOnce();
    rate.sleep();
  }

  if (stop_robot) {
    safeStop(kuka_youBot);
  }

  kuka_youBot->setPositionCmd(init_off_pos);
  kuka_youBot->publishPositionsCmd();
}

/*************
 * FUNCTIONS *
 *************/

// Init functions

void youBotInitializationXLCtrl(Robot *youBot,
                                boost::scoped_ptr<youbot::ConfigFile> &cfg_file,
                                float *Kp, float *Ki, ros::NodeHandle *nh) {
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

  for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++) {
    PID joint_pid(Kp[ii] * alpha, Ki[ii] * alpha, 0.0);
    joints.push_back(Joint(TH_MAX[ii], TH_MIN[ii], joint_pid));

    std::stringstream jointNameStream;
    jointNameStream << "" << ii + 1;
    joint_name = "Joint_" + jointNameStream.str();
    cfg_file->readInto(tmp_max_vel, joint_name, "MaxVelocity");
    joints[ii].setMaxVelocity(tmp_max_vel);
  }

  Jacobian youBot_jacobian(DOF, NB_ACTUATED_JOINTS);
  youBot = new Robot(joints, ACTUATED_JOINTS, youBot_jacobian,
                     initPositionsCmd(), nh);
}

void youBotInitializationNSCtrl(Robot *&youBot,
                                boost::scoped_ptr<youbot::ConfigFile> &cfg_file,
                                ros::NodeHandle *nh) {
  std::vector<Joint> joints;
  joints.reserve(NB_JOINT_YOUBOT);
  float tmp_max_vel;
  std::string joint_name;

  for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++) {
    joints.push_back(Joint(TH_MAX[ii], TH_MIN[ii], PID()));

    std::stringstream jointNameStream;
    jointNameStream << "" << ii + 1;
    joint_name = "Joint_" + jointNameStream.str();
    cfg_file->readInto(tmp_max_vel, joint_name, "MaxVelocity");
    joints[ii].setMaxVelocity(tmp_max_vel);
  }

  Jacobian youBot_jacobian(DOF, NB_ACTUATED_JOINTS);
  youBot = new Robot(joints, ACTUATED_JOINTS, youBot_jacobian,
                     initPositionsCmd(), nh);
}

std::vector<float> nullSpaceCtrlInit(Robot *youBot, float Kx_gain,
                                     float Kq_gain, float Kd_gain,
                                     float Kz_gain, float Kiz_gain) {
  std::vector<Endpoint> ep_limits;
  std::vector<float> qi_0;
  float x_lim_min, z_lim_min, x_lim_max, z_lim_max;

  // endpoint limits
  x_lim_min = -0.21;
  x_lim_max = -0.19;
  z_lim_min = 0.226;
  z_lim_max = 0.410;

  ep_limits.resize(2);
  qi_0.resize(3);

  ep_limits[0].setEndpointPose(
      Pose(Point(x_lim_min, 0, z_lim_min), Point(0, 0, 0)));
  ep_limits[1].setEndpointPose(
      Pose(Point(x_lim_max, 0, z_lim_max), Point(0, 0, 0)));

  // joints equilibrium angles
  qi_0[0] = 1.676;
  qi_0[1] = -4.363;
  qi_0[2] = 1.497;

  youBot->setNullspaceCtrlGains(Kx_gain, Kq_gain, Kd_gain);
  youBot->setEndpointLimits(ep_limits);
  youBot->setEndpointPID(PID(Kz_gain, Kiz_gain, 0));

  return qi_0;
}

brics_actuator::JointPositions
youBotInitializePosition(Robot *youBot, std::vector<float> qi_0) {
  brics_actuator::JointPositions init_off_pos;
  init_off_pos.positions.resize(NB_JOINT_YOUBOT);

  std::string joint_name;

  for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++) {
    std::stringstream jointNameStream;
    jointNameStream << "" << ii + 1;
    joint_name = "arm_joint_" + jointNameStream.str();
    // init_off_pos.positions[ii].timeStamp = ros::Time::now();
    init_off_pos.positions[ii].joint_uri = joint_name;
    init_off_pos.positions[ii].unit = "rad";
    init_off_pos.positions[ii].value = TH_ON_D[ii] * M_PI / 180;
  }

  // initial position must be in the workspace for this ctrl method
  init_off_pos.positions[1].value = qi_0[0];
  init_off_pos.positions[2].value = qi_0[1];
  init_off_pos.positions[3].value = qi_0[2];

  youBot->setPositionCmd(init_off_pos);
  usleep(1.0 * 1e6); // this delay seems necessary..
  youBot->publishPositionsCmd();
  usleep(3.0 * 1e6); // waits 3 seconds for the end of the movement

  return init_off_pos;
}

brics_actuator::JointPositions youBotInitializePosition(Robot *youBot) {
  brics_actuator::JointPositions init_off_pos;
  init_off_pos.positions.resize(NB_JOINT_YOUBOT);

  std::string joint_name;

  for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++) {
    std::stringstream jointNameStream;
    jointNameStream << "" << ii + 1;
    joint_name = "arm_joint_" + jointNameStream.str();
    // init_off_pos.positions[ii].timeStamp = ros::Time::now();
    init_off_pos.positions[ii].joint_uri = joint_name;
    init_off_pos.positions[ii].unit = "rad";
    init_off_pos.positions[ii].value = TH_ON_D[ii] * M_PI / 180;
  }

  youBot->setPositionCmd(init_off_pos);
  usleep(1.0 * 1e6); // this delay seems necessary..
  youBot->publishPositionsCmd();
  usleep(3.0 * 1e6); // waits 3 seconds for the end of the movement
  return init_off_pos;
}

// main functions

void updateRobotData(Robot *youBot) {
  youBot->updateJacobianInverse();
  youBot->updateJacobianTranspose();
  youBot->computeEnpointPosition();
  youBot->computeEnpointOrientation(false, true, false); // only ry orientation
                                                         //
  // ROS_INFO_STREAM("" << youBot->getEndpoint().getPose().getPosition().z);
}

void nullSpaceCtrlLoop(Robot *youBot, Pose ft_sens, float x_eq,
                       std::vector<float> qi_eq, float ref) {
  // ref allows the introduction of non null force
  // perturbation/disturbance/reference it should be noted that the new
  // reference is limited by the response time of the outerloop force control
  if (unlocked) {
    youBot->computeNullspaceCollaborativeCmd(x_eq, ft_sens.getPosition().z, ref,
                                             qi_eq);
  }
}

void xavierLamyCtrlLoop(Robot *youBot, Pose ft_sens, Pose ft_virt_guide) {
  youBot->updateJacobianTranspose();
  youBot->setInputError(
      youBot->computeJointTorquesFromWrench(ft_sens + ft_virt_guide));
  youBot->computeVelocityCollaborativeCmd();
}

void velocityRamp(Robot *youBot, int iteration) {
  brics_actuator::JointVelocities dummy_vel_cmd;
  dummy_vel_cmd = youBot->getVelocityCmd();
  // ROS_INFO_STREAM("Before: "<< dummy_vel_cmd.velocities[2].value);
  int idx = 0;
  float alpha = iteration / 15;
  for (int i = 0; i < NB_JOINT_YOUBOT; i++) {
    if (ACTUATED_JOINTS[i]) {
      dummy_vel_cmd.velocities[idx].value =
          (dummy_vel_cmd.velocities[idx].value * alpha +
           youBot->getJoints()[i].getAngularVelocity() * (1 - alpha)) /
          2;
      idx++;
    }
  }
  // ROS_INFO_STREAM("After: "<< dummy_vel_cmd.velocities[2].value);
  youBot->setVelocityCmd(dummy_vel_cmd);
}

Pose virtualLineGuide(Robot *youBot, VirtualMechanism vm, bool orientation) {
  Pose ft_guide(Point(0, 0, 0, "N"), Point(0, 0, 0, "N m"));
  kuka_youBot->computeEnpointPosition();

  if (orientation) {
    youBot->computeEnpointOrientation(false, true, false);
    ft_guide = vm.verticalXLineFixture(
        youBot->getEndpoint().getPose().getPosition().x,
        youBot->getEndpoint().getPose().getOrientation().y,
        youBot->getEndpoint().getVelocities().getPosition().x,
        youBot->getEndpoint().getVelocities().getOrientation().y);
  } else {
    ft_guide = vm.verticalXLineFixture(
        youBot->getEndpoint().getPose().getPosition().x,
        youBot->getEndpoint().getVelocities().getPosition().x);
  }

  return ft_guide;
}

/* Initialize joint velocities msg to be send through ros topic,
 * only actuated joints are controlled by velocity msgs */

brics_actuator::JointVelocities initVelocitiesCmd(const bool joint_actuated[]) {
  brics_actuator::JointVelocities vel_cmd;
  vel_cmd.velocities.reserve(NB_JOINT_YOUBOT);
  brics_actuator::JointValue jnt_val;

  for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++) {
    if (joint_actuated[ii] == true) {
      std::stringstream jointNameStream;
      jointNameStream << "" << ii + 1;
      jnt_val.joint_uri = "arm_joint_" + jointNameStream.str();
      jnt_val.unit = "s^-1 rad";
      jnt_val.value = 0.0;
      vel_cmd.velocities.push_back(jnt_val);
    } else
      continue;
  }
  return vel_cmd;
}

/* Initialize joint positions msg to be send through ros topic */

brics_actuator::JointPositions initPositionsCmd() {
  brics_actuator::JointPositions pos_cmd;
  pos_cmd.positions.resize(NB_JOINT_YOUBOT);

  for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++) {
    std::stringstream jointNameStream;
    jointNameStream << "" << ii + 1;
    pos_cmd.positions[ii].joint_uri = "arm_joint_" + jointNameStream.str();
    pos_cmd.positions[ii].unit = "rad";
    pos_cmd.positions[ii].value = 0.0;
  }
  return pos_cmd;
}

// Callback functions

void getForces(const geometry_msgs::WrenchStamped::ConstPtr &data) {
  // header
  // TODO : check that headers are consistant
  if (data->header.frame_id != REF_FRAME_ID) {
    ROS_WARN_STREAM_THROTTLE(
        5, "Reference frame of the force torque msg seems wrong, "
               << REF_FRAME_ID << " is expected.");
  }
  // wrench
  force_torque_sensor.setPositionX(data->wrench.force.x);
  force_torque_sensor.setPositionY(data->wrench.force.y);
  force_torque_sensor.setPositionZ(data->wrench.force.z);
  force_torque_sensor.setOrientationX(data->wrench.torque.x);
  force_torque_sensor.setOrientationY(data->wrench.torque.y);
  force_torque_sensor.setOrientationZ(data->wrench.torque.z);
}

void getJointStates(const sensor_msgs::JointState::ConstPtr &data) {
  for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++) {
    kuka_youBot->updateJointData(ii, data->position[ii], data->velocity[ii],
                                 data->effort[ii]);
  }
  unlocked = true;
}

void getJointSetpoints(const sensor_msgs::JointState::ConstPtr &data) {
  for (int ii = 0; ii < NB_JOINT_YOUBOT; ii++) {
    joint_effort_set_point[ii] = data->effort[ii];
    joint_velocity_set_point[ii] = data->velocity[ii];
  }
}

// security

bool checkXLimits(const Robot youBot, const float max_x, const float min_x,
                  const float max_z, const float min_z) {
  float youbot_x = youBot.getEndpoint().getPose().getPosition().x;
  float youbot_z = youBot.getEndpoint().getPose().getPosition().z;
  float youbot_ry = youBot.getEndpoint().getPose().getOrientation().y;

  if ((youbot_x > max_x) || (youbot_x < min_x))
    return true;

  else if ((youbot_z > max_z) || (youbot_z < min_z))
    return true;

  else if ((youbot_ry > 1.85) || (youbot_ry < 1.25))
    return true;

  // ROS_INFO_STREAM("Robot ry: " << youbot_ry);

  return false;
}

// stop the robot to its current position

void safeStop(Robot *youBot) {
  std::vector<Joint> joints = youBot->getJoints();

  if (NB_JOINT_YOUBOT != joints.size()) {
    ROS_ERROR("The number of joints was not properly defined, safe stop cannot "
              "happen !");
    return;
  }

  brics_actuator::JointPositions pos_cmd;
  pos_cmd.positions.resize(NB_JOINT_YOUBOT);

  for (int i = 0; i < NB_JOINT_YOUBOT; i++) {
    std::stringstream jointNameStream;
    jointNameStream << "" << i + 1;
    pos_cmd.positions[i].joint_uri = "arm_joint_" + jointNameStream.str();
    pos_cmd.positions[i].unit = "rad";
    pos_cmd.positions[i].value = joints[i].getAngle();
  }

  youBot->setPositionCmd(pos_cmd);
  youBot->publishPositionsCmd();

  // wait for keyboard to return to initial position

  ROS_INFO("When ready, push Enter key while on the terminal, to let the robot "
           "return to its initial position");

  std::string s;
  while (getline(std::cin, s) && !s.empty()) {
  }
}

//

void sigIntHandler(int sig) { g_request_shutdown = 1; }

// timers

// Constructor for the case without random perturbation, but only ball impacts
DisturbanceTimer::DisturbanceTimer(ros::NodeHandle *n) {
  this->nh = n;

  torque_speed_transition = false;
  is_impact = false;
  disturbance_unlocked = true;

  force_impulse = 0;
  impacts_counts = 0;

  last_eff_cmd.resize(NB_ACTUATED_JOINTS);
  lastImpactsTime.resize(
      5, ros::Time::now().toSec()); // 5 previous impact times are stored

  std::string topic_name = "";
  topic_name = "impulse";
  sub_impulse = nh->subscribe(topic_name, 1, &DisturbanceTimer::getImpulse,
                              this); // ball impact force
  topic_name = "arm_1/fake_impulse";
  pub_fake_impulse = nh->advertise<std_msgs::Float32>(topic_name, 1);
  srand(time(NULL));
}

DisturbanceTimer::DisturbanceTimer(ros::NodeHandle *n, float magnitude,
                                   float duration) {
  this->nh = n;

  dist_magnitude = magnitude;
  dist_duration = duration;
  dist = 0.0;
  dist_msg.data = false;
  torque_speed_transition = false;
  is_impact = false;
  disturbance_unlocked = true;

  cycle_delay = 0.25; // should be in the middle of decreasing phase
  cycle_phase = 0;    // select multiples of cycle delay, for now x1, x2 or x3
  force_impulse = 0;
  impacts_counts = 0;

  last_eff_cmd.resize(NB_ACTUATED_JOINTS);
  lastImpactsTime.resize(
      5, ros::Time::now().toSec()); // 5 previous impact times are stored

  std::string topic_name = "";
  // topic_name = "arm_1/disturbance_time";
  // pub_time_dist = nh->advertise<youbot_ros_control::StampedBool>(topic_name,
  // 1); pub_time_dist = nh->advertise<std_msgs::Bool>(topic_name, 1);
  topic_name = "arm_1/disturbance_val";
  pub_val_dist = nh->advertise<std_msgs::Float32>(topic_name, 1);
  topic_name = "impulse";
  sub_impulse = nh->subscribe(topic_name, 1, &DisturbanceTimer::getImpulse,
                              this); // ball impact force
  topic_name = "arm_1/fake_impulse";
  pub_fake_impulse = nh->advertise<std_msgs::Float32>(topic_name, 1);

  srand(time(NULL));
}

void DisturbanceTimer::trigger(const ros::TimerEvent &e) {
  ROS_INFO("TRIGGERED");
  int idx = 0;
  for (int i = 0; i < NB_JOINT_YOUBOT; i++) {
    if (ACTUATED_JOINTS[i]) {
      last_eff_cmd[idx] = joint_effort_set_point[i];
      idx++;
    }
  }
  dist_msg.data = true;
  dist = dist_magnitude * (2 * (rand() % 2) - 1); // sign is chosen randomly
  // this helps to know the phase of the cycle either +0.01, +0.02 or 0.03.
  dist_val_msg.data = dist + cycle_phase * 0.1;
  // dist_msg.stamp = ros::Time::now();
  // pub_time_dist.publish(dist_msg);
  pub_val_dist.publish(dist_val_msg);
  #if VELOCITY_TRANSITION
    stop_dist_timer = nh->createTimer(ros::Duration(dist_duration),
                                      &DisturbanceTimer::stop_curr, this, true);
  #else
    stop_dist_timer = nh->createTimer(ros::Duration(dist_duration),
                                      &DisturbanceTimer::off, this, true);
  #endif
}

void DisturbanceTimer::stop_curr(const ros::TimerEvent &e) {
#if VELOCITY_TRANSITION
  // ROS_INFO("STOPPED");
  dist_msg.data = true;
  dist = 0.;
  dist_val_msg.data = dist;
  // dist_msg.stamp = ros::Time::now();
  // pub_time_dist.publish(dist_msg);
  pub_val_dist.publish(dist_val_msg);

  next_dist_timer = nh->createTimer(ros::Duration(0.0015),
                                    &DisturbanceTimer::stop_vel, this, true);
#else
  dist_msg.data = false;
  dist = 0.;
  dist_val_msg.data = dist;
  pub_val_dist.publish(dist_val_msg);

  next_dist_timer = nh->createTimer(ros::Duration(0.0015),
                                    &DisturbanceTimer::off, this, true);

#endif
}

void DisturbanceTimer::stop_vel(const ros::TimerEvent &e) {
  // ROS_INFO("STOPPED");
  dist_msg.data = false;
  torque_speed_transition = true;

  next_dist_timer =
      nh->createTimer(ros::Duration(0.015), &DisturbanceTimer::off, this, true);
}

void DisturbanceTimer::off(const ros::TimerEvent &e) {
// ROS_INFO("TRIGGERED OFF");
#if VELOCITY_TRANSITION == false
  dist_msg.data = false;
  dist = 0.;
  dist_val_msg.data = dist;
  pub_val_dist.publish(dist_val_msg);
#endif
  dist_msg.data = false;
  torque_speed_transition = false;
  // dist_msg.stamp = ros::Time::now();
  // pub_time_dist.publish(dist_msg);

#if DIST_SYNC_WITH_IMPACT && (TORQUE_PERTURBATIONS || REF_PERTURBATIONS)
  next_dist_timer = nh->createTimer(
      ros::Duration((rand() % 301) / 100 + 2), &DisturbanceTimer::unlockDist,
      this, true); // at least 2s between perturbations, at most 5s
#else
  next_dist_timer =
      nh->createTimer(ros::Duration((rand() % 401) / 100 + dist_duration),
                      &DisturbanceTimer::trigger, this, true);
#endif
}

void DisturbanceTimer::unlockDist(const ros::TimerEvent &e) {
  // ROS_INFO("unlocked");
  disturbance_unlocked = true; // free disturbance lock
}

void DisturbanceTimer::stopImpact(const ros::TimerEvent &e) {
  is_impact = false; // free disturbance lock during impact
}

void DisturbanceTimer::getImpulse(const std_msgs::Float64::ConstPtr &data) {
  ros::Time t_imp = ros::Time::now();
  double tmp_impulse = data->data;
  // if ( tmp_impulse != 0. )
  if (tmp_impulse != 0.) // temp modification to launch disturbance without link
                         // with ball impact
  {
    // inversion to fit robot/sensor base reference (TODO: Check...)
    force_impulse =
        (tmp_impulse < 0) ? 0 : -tmp_impulse / 3; // max(0,tmp_impulse)
    // TODO: get scale factor from ball boucing node

    // std::rotate(lastImpactsTime.rbegin(), lastImpactsTime.rbegin() + 1,
    // lastImpactsTime.rend()); ROS_INFO_STREAM("IMPACT: " << tmp_impulse <<
    // "N");
    if (!is_impact) {
      impacts_counts++;
      ROS_INFO_STREAM("Impact nb: "<< impacts_counts);
#if DIST_SYNC_WITH_IMPACT && (TORQUE_PERTURBATIONS || REF_PERTURBATIONS)
      // avoid considering multiple consecutive impacts in chaotic bouncings
      lastImpactsTime.pop_back();
      lastImpactsTime.insert(lastImpactsTime.begin(), t_imp.toSec());

      // ROS_INFO_STREAM("PERIOD: " << paddle_period << "s");
      // ROS_INFO_STREAM("PERIOD HISTORY: " << lastImpactsTime.at(0) << "\t" <<
      // lastImpactsTime.at(1) << "\t" << lastImpactsTime.at(2) << "\t" <<
      // lastImpactsTime.at(0));

      this->computePaddleFreq();
      // ROS_INFO_STREAM("before lock: " << disturbance_unlocked);
      ROS_INFO_STREAM("Impact nb = "<< impacts_counts);
      if (impacts_counts >= 5) 
      // perturbations starts after fews impacts to ensure steady state
      {
        if (disturbance_unlocked && (paddle_period > 0.5)) 
        {
          disturbance_unlocked = false;
          // ROS_INFO_STREAM("locked: " << disturbance_unlocked);
          // disturbance is generated after the impact with a delay that
          // corresponds to either of 3 random phase of the cycle (eg. 25%, 50%
          // and 75% of the cycle)
          cycle_phase = rand() % 3 + 1;
          next_dist_timer = nh->createTimer(
              ros::Duration(paddle_period * cycle_delay * cycle_phase),
              &DisturbanceTimer::trigger, this, true);
        }
      }
#endif
     //if (rand()%10 > 0) // 1/10th of chance to ignore the impact feedback
     if (rand()%10 > 8) // 9/10th of chance to ignore the impact feedback
      {
      // set current effort for ball/paddle impulse
        int idx = 0;
        for (int i = 0; i < NB_JOINT_YOUBOT; i++) {
          if (ACTUATED_JOINTS[i]) {
            last_eff_cmd[idx] = joint_effort_set_point[i];
            idx++;
          }
        }
        is_impact = true; // avoid perturbation during impact
        stop_impact_timer = nh->createTimer(
            ros::Duration(0.03), &DisturbanceTimer::stopImpact, this, true);
        // ROS_INFO_STREAM("Ball impact = "<< force_impulse);
      }
      else
      {
        fake_impulse_msg.data = force_impulse;
        ROS_INFO_STREAM("Faked impact: "<< force_impulse);
        pub_fake_impulse.publish(fake_impulse_msg);
      }
    }
  }
}

void DisturbanceTimer::computePaddleFreq() {
  // ros::Duration sum(0);
  double average = 0;

  for (std::vector<double>::iterator it = lastImpactsTime.begin();
       it != lastImpactsTime.end() - 1; ++it)
  // for (int it=0; it<lastImpactsTime.size()-1; ++it)
  {
    average += *it - *(it + 1);
    // average += lastImpactsTime.at(it+1) - lastImpactsTime.at(it);
  }
  average = average / (lastImpactsTime.size() - 1);
  // average = sum.toSec() / (lastImpactsTime.size() - 1);

  if (average > 0) {
    paddle_period = average;
  } else {
    // manage error
  }
}

