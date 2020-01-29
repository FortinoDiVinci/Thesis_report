//
//  gravity_compensation.cpp
//  
//
//  Created by Vincent Fortineau on 16/05/2019.
//
//

/******************
 *    INCLUDES    *
 ******************/
 
#include <sstream>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/WrenchStamped.h>
//#include <geometry_msgs/QuaternionStamped.h>

#include "utils/filter.h"

#define G 9.80665
#define SIZE 6
#define BANDSTOP_FILTER     1
#define LOWPASS_FILTER      0

geometry_msgs::WrenchStamped sensor_data;
float BIAS[SIZE] = {-14.3223, 1.2478, -6.2585, 0.0271, 0.7662, 0.0359};
float GRAVITY[int(SIZE/2)] = {0., 0., 0.};
float LEVER[int(SIZE/2)] = {0., 0., 0.};

const int _f_ord = 4;
double _f_num[_f_ord+1] = {1.73273711695787e-05, 6.93094846783149e-05, 0.000103964227017472, 6.93094846783149e-05, 1.73273711695787e-05};
double _f_den[_f_ord+1] = {1, -3.64829756145213, 5.00542495625879, -3.06003016908075, 0.703180012212798};

/*****************
 *   FUNCTIONS   *
 *****************/

// For rotation matrix, the inverse is the transpose matrix
void weight_projection(const tf::Matrix3x3 matrix, const float z, float gravity[int(SIZE/2)])
{
    gravity[0] = -float(matrix.getRow(2).getX())*z;
    gravity[1] = -float(matrix.getRow(2).getY())*z;
    gravity[2] = -float(matrix.getRow(2).getZ())*z;
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
    //ROS_INFO("force x: %f", sensor_data.wrench.force.x);
}

void copyWrenchData(const geometry_msgs::WrenchStamped data, const float grav[int(SIZE/2)], const float lev[int(SIZE/2)], const float bias[SIZE], geometry_msgs::WrenchStamped *g_comp)
{
    //header
    g_comp->header.seq = data.header.seq;
    g_comp->header.stamp = data.header.stamp;
    //wrench
    g_comp->wrench.force.x = data.wrench.force.x - grav[0] - bias[0];
    g_comp->wrench.force.y = data.wrench.force.y - grav[1] - bias[1];
    g_comp->wrench.force.z = data.wrench.force.z - grav[2] - bias[2];
    g_comp->wrench.torque.x = data.wrench.torque.x - lev[0] - bias[3];
    g_comp->wrench.torque.y = data.wrench.torque.y - lev[1] - bias[4];
    g_comp->wrench.torque.z = data.wrench.torque.z - bias[5];
    //ROS_INFO_THROTTLE(0.01, "force z: %f", data.wrench.force.x);
    //ROS_INFO_THROTTLE(0.01, "force z: %f", g_comp->wrench.force.z);
}

// used to change force to the static frame
void rotateWrenchData(geometry_msgs::WrenchStamped *g_comp, tf::Matrix3x3 rot_mat)
{
    float forces[3] = {g_comp->wrench.force.x, g_comp->wrench.force.y, g_comp->wrench.force.z};
    float torques[3] = {g_comp->wrench.torque.x, g_comp->wrench.torque.y, g_comp->wrench.torque.z};
    
    float fx = rot_mat[0][0]*forces[0] + rot_mat[0][1]*forces[1] + rot_mat[0][2]*forces[2];
    float fy = rot_mat[1][0]*forces[0] + rot_mat[1][1]*forces[1] + rot_mat[1][2]*forces[2];
    float fz = rot_mat[2][0]*forces[0] + rot_mat[2][1]*forces[1] + rot_mat[2][2]*forces[2];
    
    float tx = rot_mat[0][0]*torques[0] + rot_mat[0][1]*torques[1] + rot_mat[0][2]*torques[2];
    float ty = rot_mat[1][0]*torques[0] + rot_mat[1][1]*torques[1] + rot_mat[1][2]*torques[2];
    float tz = rot_mat[2][0]*torques[0] + rot_mat[2][1]*torques[1] + rot_mat[2][2]*torques[2];
    
    g_comp->wrench.force.x = fx;
    g_comp->wrench.force.y = fy;
    g_comp->wrench.force.z = fz;
    g_comp->wrench.torque.x = tx;
    g_comp->wrench.torque.y = ty;
    g_comp->wrench.torque.z = tz;    
}

/******************
 *      MAIN      *
 ******************/

int main(int argc, char** argv)
{

    ros::init(argc, argv, "force_sensor_utils");
    ros::NodeHandle n;
    ros::NodeHandle n1("~");
    ros::Subscriber sub = n.subscribe("netft_data", 1, getForceCallback);
    ros::Publisher pub = n.advertise<geometry_msgs::WrenchStamped> ("force_sensor/grav_comp", 1);
    ros::Publisher pub_unfil = n.advertise<geometry_msgs::WrenchStamped> ("force_sensor/grav_comp_unfiltered", 1);
    
    tf::TransformListener listener;

    std::string sensor_frame = "sensor";
    std::string parent_frame = "base_link";
    float m;
    float l;
    float freq;
    float z; // weight
    bool not_initialized_bias;
    int avg_val;
    int counter; // used only if bias need init.
    float temp_bias[SIZE] = {0., 0., 0., 0., 0., 0.};

    int f_ord = _f_ord;
    std::vector<double> num (_f_num, _f_num + sizeof(_f_num) / sizeof(_f_num[0]) );
    std::vector<double> den (_f_den, _f_den + sizeof(_f_den) / sizeof(_f_den[0]) );   
    
    Filter bw_filter(num, den, f_ord);

    n1.param<float>("sensor_mass", m, 0.1096); //mass in kg
    n1.param<float>("sensor_arm_lever", l, 0.0103); //arm lever im m
    n1.param<float>("rate", freq, 1000.);
    n1.param<bool>("bias", not_initialized_bias, true);
    
    ros::Rate rate(freq);

    if (not_initialized_bias) {
        n1.param<int>("avg_values", avg_val, freq);
        counter = avg_val;
        for (int i = 0; i < SIZE; i++) {
            BIAS[i] = 0.0;
        }
    }

    z = G*m;

    usleep(300000); //# make sure subscriber is ready by waiting 300ms

    geometry_msgs::WrenchStamped grav_comp_data;
    grav_comp_data.header.frame_id = parent_frame;
    tf::StampedTransform tf_sens;
    
    //geometry_msgs::QuaternionStamped q_s;

    /******************
     * Initialization *
     ******************/
     
    if(not_initialized_bias)
    {
        bool tf_ok = false;

        for (int i = 0; i < counter; i++)
        {
        
            try {
                listener.lookupTransform(parent_frame, sensor_frame, ros::Time(0), tf_sens);
                tf_ok = true;
            }
            
            catch (tf::TransformException e) {
                ROS_WARN_THROTTLE(1, "%s", e.what());
                i = i - 1;
            }
            
            if (tf_ok == true) {
                weight_projection(tf_sens.getBasis(), z, GRAVITY);
                LEVER[0] = GRAVITY[1] * l;
                LEVER[1] = GRAVITY[0] * (-l);

                copyWrenchData(sensor_data, GRAVITY, LEVER, BIAS, &grav_comp_data);
            
                temp_bias[0] += grav_comp_data.wrench.force.x;
                temp_bias[1] += grav_comp_data.wrench.force.y;
                temp_bias[2] += grav_comp_data.wrench.force.z;
                temp_bias[3] += grav_comp_data.wrench.torque.x;
                temp_bias[4] += grav_comp_data.wrench.torque.y;
                temp_bias[5] += grav_comp_data.wrench.torque.z;  
                
                tf_ok = false;
                ros::spinOnce();
                rate.sleep();  
            }      
        }
        not_initialized_bias = false;
        for (int i = 0; i < SIZE; i++) {
            BIAS[i] = temp_bias[i]/avg_val;
        }
        ROS_INFO_STREAM("Bias estimation for f/t sensor done.\nForce bias: " << BIAS[0] << ", " << BIAS[1] << ", " << BIAS[2] << "\nTorque bias: " << BIAS[3] << ", " << BIAS[4] << ", " << BIAS[5] << "\n");
    }

    /*************
     * Main Loop *
     *************/
    
    while (n.ok()) {

        try {
            listener.lookupTransform(parent_frame, sensor_frame, ros::Time(0), tf_sens);
        }
        
        catch (tf::TransformException e) {
            ROS_WARN_THROTTLE(1, "%s", e.what());
            continue;
        }

        //tf::quaternionTFToMsg(tf_sens.getRotation(), q_s.quaternion);
        //q_s.header.stamp = tf_sens.stamp_;

        weight_projection(tf_sens.getBasis(), z, GRAVITY);
        LEVER[0] = GRAVITY[1] * l;
        LEVER[1] = GRAVITY[0] * (-l);

        copyWrenchData(sensor_data, GRAVITY, LEVER, BIAS, &grav_comp_data);
        rotateWrenchData(&grav_comp_data, tf_sens.getBasis());  // static frame      
        //ROS_INFO_THROTTLE(0.1, "force z: %f", grav_comp_data.wrench.force.z);
        pub_unfil.publish(grav_comp_data);
        
        // data on z is filter after gravity compensation
        grav_comp_data.wrench.force.z = bw_filter.filter(grav_comp_data.wrench.force.z);
        
        pub.publish(grav_comp_data);
        ros::spinOnce();
        rate.sleep();
    }
}

