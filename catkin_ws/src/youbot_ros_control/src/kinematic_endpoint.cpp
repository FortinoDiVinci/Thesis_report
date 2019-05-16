//
//  kinematic_endpoint.cpp
//  
//
//  Created by Vincent Fortineau on 14/05/2019.
//
//


/*****************
*    INCLUDES    *
******************/

#include <math.h>
#include <unistd.h>
//#include <matrix.h>

#include <ros/ros.h>
//#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include "youbot_ros_control/forward_kinematic.h"


/*****************
 *   FUNCTIONS   *
 *****************/

int TH_SIZE = 5;
float THETAS[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
int SEQ = 0;

void getJointPos(const sensor_msgs::JointState::ConstPtr& joint) {

    for (int i = 0; i < TH_SIZE; i++)
    {
        THETAS[i] = joint->position[i];
    }
}

geometry_msgs::PoseStamped initPoseStamped(const float t[3], const float q[4])
{

    geometry_msgs::PoseStamped ps;
    
    ps.header.seq = SEQ;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = "sensor";
    
    ps.pose.position.x = t[0];
    ps.pose.position.y = t[1];
    ps.pose.position.z = t[2];
    
    ps.pose.orientation.x = q[1];
    ps.pose.orientation.y = q[2];
    ps.pose.orientation.z = q[3];
    ps.pose.orientation.w = q[0];

    SEQ += 1;
    return ps;
}

void matmul4X4(const float a[][4], const float b[][4], float c[][4])
{
    float a00 = a[0][0];
    float a01 = a[0][1];
    float a02 = a[0][2];
    float a03 = a[0][3];

    float a10 = a[1][0];
    float a11 = a[1][1];
    float a12 = a[1][2];
    float a13 = a[1][3];
    
    float a20 = a[2][0];
    float a21 = a[2][1];
    float a22 = a[2][2];
    float a23 = a[2][3];
    
    float a30 = a[3][0];
    float a31 = a[3][1];
    float a32 = a[3][2];
    float a33 = a[3][3];
    
    float b00 = b[0][0];
    float b01 = b[0][1];
    float b02 = b[0][2];
    float b03 = b[0][3];
    
    float b10 = b[1][0];
    float b11 = b[1][1];
    float b12 = b[1][2];
    float b13 = b[1][3];
    
    float b20 = b[2][0];
    float b21 = b[2][1];
    float b22 = b[2][2];
    float b23 = b[2][3];
    
    float b30 = b[3][0];
    float b31 = b[3][1];
    float b32 = b[3][2];
    float b33 = b[3][3];
    
    c[0][0] = a00*b00 + a10*b01 + a20*b02 + a30*b03;
    c[0][1] = a01*b00 + a11*b01 + a21*b02 + a31*b03;
    c[0][2] = a02*b00 + a12*b01 + a22*b02 + a32*b03;
    c[0][3] = a03*b00 + a13*b01 + a23*b02 + a33*b03;
    
    c[1][0] = a00*b10 + a10*b11 + a20*b12 + a30*b13;
    c[1][1] = a01*b10 + a11*b11 + a21*b12 + a31*b13;
    c[1][2] = a02*b10 + a12*b11 + a22*b12 + a32*b13;
    c[1][3] = a03*b10 + a13*b11 + a23*b12 + a33*b13;
    
    c[2][0] = a00*b20 + a10*b21 + a20*b22 + a30*b23;
    c[2][1] = a01*b20 + a11*b21 + a21*b22 + a31*b23;
    c[2][2] = a02*b20 + a12*b21 + a22*b22 + a32*b23;
    c[2][3] = a03*b20 + a13*b21 + a23*b22 + a33*b23;
    
    c[3][0] = a00*b30 + a10*b31 + a20*b32 + a30*b33;
    c[3][1] = a01*b30 + a11*b31 + a21*b32 + a31*b33;
    c[3][2] = a02*b30 + a12*b31 + a22*b32 + a32*b33;
    c[3][3] = a03*b30 + a13*b31 + a23*b32 + a33*b33;
    
}

void translation_from_matrix(const float rot_m[][4], float t[3])
{
    t[0] = rot_m[0][3];
    t[1] = rot_m[1][3];
    t[2] = rot_m[2][3];
}

void rotation_from_matrix(const float rot_m[][4], float q[4]) {
    
    float r11 = rot_m[0][0];
    float r12 = rot_m[0][1];
    float r13 = rot_m[0][2];
    float r21 = rot_m[1][0];
    float r22 = rot_m[1][1];
    float r23 = rot_m[1][2];
    float r31 = rot_m[2][0];
    float r32 = rot_m[2][1];
    float r33 = rot_m[2][2];
    float q0 = (r11 + r22 + r33 + 1.0f) / 4.0f;
    float q1 = (r11 - r22 - r33 + 1.0f) / 4.0f;
    float q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
    float q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
    // this section deals with computation error close to 0
    if (q0 < 0.0f) {
        q0 = 0.0f;
    }
    if (q1 < 0.0f) {
        q1 = 0.0f;
    }
    if (q2 < 0.0f) {
        q2 = 0.0f;
    }
    if (q3 < 0.0f) {
        q3 = 0.0f;
    }
    
    q0 = sqrt(q0);
    q1 = sqrt(q1);
    q2 = sqrt(q2);
    q3 = sqrt(q3);
    
    if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
        //q0
        q1 = copysign(q1, r32 - r23);
        q2 = copysign(q2, r13 - r31);
        q3 = copysign(q3, r21 - r12);
    }
    else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
        q0 = copysign(q0, r32 - r23);
        //q1
        q2 = copysign(q2, r21 + r12);
        q3 = copysign(q3, r13 + r31);
    }
    else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
        q0 = copysign(q0, r13 - r31);
        q1 = copysign(q1, r21 + r12);
        //q2
        q3 = copysign(q3, r32 + r23);
    }
    else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
        q0 = copysign(q0, r21 - r12);
        q1 = copysign(q1, r31 + r13);
        q2 = copysign(q2, r32 + r23);
        //q3
    }
    else {
        std::cout << "Err in quaternion conversion\n";
    }
    float r = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3); //norm
    q[0] = q0 / r;
    q[1] = q1 / r;
    q[2] = q2 / r;
    q[3] = q3 / r;
    
}

/******************
 *      MAIN      *
 ******************/

int main(int argc, char** argv)
{

    ros::init(argc, argv, "kinematic_endpoint");
    ros::NodeHandle n;   
    ros::NodeHandle n1("~");  
    ros::Subscriber sub = n.subscribe("joint_states", 1, getJointPos);
    ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>
                            ("forward_kinematic/sensor", 1);

    usleep(300000); //# make sure subscriber is ready by waiting 300ms

    const float th_sens = -110 * M_PI / 180; // rotation along z axis between sensor axis and robot end effector
    const float sensor_link[][4] = {
        {cos(th_sens), -sin(th_sens), 0, 0},
        {sin(th_sens), cos(th_sens) , 0, 0},
        {0           , 0            , 1, 0},
        {0           , 0            , 0, 1}
    };
    
    float ratio;
    float freq;
    std::string ref_frame;
    
    n1.param<float>("tf_ratio", ratio, 1.0);
    n1.param<float>("rate", freq, 1000.0);
    n1.param<std::string>("frame_id", ref_frame, "base_link");

    tf::TransformBroadcaster br;
    //tf::Transform listener;

    ros::Rate rate(freq);
    
    float rot_matrix[4][4];       
    float rot_matrix_sensor[4][4];
    float t[3];
    //float q[4];
    
    while (n.ok())
    {
        
        forward_kinematic(THETAS, rot_matrix);  
        matmul4X4(rot_matrix, sensor_link, rot_matrix_sensor);
        translation_from_matrix(rot_matrix, t);
        //rotation_from_matrix(rot_matrix_sensor, q);
        t[0] *= ratio;
        t[1] *= ratio;
        t[2] *= ratio;
        
        tf::Vector3 trans(t[0], t[1], t[2]); //tf::tfScalar() cast ??
        tf::Matrix3x3 rot(rot_matrix[0][0], rot_matrix[0][1], rot_matrix[0][2],
                          rot_matrix[1][0], rot_matrix[1][1], rot_matrix[1][2],
                          rot_matrix[2][0], rot_matrix[2][1], rot_matrix[2][2]);
        
        #if 0 
        std::cout << "Homogenous matrix:\n";                  
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j++; j < 4) {
                std::cout << rot_matrix_sensor[i][j] << ' ';
            }
            std::cout << '\n';
        }
        
        std::cout << "Matrix3x3:\n";                  
        for (int i = 0; i < 3; i++) {
            //tf::Vector3 v = 
            for (int j = 0; j++; j < 3) {
                std::cout << rot[i][j] << ' ';
            }
            std::cout << '\n';
        }
        #endif  
                        
        tf::Transform transform(rot, trans);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), ref_frame, "sensor"));
        
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;

}


