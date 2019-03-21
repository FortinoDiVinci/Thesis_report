#!/usr/bin/env python
# This program is dedicated to the kuka youbot, without base nor gripper
# therefore a 5 axis robots communicating with this script throught ROS topics

__version__ = '0.1'
__author__ = 'Vincent FORTINEAU <vincent.fortineau@centralesupelec.fr>'

########################
#  LIBRARIES
########################

import rospy
import time 
import threading

from brics_actuator.msg import JointPositions, JointVelocities, JointValue
from sensor_msgs.msg import JointState

########################
#  CONST & GLOBAL VAR
########################

# thetas[i] = [pos_i, vel_i, torq_i]
thetas = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
init_flag = False
# software limitation to work in specified space
joint_upper_limit = [0.5, 1.76, -0.95, 2.84, 5.6]
joint_lower_limit = [0.02, 1.35, -2.06, 1.31, 0.12]
#
out_of_bound = [0, 0, 0, 0, 0]
speed_sign = [0, 0, 0, 0, 0]

########################
#  FUNCTIONS
########################

# Callback function

def getJointStateCallback(joints):
    global thetas, joint_upper_limit, joint_lower_limit, out_of_bound, speed_sign
    try:
        for theta in xrange(len(thetas)):    
            thetas[theta][0] = joints.position[theta]
            thetas[theta][1] = joints.velocity[theta]   
            thetas[theta][2] = joints.effort[theta] 
            
            
            speed_sign[theta] = sign(joints.velocity[theta])

            if joints.position[theta] >= joint_upper_limit[theta]:
                out_of_bound[theta] = 1
            elif joints.position[theta] <= joint_lower_limit[theta]: 
                out_of_bound[theta] = -1
            else:
                out_of_bound[theta] = 0
          
    except:
        print("An error occured while reading the robot joint state")

# Callback function       

def getJointStateOnce(joints):
    global thetas, init_flag, sub
    try:
        for theta in xrange(len(thetas)):    
            thetas[theta][0] = joints.position[theta]
            thetas[theta][1] = joints.velocity[theta]   
            thetas[theta][2] = joints.effort[theta]    
               
    except:
        print("An error occured while reading the robot joint state")    
    sub.unregister()
    init_flag = True

#

def subJointStateOnce():
    # Get initial values and set them to thetas
    global sub
    sub = rospy.Subscriber("joint_states", JointState, getJointStateOnce)
    
    counter = 0
    # Wait for the execution of getJointState (subscriber callback function)
    while(init_flag == False):
        time.sleep(0.1)
        counter = counter + 1
        if counter > 50:
            print("Fail to get state")
            return -1	
    return 0

# Publish joints position in predefined states

def operatingState(pub, conf):
    
    if conf == 1:
        func_pos = [0.5, 1.76, -2.06, 2.84, 0.12]
    elif conf == 2:
        func_pos = [0.02, 1.35, -0.95, 1.31, 5.60]
    else:
        print("Please choose between config 1 or 2")
        return
    
    nbJoints = len(func_pos)
    
    armJointPositions = JointPositions()
    for joint in xrange(nbJoints):
        pos = JointValue()
        pos.joint_uri = "arm_joint_" + str(joint+1)
        pos.value = func_pos[joint]
        pos.unit = "rad"
        armJointPositions.positions.append(pos)

    pub.publish(armJointPositions)
    
    return True

# Publish joints position in initial state

def initState(pub):
    
    init_pos = [0.01007, 0.01007, -0.0158, 0.02213, 0.11062]
    
    nbJoints = len(init_pos)
    
    armJointPositions = JointPositions()
    for joint in xrange(nbJoints):
        pos = JointValue()
        pos.joint_uri = "arm_joint_" + str(joint+1)
        pos.value = init_pos[joint]
        pos.unit = "rad"
        armJointPositions.positions.append(pos)

    pub.publish(armJointPositions)
    
    return True

# Dummy delay that experimentaly seems necessary after publishing joint
# position

def pubDelay():
    for wait in xrange(0,3):
        time.sleep(0.1)    

#

def isMoving():
    nul_th_count = 0  
    
    for theta in xrange(len(thetas)): 
        if thetas[theta][1] < 0.01 and thetas[theta][1] > -0.01:
            nul_th_count = nul_th_count + 1
            
    if nul_th_count == len(thetas):               
        return False    
    else:
        return True

#

def sign(x):
    
    if (x == 0): 
        return 0
    elif (x < 0):
        return -1
    else:
        return 1
# 

def checkPosThread():
    th = threading.currentThread()
    global thetas  # TODO a cleaner way should be used ?
        
    while getattr(th, "stop", False) == False:
        for theta_i in xrange(len(thetas)):
            if (thetas[theta_i][0] <= joint_upper_limit[theta_i] and
                thetas[theta_i][0] >= joint_lower_limit[theta_i]):
                # this joint is okay
                # do_something()
                pass
            else:
                print("The command required is out of the working \
                        space: %")
            
    print("\nExiting Position checking thread...")

#

def checkPosCallback(joint_vel_cmd):
    global pub_vel, thetas, out_of_bound, speed_sign
	
    not_generated_cmd = [0, 1, 2, 3, 4]
   
    for cmd in joint_vel_cmd.velocities:
        try:
            joint_nb = int(cmd.joint_uri[-1]) - 1
        except:
            print("joint uri (name) is not properly defined." \
                  "\n %s was given" % cmd.joint_uri)
        else:
            not_generated_cmd.remove(joint_nb) # remove by value
            # this case test if joint is out of bound and if out_of_boud 
            # and speed_sign have the same sign
            if (out_of_bound[joint_nb] != 0 and
                out_of_bound[joint_nb] + sign(cmd.value) != 0):  
                #
                cmd.value = 0.
                rospy.logwarn("Joint %d is out of bound", (joint_nb+1))
                
    for joint_cmd in not_generated_cmd:
        if (out_of_bound[joint_cmd] != 0 and speed_sign[joint_cmd] != 0 and
            out_of_bound[joint_cmd] + speed_sign[joint_cmd] != 0):
            #
            joint_vel_cmd.velocities.append(
                createJointValueVel(0., joint_cmd + 1))
            rospy.logwarn("Joint %d is out of bound (no cmd)", (joint_cmd + 1))
            
    pub_vel.publish(joint_vel_cmd)
    #pubDelay()
    #print(joint_vel_cmd)
    
#

def createJointValueVel(val, joint_nb):
    j_v = JointValue()
    j_v.joint_uri = "arm_joint_" + str(joint_nb)
    j_v.value = val
    j_v.unit = "s^-1 rad" # velocity unit

    return j_v

########################
#  MAIN
########################
    
if __name__=="__main__":
    
    rospy.init_node('teleop_joint_control')
	
    """ROS Publisher, Subscribers and Services"""
	
    pub_pos = rospy.Publisher('arm_1/arm_controller/position_command',
                               JointPositions, queue_size=1)
    pub_vel = rospy.Publisher('arm_1/arm_controller/velocity_command',
                               JointVelocities, queue_size=1)
    
    sub = None 
    sub_state = rospy.Subscriber("joint_states", JointState,
                                 getJointStateCallback)
    
    """Necessary dummy operation to initiate communication"""

    # Dummy publishing 
    armJointPos = JointPositions()      
    pub_pos.publish(armJointPos)
    pubDelay()
    
    """Initialisation"""
    
    is_Moving = operatingState(pub_pos, 2)
    pubDelay()
        
    while(isMoving()):
        pass
    is_Moving = False
     
    is_Moving = initState(pub_pos)
    pubDelay()
    
    """Main operations"""
    
    sub_cmd = rospy.Subscriber("joint_vel_cmd/", JointVelocities,
                                checkPosCallback)
                                
    #th = threading.Thread(target=checkPosThread, args=(,))
    #th.start()
    
    rospy.spin()
    
    #th.stop = True
    #th.join()
    
    sub_cmd.unregister()
    
    is_Moving = initState(pub_pos)
    pubDelay()
    
    while(isMoving()):
        pass
    is_Moving = False   
    
    sub_state.unregister()
    
