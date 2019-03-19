#!/usr/bin/env python
# Code created by Vincent FORTINEAU (vincent.fortineau@centralesupelec.fr).

import rospy

from brics_actuator.msg import JointPositions, JointVelocities, JointValue
from sensor_msgs.msg import JointState

import sys, select, termios, tty, signal, time, threading


# thetas[i] = [pos_i, vel_i, torq_i]
thetas = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]

numberOfJoints = 5
init_flag = False


def getJointState(joints):
    global thetas
    try:
        for theta in xrange(0, numberOfJoints):    
            thetas[theta][0] = joints.position[theta]
            thetas[theta][1] = joints.velocity[theta]   
            thetas[theta][2] = joints.effort[theta]    
               
    except:
        print("An error occured while reading the robot joint state")
        

def getJointStateOnce(joints):
    global thetas, init_flag, sub
    try:
        for theta in xrange(0, numberOfJoints):    
            thetas[theta][0] = joints.position[theta]
            thetas[theta][1] = joints.velocity[theta]   
            thetas[theta][2] = joints.effort[theta]    
               
    except:
        print("An error occured while reading the robot joint state")    
    sub.unregister()
    init_flag = True

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

def functionState(pub, conf):
    
    if conf == 1:
        func_pos = [0.02, 1.76, -2.06, 2.84, 0.12]
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

def pub_delay():
    for wait in xrange(0,3):
        time.sleep(0.1)    


def isMoving():
    nul_th_count = 0  
    
    for theta in xrange(0, numberOfJoints): 
        if thetas[theta][1] < 0.01 and thetas[theta][1] > -0.01:
            nul_th_count = nul_th_count + 1
            
    if nul_th_count == numberOfJoints:               
        return False    
    else:
        return True

def checkPosThread():
    th = threading.currentThread()
    global thetas  # TODO a cleaner way should be used
        
    while getattr(th, "stop", False) == False:
        for theta in thetas:
            if theta[0] > 2:
                print("outside limits")
            
    print("Exiting Position checking thread...")
    
if __name__=="__main__":
    
    rospy.init_node('teleop_joint_control')
	
    pub_pos = rospy.Publisher('arm_1/arm_controller/position_command', JointPositions, queue_size=1)
    pub_vel = rospy.Publisher('arm_1/arm_controller/velocity_command', JointVelocities, queue_size=1)
    
    sub = None
    
    sub_state = rospy.Subscriber("joint_states", JointState, getJointState)

    # Dummy publishing 
    armJointPos = JointPositions()      
    pub_pos.publish(armJointPos)
    # Dumy delay
    for wait in xrange(0,3):
        time.sleep(0.1)
    
    is_Moving = functionState(pub_pos, 2)
    pub_delay()
        
    while(isMoving()):
        pass
    is_Moving = False
     
    is_Moving = initState(pub_pos)
    pub_delay()
    
    th = threading.Thread(target=checkPosThread)
    th.start()
    
    rospy.spin()
    
    th.stop = True
    th.join()
    
