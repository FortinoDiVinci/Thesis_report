#!/usr/bin/env python
# Initial code created by Graylin Trevor Jay (tjay@cs.brown.edu) an published under Creative Commons Attribution license.
# addition for signal interrupt by Koen Buys

#import youbot_driver_ros_interface
#import roslib; roslib.load_manifest('youbot_oodl')
import rospy

from geometry_msgs.msg import Twist
from brics_actuator.msg import JointPositions, JointVelocities, JointValue
from sensor_msgs.msg import JointState

import sys, select, termios, tty, signal, time

msg = """
Reading from the keyboard  and Publishing to JointState!
---------------------------
Moving around:
   p	m
   o	l
   i	k
   u	j
   y	h

q/z : increase/decrease max speeds by 10%
c   : change control mod (pos / vel)
anything else : stop

CTRL-C to quit
"""

moveBindings = {
#		     x,y,theta ratio
		'p':(1,1), 	# clockwise turn for joint1
		'm':(1,-1), 	# i-clockwise turn for joint1
		'o':(2,1), 	# clockwise turn for joint2
		'l':(2,-1),	# i-clockwise turn for joint2
		'i':(3,1), 	# clockwise turn for joint3
		'k':(3,-1), 	# i-clockwise turn for joint3
		'u':(4,1), 	# clockwise turn for joint4
		'j':(4,-1), 	# i-clockwise turn for joint4
		'y':(5,1), 	# clockwise turn for joint5
		'h':(5,-1), 	# i-clockwise turn for joint5
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
	      }

modeBindings={
		'c':(1,1),
	     }

jointsUpperLimit = [
        5.84014,
        2.61799,
        -0.015708,
        3.4292,
        5.64159,
]

jointsLowerLimit = [
        0.0100692,
        0.0100692,
        -5.02655,
        0.0221239,
        0.110619,
]

thetas = [0, 0, 0, 0, 0]
thetas_speed = [0, 0, 0, 0, 0]

numberOfJoints = 5
init_flag = False
pos_ctrl_flag = True

speed = 0.1

class TimeoutException(Exception): 
    pass 


def getKey():
    def timeout_handler(signum, frame):
        raise TimeoutException()
    
    old_handler = signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(1) #this is the watchdog timing
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    try:
       key = sys.stdin.read(1)
       #print "Read key"
    except TimeoutException:
       #print "Timeout"
       return "-"
    finally:
       signal.signal(signal.SIGALRM, old_handler)

    signal.alarm(0)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed):
	return "currently:\tspeed %s " % (speed)


# initial values of the robot (get thetas states)
def getJointState(joints):
    global thetas, init_flag, sub#
    try:
        for theta in xrange(0, numberOfJoints):    
			thetas[theta] = joints.position[theta]          
            # Initial speed state is alway null
			thetas_speed[theta] = 0 
    except:
        print("An error occured while reading the robot joint state")
    
    sub.unregister()
    init_flag = True
    print("Joint state initialized")

def subJointStateOnce():
    # Get initial values and set them to thetas
    global sub
    sub = rospy.Subscriber("joint_states", JointState, getJointState)
    
    counter = 0
    # Wait for the execution of getJointState (subscriber callback function)
    while(init_flag == False):
        time.sleep(0.1)
        counter = counter + 1
        if counter > 50:
            print("Failure to get initial state")
            return -1	
    return 0

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
	
    pub_pos = rospy.Publisher('arm_1/arm_controller/position_command', JointPositions)
    pub_vel = rospy.Publisher('arm_1/arm_controller/velocity_command', JointVelocities)
    #pub = rospy.Publisher('cmd_vel', Twist)
    rospy.init_node('teleop_twist_keyboard')
    
    sub = None
    subJointStateOnce()
    
    # If initial state is normal, all thetas will be close to 0
    # There is an offset difference between the joint state
    # and the minimal cmd, the minimum is therefore added	
    for theta in xrange(0, numberOfJoints):
        if theta+1 == 3: # third joint
            thetas[theta] = thetas[theta] + jointsUpperLimit[theta]
        else:
            thetas[theta] = thetas[theta] + jointsLowerLimit[theta]

    status = 0
    new_th = 0

    try:
        print msg
        print vels(speed)
        print("In Position control mode")
        
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                joint = moveBindings[key][0]
                new_th = moveBindings[key][1]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]

                print vels(speed)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            elif key in modeBindings.keys():
            	if(pos_ctrl_flag): 
            		pos_ctrl_flag = False
            		print("In Velocity control mode")
            	else:
            		pos_ctrl_flag = True
            		subJointStateOnce()
            		print("In Position control mode")
            	
            else:
                new_th = 0
                joint = 5
                if (key == '\x03'):
                    break
            # Position control
            if (pos_ctrl_flag):
            
            	temp = thetas[joint-1] + new_th * speed
            	
                if temp < jointsUpperLimit[joint-1]:
		
                    if temp > jointsLowerLimit[joint-1]:
                        thetas[joint-1] = temp
                        # create position variable to send
                        armJointPositions = JointPositions()
                        pos = JointValue()
                        pos.joint_uri = "arm_joint_" + str(joint)
                        pos.value = thetas[joint-1]
                        pos.unit = "rad"
                        armJointPositions.positions.append(pos)
                        # Send new position
                        pub_pos.publish(armJointPositions)
                    else:
                        print("Joint %-2s lower limit is %-6s" %(joint, jointsLowerLimit[joint-1]))
                else:
                    print("Joint %-2s upper limit is %-6s" %(joint, jointsUpperLimit[joint-1]))
            # Speed control
            else:
                thetas_speed[joint-1] = thetas_speed[joint-1] + new_th * speed
                # create velocity variable to send
                armJointVelocities = JointVelocities()
                spe = JointValue()
                spe.joint_uri = "arm_joint_" + str(joint)
                spe.value = thetas_speed[joint-1]
                spe.unit = "s^-1 rad"
                armJointVelocities.velocities.append(spe)
                # Send new joint speed
                pub_vel.publish(armJointVelocities)

            
        rospy.spinOnce()	

    except:
        print "error"

    finally:
        armJointPositions = JointPositions()
        pos = JointValue()
        #pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


