In this experiment, the mocap optitrack is used.
The goal of this experiment is to try to estimate the impedance of the human
arm, using endpoint coordinates from both the direct kinematics and the
motion capture. 

The arm first stayed static in an isomorphic task with low stiffness, then
still with low stiffness did sinusoid moves, switched to stiff behaviour
again in an isomorphic task and finally another sinusoid movement (stiff).

The robot is first controlled without any contact in therefore free
movement, up until aproximatly 60 sec. Then the control loop ros node is
used to render the robot as transparent as possible (with stability).