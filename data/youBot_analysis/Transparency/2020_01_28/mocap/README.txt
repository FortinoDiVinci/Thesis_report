In this experiment, the mocap optitrack is used.
The first mouvements (commended in velocity) are done at low speed to be able to calibrate the optitrack pose.
3 target were placed on the robot endeffector (before both the handle and the sensor).

The robot was then controlled in position to reach the experiment position, and after positionning was done effectively, the robot 4th joint was torque controlled.
The other joints were still position controlled with higher gain than usual on the position P controller (P2 = 60, P3 = 60).

The robot end effector was placed next to a badminton paddle, fixed with 1kg weights.
The goal of the experiment, is to identify the paddle stiffness, and see if position from the optitrack system and the robot are consistent.
If not, flexibility in the robot might be the explanation. In that was, are the differences in the identification significant ?