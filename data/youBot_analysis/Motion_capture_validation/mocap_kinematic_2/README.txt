In this experiment, the mocap optitrack is used.
The goal of this experiment is to try to validate the kinematics function for endpoint computation of the robot.

1 Rigid body was placed and detectable by the camera on the robot: 
- The rigid body on the fifth joint has 4 markers (6 in the calibration set 3), 
the marker centered on the handle is the reference point 
(it was placed below the ball of the handle) 
=> as a consequence, the point given by the optitrack system will have an offset 
on y and z. The offset on y is irrelevant since the robot move in the xz plan.
The offset on z can be estimated with an accuracy in the millimeter range...

Only joints 2, 3 and 4 were controlled during the experiment, using rqt.