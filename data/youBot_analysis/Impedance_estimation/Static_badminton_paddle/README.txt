In this experiment, the mocap optitrack is used.
The goal of this experiment is to try to estimate the impedance of the badminton
paddle, using endpoint coordinates from both the direct kinematics and the
motion capture. The Transformation matrix was already obtained in previous 
experiments (Motion_capture_validation/mocap_kinematic_2)

The paddle was placed against a table, stabilized using 4 1kg weight.

The robot is first position controlled without any contact in therefore free
movement. Then only joint 2 is torque controlled with a first contact with
a setpoint set to 0, to estimate the influence of gravity alone on the 
paddle.

The robot gain for the position control of joint 2,3&4 are the following:
Joint 2 P = 60
Joint 3 P = 60
Joint 4 P = 30