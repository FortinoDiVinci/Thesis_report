Data collected on April 7th, 8th, 9th 2021
Several different experimental conditions were tested. Two new users were
recorder. Voluntary stiff physical interactions were also recorded. 
A new condition was also tested, whith some of the force feedback "ghosted".

* Name (hand): yy:mm:dd hh:mm (end of the experiment) User Comment

Perturbed ball bouncing task achieved with haptic feedbacks, by 3 differents
users.

* Calibration01: 21:04:07 11:14
* Trial 01 (l*): 21:04:07 11:25 user01 (expert)
* Trial 02 (l*): 21:04:07 15:16 user02 (beginner)
* Trial 03 (r ): 21:04:07 15:39 user03 (beginner)

Same experiment with voluntary strong cocontraction of the arm (stiff)

* Trial 04 (l*): 21:04:07 16:08 user01 => motion cap was not recorded..

Unerturbed ball bouncing task achieved with haptic feedbacks. A tenth of the
haptic feedfack are "ghosted" to try to observe the virtual behaviour in 
experiment (a) and experiment (b) is the complement.

* Calibration02: 21:04:08 09:48
* Trial 05 (l*): 21:04:08 09:24 user01 (a) // ghosted feedback are not timed 
* Trial 06 (l*): 21:04:08 09:56 user01 (a)
* Trial 07 (l*): 21:04:08 10:11 user01 (b)
* Trial 08 (l*): 21:04:08 10:29 user01 (b)
* Trial 09 (l*): 21:04:08 10:45 user01 (a)

Same experiment with voluntary strong cocontraction of the arm (stiff)

* Trial 10 (l*): 21:04:08 11:06 user01 (b)
* Trial 11 (l*): 21:04:08 12:28 user01 (a)

Perturbed ball bouncing task achieved with 10% "ghosted" haptic feedbacks

* Trial 12 (r ): 21:04:08 12:47 user01 
* Trial 13 (l*): 21:04:08 13:33 user01 (strong cocontraction)
* Trial 14 (l*): 21:04:08 13:53 user01

Perturbed ball bouncing task achieved with haptic feedbacks (stiff)

* Calibration03: 21:04:09 10:30
* Trial 15 (l*): 21:04:09 11:38 user01 (strong cocontraction)

------------
Parameters
------------

Ball_bouncing package parameters (simulated environment):
Target height: 1.7 (m)
Z position offset: 0.32 (m)
Kinematic coefficient: 6
Restiution coefficient: 0.6
Gravity force: 9.81 (m.s^-2)

Ball impact force are computed with the following equation: 
fi(k) = -(vb(k) - vp(k))*(1+alpha)*sqrt(mb*mp*K/(mp+mb))/pi
- vb and vp respectively the ball and paddle velocities
- mb and mp respectively the ball and paddle masses
- alpha = 0.6, the restitution coefficient
- K = 650, the total equivalent stiffness
Ball impact are introduced as torque perturbations of 30ms

Robot control parameters:
Admittance control as described in Fortineau et al. (2020)
Cartesian admittance control
Kp = 1.5e-2 (proportionnal gain)
Ki = 8e-2 (integral gain)
Position control (x axis)
Kx = 20 (proportionnal gain)
Joint control (q0)
K_q = 5 (proportionnal gain)
Kd_q = 0.1 (derivative gain)
