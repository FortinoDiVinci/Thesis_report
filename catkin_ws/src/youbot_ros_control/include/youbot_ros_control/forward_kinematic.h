//
//  forward_kinematic.h
//  
//
//  Created by Vincent Fortineau on 14/05/2019.
//
//

#ifndef forward_kinematic_hpp
#define forward_kinematic_hpp

#include <math.h>
#include <iostream>

//float (*forward_kinematic(const float* ))[4];
void forward_kinematic(const float*, float (*)[4]);
void forwardKinematicTranslationOnly(const float*, float*);

#endif /* forward_kinematic_hpp */
