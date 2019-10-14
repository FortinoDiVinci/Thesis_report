
//
//  jacobian.h
//  
//
//  Created by Vincent Fortineau on 26/06/2019.
//
//

#ifndef jacobian_h
#define jacobian_h

#include <math.h>

void youBotJacobianT(const float thetas[5], float jac_matrix[][6]);
// single joint ctrl
void youBotJacobianTJoint4(const float th4, float jac_matrix[][6]); 
void youBotJacobianTJoint3(const float th3, float jac_matrix[][6]);
void youBotJacobianTJoint2(const float th2, float jac_matrix[][6]);
// two joints ctrl 
void youBotJacobianTJoints34(const float thetas[5], float jac_matrix[][6]);
void youBotJacobianTJoints34XYZDOF(const float thetas[5], float jac_matrix[][3]); // only forces
void youBotJacobianTJoints34ZDOF(const float thetas[5], float jac_matrix[][3]); //only z
// three joints ctrl
void youBotJacobianTJoints234(const float thetas[5], float jac_matrix[][6]);
void youBotJacobianTJoints234XYZDOF(const float thetas[5], float jac_matrix[][3]); //only z
void youBotJacobianTJoints234ZDOF(const float thetas[5], float jac_matrix[][3]); //only z

#endif /* jacobian_h */


