/*
 * kinematics.cpp
 *
 *  Created on: Nov 27, 2014
 *      Author: crm
 */

#include "kinematics.h"

namespace cpr_robots{


kinematics::kinematics() {

	// Joint Min and Max values in degree
	jointMinMax[0][0] = -150.0;		jointMinMax[0][1] = 150.0;
	jointMinMax[1][0] = -30.0;		jointMinMax[1][1] =  60.0;
	jointMinMax[2][0] = -40.0;		jointMinMax[2][1] = 120.0;
	jointMinMax[3][0] = -130.0;		jointMinMax[3][1] = 130.0;
	jointMinMax[4][0] = -90.0;		jointMinMax[4][1] = 90.0;
	jointMinMax[5][0] = -130.0;		jointMinMax[5][1] = 130.0;

}

kinematics::~kinematics() {
	// TODO Auto-generated destructor stub
}


void kinematics::CheckJointMinMax(float * j){

	for(int i=0; i<nrOfJoints; i++){
		if(j[i] < jointMinMax[i][0]) j[i] = jointMinMax[i][0];
		if(j[i] > jointMinMax[i][1]) j[i] = jointMinMax[i][1];
	}

}


}
