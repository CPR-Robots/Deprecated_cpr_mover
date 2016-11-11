/*
 * kinematics.cpp
 *
 *  Created on: Nov 27, 2014
 *      Author: crm
 */

#include "kinematics.h"

namespace cpr_robots{


kinematics::kinematics() {

	// Joint Min and Max values in degree, default values
	jointMinMax[0][0] = -150.0;		jointMinMax[0][1] = 150.0;
	jointMinMax[1][0] = -30.0;		jointMinMax[1][1] =  60.0;
	jointMinMax[2][0] = -110.0;		jointMinMax[2][1] = 60.0;
	jointMinMax[3][0] = -130.0;		jointMinMax[3][1] = 130.0;
	jointMinMax[4][0] = -90.0;		jointMinMax[4][1] = 90.0;
	jointMinMax[5][0] = -130.0;		jointMinMax[5][1] = 130.0;

}

kinematics::~kinematics() {
	// TODO Auto-generated destructor stub
}

void kinematics::SetJointMinMax(int robot){
	if(robot == 0){				// Mover4
		jointMinMax[0][0] = -150.0;		jointMinMax[0][1] = 150.0;
		jointMinMax[1][0] = -30.0;		jointMinMax[1][1] =  60.0;
		jointMinMax[2][0] = -40.0;		jointMinMax[2][1] = 140.0;
		jointMinMax[3][0] = -130.0;		jointMinMax[3][1] = 130.0;
		jointMinMax[4][0] = -90.0;		jointMinMax[4][1] = 90.0;
		jointMinMax[5][0] = -130.0;		jointMinMax[5][1] = 130.0;

	}else if(robot == 1){		// Mover6
		jointMinMax[0][0] = -150.0;		jointMinMax[0][1] = 150.0;
		jointMinMax[1][0] = -30.0;		jointMinMax[1][1] =  60.0;
		jointMinMax[2][0] = -110.0;		jointMinMax[2][1] = 60.0;
		jointMinMax[3][0] = -130.0;		jointMinMax[3][1] = 130.0;
		jointMinMax[4][0] = -90.0;		jointMinMax[4][1] = 90.0;
		jointMinMax[5][0] = -130.0;		jointMinMax[5][1] = 130.0;

	}
}


// prevents to move into the joint end stops, but allows to get out if you are in
void kinematics::CheckJointMinMax(float * j){

	double dir = 0.0;
	for(int i=0; i<nrOfJoints; i++){
		dir = j[i] - lastJoints[i];		// current motion direction

		if(j[i] < jointMinMax[i][0]){	// joint is below min
			if(dir > 0.0)
				;						// moving in positive direction is ok
			else
				j[i] = jointMinMax[i][0];	// moving in negative direction is not allowed
		}
		if(j[i] > jointMinMax[i][1]){
			if(dir < 0.0)
				;
			else
				j[i] = jointMinMax[i][1];
		}
	}

	for(int i=0; i<nrOfJoints; i++){		// copy the current joint to remember in the next step
		lastJoints[i] = j[i];
	}

}


}
