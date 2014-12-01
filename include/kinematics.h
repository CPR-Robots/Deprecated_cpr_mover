/*
 * kinematics.h
 *
 *  Created on: Nov 27, 2014
 *      Author: crm
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

namespace cpr_robots{

class kinematics {
public:
	kinematics();
	virtual ~kinematics();

	int nrOfJoints;
	double jointMaxVelocity[6];			// degree/sec, Provided by cpr_mover4
	double jointMinMax[6][2];			// Min and max values in degree

	void CheckJointMinMax(float * j);

};

}
#endif /* KINEMATICS_H_ */
