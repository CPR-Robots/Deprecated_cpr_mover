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
	double lastJoints[6];				// joint values from the last step
	double jointMinMax[6][2];			// Min and max values in degree

	void SetJointMinMax(int robot);			// 0 for Mover4, 1 for Mover6
	void CheckJointMinMax(float * j);

};

}
#endif /* KINEMATICS_H_ */
