/*
 * ProtocolCPRCAN.h
 *
 *  Created on: Jul 7, 2014
 *      Author: crm
 */

#ifndef PROTOCOLCPRCAN_H_
#define PROTOCOLCPRCAN_H_

#include <unistd.h>			//  usleep()
#include <stdio.h>
#include <stdlib.h>
#include "CANInterfacePCAN.h"



class ProtocolCPRCAN {
public:
	ProtocolCPRCAN();
	virtual ~ProtocolCPRCAN();

	bool flagDoComm;
	int nrOfJoints;
	int jointIDs[6];
	float ticsPerDegree[6];		// encoder and gear constant
	float ticsZero[0];			// zero position

	CANInterfacePCAN itf;

	void Init(std::string robotType);
	bool Connect();
	bool Disconnect();
	bool GetConnectionStatus();

	void SetJoints(float * j);
	void GetJoints(float * j);
	std::string GetErrorMsg();

	void ResetJointsToZero();
	void ResetError();
	void EnableMotors();
	void DisableMotors();

	void SetIO(int joint, int which, bool state);

};

#endif /* PROTOCOLCPRCAN_H_ */
