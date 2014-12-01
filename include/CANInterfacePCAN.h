/*
 * CANInterfacePCAN.h
 *
 *  Created on: 30.06.2014
 *      Author: crm
 */

#ifndef CANINTERFACEPCAN_H_
#define CANINTERFACEPCAN_H_


#include "libpcan.h"		// fuer PEAK PCAN-USB

struct msg{
	int id;				// message id
	int length;			// length of data part
	unsigned char data[8];			// data
	long time;			// receive time
};



class CANInterfacePCAN {
public:
	CANInterfacePCAN();
	virtual ~CANInterfacePCAN();

	bool flagStopThread;
	bool flagConnected;
	msg msgBuffer[256];
	HANDLE h;					// handle to the PCAN interface


	bool Connect();
	bool Disconnect();

	void WriteMessage(int id, int length, unsigned char* data);
	int GetLastMessage(int id, int *length, unsigned char* data);

private:

	
};

#endif /* CANINTERFACEPCAN_H_ */
