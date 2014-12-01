/*
 * CANInterfacePCAN.cpp
 *
 *  Created on: 30.06.2014
 *      Author: crm
 */

#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <string.h>


#include <fcntl.h> 			// fuer O_RDWR

#include <thread>
#include <pthread.h>

#include "CANInterfacePCAN.h"

using namespace std;




void wait2(int millisec){
	usleep( millisec * 1000);	// usleep needs unistd.h, value in microsec
}


//*****************************************************************
// Liest CAN-Nachrichten und speichert sie ab
void threadReadLoop(void * context)
{



	TPCANRdMsg msg;					// PEAK message structure
	msg.Msg.LEN = 0;
	int ret = 0;
	int i=0;


	CANInterfacePCAN *ctx;
	ctx = (CANInterfacePCAN*)context;

    sched_param sch;
    int policy;
    pthread_getschedparam(pthread_self(), &policy, &sch);

    std::cout<< "CANReadLoop is executing at priority " << sch.sched_priority << '\n';

    while(!ctx->flagStopThread){
    	//wait2(1000);
    	/*
		* Main Read loop
		*/

		if(ctx->flagConnected){
			ret = LINUX_CAN_Read_Timeout(ctx->h, &msg, 1000); /* wait 1 ms */
			//if(ret == CAN_ERR_OK){
			if(msg.Msg.LEN > 2){
				//cout << "CANReceive: " << ((int)msg.Msg.DATA[0]) << " Pos: " << ((int)msg.Msg.DATA[2]) << " " << ((int)msg.Msg.DATA[3])  << endl;
				int id = msg.Msg.ID;
				ctx->msgBuffer[id].length = msg.Msg.LEN;
				ctx->msgBuffer[id].id = id;
				for(i=0; i<8; i++)
					ctx->msgBuffer[id].data[i] = msg.Msg.DATA[i];
			}else{
				;
			}
		}else{
			wait2(100);
		}
    	//cout << "...\n";
    }

    std::cout << "CANReadLoop stopped" << endl;
}


CANInterfacePCAN::CANInterfacePCAN() {
	// TODO Auto-generated constructor stub

	// init the message buffer
	for(int i=0; i< 256; i++){
		msgBuffer[i].length = 0;
		msgBuffer[i].id = i;
		for(int j=0; j<8; j++)
			msgBuffer[i].data[j] = 0x80;
	}

	flagConnected = false;


	cout<<"PCAN interface created" << endl;
}


CANInterfacePCAN::~CANInterfacePCAN() {
	// TODO Auto-generated destructor stub
}

//**********************************************
bool CANInterfacePCAN::Connect(){


	string s;
	flagConnected = false;
	try{
		h = LINUX_CAN_Open("/dev/pcan32", O_RDWR);			// pcan32 for PCAN-USB adapter

		int status = CAN_Status(h);
		string sts = "";
		if(status == 0x0000 || status == 0x0020)			// ok oder RcBufferEmpty
			sts = "no error";
		else if( status == 0x0004 || status == 0x0008 || status == 0x0010)
			sts = "bus error";
		else if( status ==0x0100  || status == 0x2000 || status == 0x1C00 || status == 0x4000 || status == 0x8000)
			sts = "device error";
		else
			sts = "general error";

		if(sts == "no error"){
			flagConnected = true;
			s = "PCANUSB opened successfully: ";
			s += sts;
		}else{
			flagConnected = false;
			s = "could not connect to PCANUSB: ";
			s += sts;
		}



	}catch(std::exception e){
		s = "Could not connect to PCANUSB! "; // + string(e.what());

	}

	cout << s << endl;

	// Constructs the new thread and runs it. Does not block execution.
	thread t1(threadReadLoop, (void*)this);
	//Makes the main thread wait for the new thread to finish execution, therefore blocks its own execution.
	// compile with -std=c++11 or for gcc below 4.7: -std=c++0x -pthread
	// pthread.h einbinden


	t1.detach();		// startet den Thread parallel


	return flagConnected;
}

//**********************************************
bool CANInterfacePCAN::Disconnect(){

	flagConnected = false;
	flagStopThread = true;
	wait2(500);
	CAN_Close(h);
	cout<<"CAN itf: disconnected" << endl;
	return flagConnected;
}




//**********************************************
void CANInterfacePCAN::WriteMessage(int id, int length, unsigned char* data){

	TPCANMsg msg;					// PEAK message structure
	int ret = 0;

	if(!flagConnected)
		return;

	msg.ID = id;
	msg.LEN = (unsigned char)length;
	for (int i = 0; i < length; i++)
		msg.DATA[i] = data[i];
	msg.MSGTYPE = 0x00;				// only standard messages

	ret = CAN_Write(h, &msg);

	if (ret != CAN_ERR_OK){
		string s = "error in writing";
	}

	//cout << "CANWrite: "  << ((int)msg.DATA[2]) << " " << ((int)msg.DATA[3])  << endl;


	return;
}


//**********************************************
int CANInterfacePCAN::GetLastMessage(int id, int *length, unsigned char* data){

	if(id>255)
			throw std::string("invalid message id!");
	length[0] = msgBuffer[id].length;
	for(int i=0; i<8; i++)
		data[i] = msgBuffer[id].data[i];
	//m.time = msgBuffer[id].time;
	return 0;


}












