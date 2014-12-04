/*
 * ProtocolCPRCAN.cpp
 *
 *  Created on: Jul 7, 2014
 *      Author: crm
 */

#include <iostream>
#include "ProtocolCPRCAN.h"

using namespace std;


void wait3(int millisec){
	usleep( millisec * 1000);	// usleep braucht unistd.h und ist in microsec
}

ProtocolCPRCAN::ProtocolCPRCAN() {

	nrOfJoints = 4;

	jointIDs[0] = 16;
	jointIDs[1] = 32;
	jointIDs[2] = 48;
	jointIDs[3] = 64;

	ticsPerDegree[0] = -65.87;
	ticsPerDegree[1] = -65.87;
	ticsPerDegree[2] = 65.87;
	ticsPerDegree[3] = -101.0;

	for(int i=0; i<4; i++)
		ticsZero[i] = 32000;

}

ProtocolCPRCAN::~ProtocolCPRCAN() {
}


void ProtocolCPRCAN::Init(std::string robotType){

	if(robotType == "mover4"){

		nrOfJoints = 4;
		jointIDs[0] = 16;
		jointIDs[1] = 32;
		jointIDs[2] = 48;
		jointIDs[3] = 64;
		ticsPerDegree[0] = -65.87;
		ticsPerDegree[1] = -65.87;
		ticsPerDegree[2] = 65.87;
		ticsPerDegree[3] = -101.0;
		for(int i=0; i<4; i++)
			ticsZero[i] = 32000;

		std::cout << "ProtocolCPRCAN configured for mover4" << std::endl;


	}else if(robotType == "mover6"){

		nrOfJoints = 6;
		jointIDs[0] = 16;
		jointIDs[1] = 32;
		jointIDs[2] = 48;
		jointIDs[3] = 64;
		jointIDs[4] = 80;
		jointIDs[5] = 96;

		ticsPerDegree[0] = -65.87;
		ticsPerDegree[1] = -65.87;
		ticsPerDegree[2] = 65.87;
		ticsPerDegree[3] = -69.71;
		ticsPerDegree[4] = 3.2;
		ticsPerDegree[5] = 3.2;

		for(int i=0; i<4; i++)
			ticsZero[i] = 32000;
		ticsZero[4] = 512;				// these two values might need some adoptions if the joints are not algined correctly
		ticsZero[5] = 512;

		std::cout << "ProtocolCPRCAN configured for mover6" << std::endl;

	}


}

//*******************************************
bool ProtocolCPRCAN::Connect(){
	cout << "Protocol: Connect" << endl;
	if(!itf.flagConnected)
		itf.Connect();

	flagDoComm = itf.flagConnected;

	return itf.flagConnected;
}

//*******************************************
bool ProtocolCPRCAN::Disconnect(){
	if(itf.flagConnected)
		itf.Disconnect();

	return itf.flagConnected;
}


//******************************************
bool ProtocolCPRCAN::GetConnectionStatus(){

	if(itf.flagConnected)
		return true;
	else
		return false;
}




//*******************************************
// Expects an array of setpoint joint values in degree
void ProtocolCPRCAN::SetJoints(float * j){

	// storage necessary for the CAN communication
	int id = 16;
	int l = 5;				// length for position commands: 5 byte. for velocity commands 4 bytes
	unsigned char data[8] = {4, 125, 99, 0, 45};	// cmd, vel, posH, posL, counter

	// Write the desired set point position to the joints
	int tics = 0;
	if(flagDoComm){
		for(int i=0; i<nrOfJoints; i++){				// go through all four joints
			tics = ticsZero[i] + j[i] * ticsPerDegree[i];				// get the tics out of the joint position in degree
			data[2] = tics / 256;					// Joint Position High Byte
			data[3] = tics % 256;					// Joint Position Low Byte
			itf.WriteMessage(jointIDs[i], l, data);			// write the message to the joint controller
			wait3(2);						// wait a short time to avoid a crowded bus
			}
		}

}

//*****************************************************
void ProtocolCPRCAN::GetJoints(float * j){

	int l = 0;
	unsigned char data[8];

	for(int i=0; i<nrOfJoints; i++){
		itf.GetLastMessage( jointIDs[i]+1, &l, data );
		j[i] =  ((((float)((256*((unsigned int)data[2]))+((unsigned int)data[3]))) - ticsZero[i]) / ticsPerDegree[i]);
	}
}

//******************************************************
std::string ProtocolCPRCAN::GetErrorMsg(){

	std::string error;
	if(!itf.flagConnected)
		return "Not connected";


	int singleErrors[nrOfJoints];
	int allErrors = 0;

	int l = 0;
	unsigned char data[8];

	for(int i=0; i<nrOfJoints; i++){
		itf.GetLastMessage( jointIDs[i]+1, &l, data );
		singleErrors[i] = (int)data[0];
		allErrors = allErrors | singleErrors[i];
	}

	if(allErrors == 0){
		error.append("running");
	}else{
		if( allErrors & 0b00000001 ) error.append("WTD ");
		if( allErrors & 0b00000010 ) error.append("VelLag ");
		if( allErrors & 0b00000100 ) error.append("motorNotEnabled ");
		if( allErrors & 0b00001000 ) error.append("CommWatchDog ");
		if( allErrors & 0b00010000 ) error.append("PosLag ");
		if( allErrors & 0b00100000 ) error.append("Enc ");
		if( allErrors & 0b01000000 ) error.append("OvCurr ");
		if( allErrors & 0b10000000 ) error.append("CAN ");
	}
	//error = std::to_string(allErrors);

	return error;
}



//********************************************
// Resets the position EEPROM on the four joint controller
void ProtocolCPRCAN::ResetJointsToZero(){
	int l = 4;
	unsigned char data[8] = {1, 8, 125, 0, 0, 0, 0};	// CAN message to set the joint positions to zero

	flagDoComm = false;

	DisableMotors();			// otherwise the robot will make a jump afterwards (until the lag error stops him)

	for(int i=0; i<nrOfJoints; i++){
		itf.WriteMessage(jointIDs[i], l, data);	// first reset command.. but thats not sufficient
		wait3(5);
		itf.WriteMessage(jointIDs[i], l, data);	// the command has to be sent twice in the time of two seconds to take effect
		wait3(5);
	}
	cout << "Joints set to zero" << endl;

	flagDoComm = true;
}

//***********************************************************
// Send the robot the resetError messages
void ProtocolCPRCAN::ResetError(){
	int id = 16;
	int l = 2;
	unsigned char data[8] = {1, 6, 0, 0, 0, 0, 0};		// CAN message to reset the errors

	flagDoComm = false;

	for(int i=0; i<nrOfJoints; i++){
		itf.WriteMessage(jointIDs[i], l, data);
		wait3(3);
	}

	cout << "Reset joints" << endl;

	flagDoComm = true;
}


//********************************************
// enable the motors, necessary to move them
void ProtocolCPRCAN::EnableMotors(){
	int id = 16;
	int l = 2;
	unsigned char data[8] = {1, 9, 0, 0, 0, 0, 0};		// CAN message to enable the motors
	flagDoComm = false;
	for(int i=0; i<nrOfJoints; i++){
		itf.WriteMessage(jointIDs[i], l, data);
		wait3(3);
	}
	cout << "Motors enabled" << endl;
	flagDoComm = true;
}

//********************************************
void ProtocolCPRCAN::DisableMotors(){
	int id = 16;
	int l = 2;
	unsigned char data[8] = {1, 10, 0, 0, 0, 0, 0};		// CAN message to disable the motors
	flagDoComm = false;
	for(int i=0; i<nrOfJoints; i++){
		itf.WriteMessage(jointIDs[i], l, data);
		wait3(3);
	}
	cout << "Motors disabled" << endl;
	flagDoComm = true;
}

//***************************************************************
// Sets the digital Outputs
// in the base there are 3 (standard 5V) or 4 (24V DIO-Extension) outputs: joint = 0, which = 0 to 4
// in the TCP there are 2 outputs for the gripper: joint = 3, which = 0 and 1
// Gripper: the standard gripper need output 3/0 activated to be enabled, then 3/1 defines open and close
void ProtocolCPRCAN::SetIO(int joint, int which, bool state)
{

	int id = 16;
	int l = 2;
	unsigned char data[8] = {1, 10, 0, 0, 0, 0, 0};

	if (!itf.flagConnected)
	{
		cout << "cannot set DOut before connecting." << endl;
		return;
	}

	id = jointIDs[joint];
	data[0] = 0x01;
	if (which == 0)
		data[1] = 0x20;
	else if (which == 1)
		data[1] = 0x21;
	else if (which == 2)
		data[1] = 0x22;
	else if (which == 3)    // The third one is not always available
		data[1] = 0x23;
	else
	{
		cout << "ProtocolCPRCAN: Trying to set wrong IO" << endl;
		return;
	}

	if (state) data[2] = 0x01;
	else data[2] = 0x00;

	l = 3;

	flagDoComm = false;
	wait3(1);
	itf.WriteMessage(id, l, data);
	flagDoComm = true;
}

