/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Commonplace Robotics GmbH
 *  http://www.commonplacerobotics.com
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name Commonplace Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
// Created on: 	October 26th, 2014
// Last change: December 4th, 2014

/*
	Functionality:
	* Establishes a main loop to control the robot arm Mover4 or Mover6
	* Connects to a Mover robot using the PCAN adapter
	* Allows to move the joints with the cpr_rviz_plugin
	* Accepts joint_trajectory_actions from e.g. moveit
	* Publishes the current joint positions and the robot status

	ToDo:
	* Include the CPRRS232 USB-CAN-adapter
	* improve structure and quality, debug
*/


#include <cpr_mover.h>
#include <list>



using namespace std;

//typedef actionlib::SimpleActionServer<control_msgs::JointTrajectoryAction> Server;
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

double newJoints[] = {0.0, 0.0, 0.0, 0.0};	// in degree
std::list<robotState> targetPointList;		// die Liste der abzufahrenden Punkte

double deg2rad = 3.14159 / 180.0;
double rad2deg = 180.0 / 3.14159;


//**************************************************************
// print the points in the target list
// todo: lokal integrieren
void printTargetPointList(){
	ROS_INFO("Current targetPointList with %d points:", targetPointList.size());
	list<robotState>::iterator it;
	it=targetPointList.begin();
	int size = targetPointList.size();
	for(int i=0; i<size; i++){
		ROS_INFO("P%d: %.2lf %.2lf %.2lf, Duration: %lf s" , i, (*it).j[0], (*it).j[1], (*it).j[2], (*it).duration );
		it++;
	}

}


//***************************************************************************
// Processing and JointTrajectoryAction
// todo: lokal integrieren
//void execute(const control_msgs::JointTrajectoryGoalConstPtr& goal, Server* as)
void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as)
{
  	
  double pos = 3.0; 
  double rad2deg = 180.0 / 3.141;
  robotState rs;
  float lastDuration = 0.0;

  int nrOfPoints = goal->trajectory.points.size();					// Anzahl der einzulesenden Punkte
  for(int i=0; i<nrOfPoints; i++){
	  rs.j[0] = goal->trajectory.points[i].positions[0] * rad2deg;	// ros values come in rad, here we work in degree
	  rs.j[1] = goal->trajectory.points[i].positions[1] * rad2deg;
	  rs.j[2] = goal->trajectory.points[i].positions[2] * rad2deg;
	  rs.j[3] = goal->trajectory.points[i].positions[3] * rad2deg;
	  float dtmp = goal->trajectory.points[i].time_from_start.toSec();
	  rs.duration = dtmp - lastDuration;							// time_from_start is adding up, these values are only for the single motion
	  lastDuration = dtmp;

	  targetPointList.push_back(rs);
  }
  ROS_INFO("Trajectory with %d positions received \n", nrOfPoints);
  printTargetPointList();

  as->setSucceeded();
}


void quit(int sig)
{
  ros::shutdown();
  exit(0);
}



int main(int argc, char** argv)
{

	ros::init(argc, argv, "cpr_mover4");


	//Start the ActionServer for JointTrajectoryActions from MoveIT
	ros::NodeHandle n2;


	Server server(n2, "cpr_mover/follow_joint_trajectory", boost::bind(&execute, _1, &server), false);
  	ROS_INFO("ActionServer: Starting");
  	server.start();	



	// Start the robot
	cpr_robots::cpr_mover robot;
	robot.init();	
	robot.mainLoop();		//spinning is done inside the main loop			

  	signal(SIGINT,quit);	
	return(0);
}



namespace cpr_robots{


	

	//*************************************************************************************
	cpr_mover::cpr_mover(){

		flagMover4 = true;      // default choice
		flagMover6 = false;

		// whicht robot to operate?
		// should be defined in the launch file / parameter server
		//<param name="robot_type" value="mover4"/>
		std::string global_name, relative_name, default_param;
		if (n.getParam("/robot_type", global_name)){
			ROS_INFO(global_name.c_str());

			if(global_name == "mover4"){
				flagMover4 = true;
				flagMover6 = false;
			}else if(global_name == "mover6"){
				flagMover4 = false;
				flagMover6 = true;
			}else{
				flagMover4 = true;
				flagMover6 = false;
			}

		}else{
			ROS_INFO("no robot name found");
		}

	}


	//*************************************************************************************
	cpr_mover::~cpr_mover(){

	}

	//*************************************************************************************
	void cpr_mover::init(){
		ROS_INFO("...initing...");

		flag_stop_requested = false;

		if(flagMover6){
			nrOfJoints = 6;
			kin.nrOfJoints = 6;
			itf.Init("mover6");
		}else{
			nrOfJoints = 4;
			kin.nrOfJoints = 4;
			itf.Init("mover4");
		}

		setPointState.j[0] =   0.0;			// values are initialized with 6 field to be usable for Mover4 and Mover6
		setPointState.j[1] = -20.0;
		setPointState.j[2] =  20.0;
		setPointState.j[3] =  20.0;
		setPointState.j[4] =  30.0;
		setPointState.j[5] =   0.0;

		jointMaxVelocity[0] = 20.0;
		jointMaxVelocity[1] = 20.0;
		jointMaxVelocity[2] = 20.0;
		jointMaxVelocity[3] = 20.0;
		jointMaxVelocity[4] = 20.0;
		jointMaxVelocity[5] = 20.0;


		// when starting up (or when reading the HW joint values) the target position has to be aligned with the setPoint position
		for(int i=0; i<nrOfJoints; i++)
			targetState.j[i] = setPointState.j[i];

		ovrPercent = 50.0;
		cycleTime = 50.0;


		msgJointsCurrent.header.stamp = ros::Time::now();
		msgJointsCurrent.name.resize(6);
		msgJointsCurrent.position.resize(6);
		msgJointsCurrent.name[0] ="Joint0";
		msgJointsCurrent.position[0] = 0.0;
		msgJointsCurrent.name[1] ="Joint1";
		msgJointsCurrent.position[1] = 0.0;
		msgJointsCurrent.name[2] ="Joint2";
		msgJointsCurrent.position[2] = 0.0;
		msgJointsCurrent.name[3] ="Joint3";
		msgJointsCurrent.position[3] = 0.0;
		msgJointsCurrent.name[4] ="Joint4";
		msgJointsCurrent.position[4] = 0.0;
		msgJointsCurrent.name[5] ="Joint5";
		msgJointsCurrent.position[5] = 0.0;

		// Publish the current joint states
		pubJoints = n.advertise<sensor_msgs::JointState>("/joint_states", 1);

		for(int i=0; i<6; i++)
			cmdVelocities[i] = 0.0;

		subJointVel = n.subscribe<sensor_msgs::JointState>("/CPRMoverJointVel", 1, &cpr_mover::jointVelCallback, this);

		msgErrorStates.data = "error 0x04";
		pubErrorStates = n.advertise<std_msgs::String>("/CPRMoverErrorCodes", 1);


		subCommands = n.subscribe<std_msgs::String>("/CPRMoverCommands", 1, &cpr_mover::commandsCallback, this);
			

	}


	//****************************************************************
	void cpr_mover::mainLoop()
	{
  		ROS_INFO("Starting Mover Main Loop");
  		
 	 	for(;;)
  		{
			MotionGeneration();			// Generate the joint motion
			CommunicationHW();			// Forward the new setpoints to the hardware
			CommunicationROS();			// Publish the joint states and error info

			if(flag_stop_requested)
				break;

			ros::spinOnce();
			ros::Duration(cycleTime/1000.0).sleep();		// main loop with 20 Hz.
						
  		}
		ROS_INFO("Closing Mover Main Loop");

	} //endof mainLoop


	//*************************************************************************************
	// Here we receive the discrete commands like Connect, Reset, Enable
	// the commands are forwarded to the interface class
	void cpr_mover::commandsCallback(const std_msgs::String::ConstPtr& msg){

		//ROS_INFO("CMD: %s ", msg->data.c_str()) ;
		std::string rec = msg->data;


		if( rec == "Connect"){
			itf.Connect();
			ROS_INFO("Connect");
		}
		else if( rec == "Reset"){
			itf.GetJoints( setPointState.j );
			for(int i=0; i<nrOfJoints; i++)
				targetState.j[i] = setPointState.j[i];
			itf.ResetError();
			ROS_INFO("Reset and load joint position");
		}
		else if( rec == "Enable" ){
			itf.EnableMotors();
			ROS_INFO("Enable");
		}
		else if( rec == "GripperOpen" ){
			itf.SetIO(3, 1, true);
			itf.SetIO(3, 0, true);
			ROS_INFO("GripperOpen");
		}
		else if( rec == "GripperClose" ){
			itf.SetIO(3, 1, true);
			itf.SetIO(3, 0, false);
			ROS_INFO("GripperClose");
		}
		else if( rec[0] == 'O' && rec[1] == 'v' && rec[2] == 'e'){
			int l = rec.length();
			std::string ovr = rec.substr(9, l-1);
			int newovr = atoi(ovr.c_str());

			if(newovr > 100) newovr = 100;
			if(newovr < 0) newovr = 0;
			ovrPercent = newovr;

			ROS_INFO("New Override %d", newovr);
		}


	}

	//*************************************************************************************
	// receive joint velocity commands. Values in percent of maxVelocity, [-100..100]
	void cpr_mover::jointVelCallback(const sensor_msgs::JointState::ConstPtr& msg){
		double tmp = 0.0;
		int i=0;
		for(i=0; i<nrOfJoints; i++){
			tmp = msg->velocity[i];
			if(tmp < -100.0) tmp = -100.0;		// verify the limits
			if(tmp >  100.0) tmp =  100.0;
			cmdVelocities[i] = tmp;
		}

	}


	//*****************************************************************
	// Joint interplation from current to target position
	// currently all joints move with max velocity, will be changed 
	// targetState: the position the robot is aproaching
	// setPointState: the setPointPosition on the way to targetState
	// currentPosition: the real hardware position, probably with a small delay to setPointState
	void cpr_mover::MotionGeneration(){
		int i=0;
		double dist = 0.0;
		double maxMove = 0.0;

		bool flagDone = true;

		double delta = 0.0;
		for(int i=0; i<nrOfJoints; i++)
			delta += abs(targetState.j[i] - setPointState.j[i]);


		if( delta > 0.001 || targetPointList.size() > 0){		// Jog the robot. Only if there is no replay active

			if(!flagPointReplayInited)
				ComputeCurrentJointReplayVel(setPointState, targetState, jointReplayVel);

			for(int i=0; i<nrOfJoints; i++){
				dist = targetState.j[i] - setPointState.j[i];

				maxMove = (ovrPercent/100.0) * jointReplayVel[i] * (cycleTime / 1000.0);
				if(dist > maxMove) dist = maxMove;
				if(dist < -maxMove) dist = -maxMove;
				setPointState.j[i] += dist;
				if(abs(dist) > 0.001)				// arrival at target position?
					flagDone = false;
			}

		    // change to a new target position while the list is not empty
			if(flagDone){
				robotState rs;
				int size = targetPointList.size();
				if(size > 0){
					list<robotState>::iterator i;
					targetState = targetPointList.front();
					targetPointList.pop_front();
					flagPointReplayInited = false;
					ROS_INFO("New Position from List (remaining: %d): %.2lf %.2lf %.2lf %.2lf",targetPointList.size(), targetState.j[0], targetState.j[1], targetState.j[2], targetState.j[3]);

				}
			}

		}else{
			for(int i=0; i<nrOfJoints; i++)
				setPointState.j[i] += (cmdVelocities[i]/100.0) * (ovrPercent/100.0) * (jointMaxVelocity[i] * (cycleTime/1000.0));

			for(int i=0; i<nrOfJoints; i++)
				targetState.j[i] = setPointState.j[i];

		}



		kin.CheckJointMinMax(setPointState.j);		// check if the joints are above minmax values


		return;
	}



	//***************************************************************************
	void cpr_mover::ComputeCurrentJointReplayVel(robotState cp, robotState tp, double * vel){

		float dist[6];
		float duration[6];
		float maxDist = 0.0;
		float maxDuration = 0.0;
		for(int i=0; i<nrOfJoints; i++){
			dist[i] = abs(tp.j[i] - cp.j[i]);
			if( dist[i] > maxDist) maxDist = dist[i];
			duration[i] = abs(dist[i] / jointMaxVelocity[i]);
			if( duration[i] > maxDuration) maxDuration = duration[i];
		}

		float scale = 1.0;
		if(maxDuration < tp.duration)
			scale = maxDuration / tp.duration;

		for(int i=0; i<nrOfJoints; i++){
			vel[i] = jointMaxVelocity[i] * (duration[i] / maxDuration ) * scale;
		}

		flagPointReplayInited = true;

		ROS_INFO("Scale: %lf (%lf / %lf) -  %lf %lf %lf %lf", scale, maxDuration, tp.duration,  vel[0], vel[1], vel[2], vel[3]);


	}


	//***************************************************************
	// Forward the new setpoints to the hardware
	void cpr_mover::CommunicationHW(){

		if(!itf.GetConnectionStatus())		// if HW is not connected then skip the rest
			return;

		for(int i=0; i<nrOfJoints; i++){
			itf.SetJoints( setPointState.j );
		}

	}


	//***************************************************************
	// forward the current joints to RViz etc
	void cpr_mover::CommunicationROS(){

		static int pCnt = 0;

		msgJointsCurrent.header.stamp = ros::Time::now();
		msgJointsCurrent.position[0] = deg2rad * setPointState.j[0];		// Robot CAN communication works in degree
		msgJointsCurrent.position[1] = deg2rad * setPointState.j[1];
		msgJointsCurrent.position[2] = deg2rad * setPointState.j[2];
		msgJointsCurrent.position[3] = deg2rad * setPointState.j[3];
		msgJointsCurrent.position[4] = deg2rad * setPointState.j[4];
		msgJointsCurrent.position[5] = deg2rad * setPointState.j[5];
		pubJoints.publish(msgJointsCurrent);								// ROS communication works in Radian


		msgErrorStates.data = itf.GetErrorMsg();

		pubErrorStates.publish(msgErrorStates);


		pCnt++;				// nur in jedem 10. Schritt eine Ausgabe drucken
		if(pCnt > 20){
			//ROS_INFO("Joint (%d): %.2lf %.2lf %.2lf %.2lf",targetPointList.size(), setPointState.j[0], setPointState.j[1], setPointState.j[2], setPointState.j[3]);
			pCnt = 0;
		}
	}






}




