/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Commonplace Robotics GmbH
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
// V01.4 October 7th, 2016:	JointMinMax are set differently for M4 and M6, increased velocity
// V01.5 Nov 11th, 2016:	Different definition of Joint topic wih 6+2 resp. 4+2 values

/*
	Functionality:
	* Establishes a main loop to control the robot arm Mover4 or Mover6
	* Connects to a Mover robot using the PCAN adapter
	* Allows to move the joints with the cpr_rviz_plugin
	* Forwards joints and gripper joints in a joint_states message
	* Accepts joint_trajectory_actions from e.g. moveit
	* Accepts gripperCommandActions from e.g. moveit
	* Publishes the current joint positions and the robot status

	ToDo:
	* Include the CPRRS232 USB-CAN-adapter
	* improve structure and quality, debug
*/


#include <cpr_mover.h>
#include <list>



using namespace std;

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> TrajectoryServer;
typedef actionlib::SimpleActionServer<control_msgs::GripperCommandAction> GripperServer;

double newJoints[] = {0.0, 0.0, 0.0, 0.0};	// internal joint vlaues are in degree
std::list<robotState> targetPointList;		// list of points to move to
int gripperRequest = 0;						// Stores the GripperAction requests. 0: no request, 1: please open, 2: please close. The main loop resets this value to 0 when done.
double gripperJointStatus = 0.0;			// State of the gripper joints. This is a workaround, the real joints are commanded via digital io
double gripperJointMax = 35.0 * 3.14159 / 180.0;	// Opening width

double deg2rad = 3.14159 / 180.0;
double rad2deg = 180.0 / 3.14159;


//**************************************************************
// print the points in the target list
void printTargetPointList(){
	ROS_INFO("Current targetPointList with %d points:", (int)(targetPointList.size()));
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
void executeTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, TrajectoryServer* as)
{
  double pos = 3.0; 
  double rad2deg = 180.0 / 3.141;
  robotState rs;
  float lastDuration = 0.0;

  int nrOfPoints = goal->trajectory.points.size();					// Number of points to add
  for(int i=0; i<nrOfPoints; i++){
	  rs.j[0] = goal->trajectory.points[i].positions[0] * rad2deg;	// ros values come in rad, internally we work in degree
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

//***************************************************************************
// React on Gripper Commands
void executeGripper(const control_msgs::GripperCommandGoalConstPtr & goal, GripperServer* as)
{
	float gapSize = goal->command.position;

	if(gapSize > 0.0){
		gripperRequest = 1;
		  ROS_INFO("GripperAction: open");
	}
	else{
		gripperRequest = 2;
		  ROS_INFO("GripperAction: close");
	}
	as->setSucceeded();
}


//****************************************************
void quit(int sig)
{
  ros::shutdown();
  exit(0);
}



//******************** MAIN ************************************************
int main(int argc, char** argv)
{
	ros::init(argc, argv, "cpr_mover_robot");
	ros::NodeHandle n2;

	//Start the ActionServer for JointTrajectoryActions and GripperCommandActions from MoveIT
	TrajectoryServer tserver(n2, "cpr_mover/follow_joint_trajectory", boost::bind(&executeTrajectory, _1, &tserver), false);
  	ROS_INFO("TrajectoryActionServer: Starting");
  	tserver.start();
	GripperServer gserver(n2, "cpr_mover/gripper_command", boost::bind(&executeGripper, _1, &gserver), false);
 	ROS_INFO("GripperActionServer: Starting");
 	gserver.start();

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

		ROS_INFO("CPR-Mover Mainloop V01.5 Nov. 11th, 2016");
		flagMover4 = true;      // default choice
		flagMover6 = false;

		// whicht robot to operate?
		// should be defined in the launch file / parameter server
		//<param name="robot_type" value="mover4"/>
		std::string global_name, relative_name, default_param;
		if (n.getParam("/robot_type", global_name)){
			ROS_INFO("%s", global_name.c_str());

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
			kin.SetJointMinMax(1);
			itf.Init("mover6");

			msgJointsCurrent.header.stamp = ros::Time::now();
			msgJointsCurrent.name.resize(8);
			msgJointsCurrent.position.resize(8);
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
			msgJointsCurrent.name[6] ="Gripper1";		// Joints 7 and 8 are gripper joints
			msgJointsCurrent.position[6] = 0.0;
			msgJointsCurrent.name[7] ="Gripper2";
			msgJointsCurrent.position[7] = 0.0;
		}else{
			nrOfJoints = 4;
			kin.nrOfJoints = 4;
			kin.SetJointMinMax(0);
			itf.Init("mover4");

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
			msgJointsCurrent.name[4] ="Gripper1";		// Joints 5 and 6 are gripper joints
			msgJointsCurrent.position[4] = 0.0;
			msgJointsCurrent.name[5] ="Gripper2";
			msgJointsCurrent.position[5] = 0.0;
		}

		setPointState.j[0] =   0.0;			// values are initialized with 6 field to be usable for Mover4 and Mover6
		setPointState.j[1] = -20.0;
		setPointState.j[2] =  20.0;
		setPointState.j[3] =  20.0;
		setPointState.j[4] =  30.0;
		setPointState.j[5] =   0.0;

		jointMaxVelocity[0] = 40.0;
		jointMaxVelocity[1] = 40.0;
		jointMaxVelocity[2] = 40.0;
		jointMaxVelocity[3] = 40.0;
		jointMaxVelocity[4] = 40.0;
		jointMaxVelocity[5] = 40.0;


		// when starting up (or when reading the HW joint values) the target position has to be aligned with the setPoint position
		for(int i=0; i<nrOfJoints; i++)
			targetState.j[i] = setPointState.j[i];

		ovrPercent = 50.0;
		cycleTime = 50.0;
		

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
			MotionGeneration();			// Generate the joint motion and actuate the gripper
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
			gripperJointStatus = gripperJointMax;		// Workaround: the gripper should be in this position now, even if it is commanded by digital IO. This value is send to RViz
			ROS_INFO("GripperOpen");
		}
		else if( rec == "GripperClose" ){
			itf.SetIO(3, 1, true);
			itf.SetIO(3, 0, false);
			gripperJointStatus = 0.0;
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
					ROS_INFO("New Position from List (remaining: %d): %.2lf %.2lf %.2lf %.2lf",(int)(targetPointList.size()), targetState.j[0], targetState.j[1], targetState.j[2], targetState.j[3]);
				}
			}
		}else{
			for(int i=0; i<nrOfJoints; i++)
				setPointState.j[i] += (cmdVelocities[i]/100.0) * (ovrPercent/100.0) * (jointMaxVelocity[i] * (cycleTime/1000.0));

			for(int i=0; i<nrOfJoints; i++)
				targetState.j[i] = setPointState.j[i];
		}

		kin.CheckJointMinMax(setPointState.j);		// check if the joints are above minmax values

		// And then handle the gripper requests from the GripperServer
		if(gripperRequest == 1){
			itf.SetIO(3, 1, true);
			itf.SetIO(3, 0, true);
			gripperRequest = 0;
			gripperJointStatus = gripperJointMax;		// Workaround: the gripper should be in this position now, even if it is commanded by digital IO. This value is send to RViz
		}else if(gripperRequest == 2){
			itf.SetIO(3, 1, true);
			itf.SetIO(3, 0, false);
			gripperRequest = 0;
			gripperJointStatus = 0.0;
		}
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
		if(flagMover4){
			msgJointsCurrent.position[4] = gripperJointStatus;					// The two gripper joints. Workaround, in the Mover robots these are digital IO controlled
			msgJointsCurrent.position[5] = gripperJointStatus;
		}else{	//Mover6, two more joints
			msgJointsCurrent.position[4] = deg2rad * setPointState.j[4];
			msgJointsCurrent.position[5] = deg2rad * setPointState.j[5];
			msgJointsCurrent.position[6] = gripperJointStatus;					
			msgJointsCurrent.position[7] = gripperJointStatus;
		}
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




