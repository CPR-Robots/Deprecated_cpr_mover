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
// Created on: Oct. 26th 2013
// Last change: December 11th, 2014


#ifndef cpr_mover_H
#define cpr_mover_H

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <list>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <actionlib/server/simple_action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>

#include "kinematics.h"
#include "ProtocolCPRCAN.h"



struct robotState
{
	float p[6];		// cart position
	float j[6];		// joint position
	int errorCode[6];
	float duration;	// duration for motion; needed for actionServer
};

namespace cpr_robots{




// the struct stores information about the current robots state:
// joint positions, cartesian position and error code



class cpr_mover{

	private:

		bool flagMover4;				// which robot to use?
		bool flagMover6;


		bool flag_stop_requested;

		robotState setPointState;
	  	robotState currentState;
		robotState targetState;


		int nrOfJoints;					// 4 or 6
		double ovrPercent;				// [0..100]
		double jointMaxVelocity[6];			// degree/sec
		double cycleTime;				// in ms

		kinematics kin;
		ProtocolCPRCAN itf;				// The hardware interface

		bool flagPointReplayInited;
		double jointReplayVel[6];		// degree/sec


		double cmdVelocities[6];		// the commanded velocities via subJointVel

		ros::NodeHandle n;
		sensor_msgs::JointState msgJointsCurrent;		/**< the current joints */
		ros::Publisher  pubJoints;			/**< publishes the current joint positions  */
		ros::Subscriber subJointVel;		// Subscribes to joint velocity commands e.g. from the RViz plugin

		std_msgs::String msgCommands;
		ros::Subscriber subCommands;
		std_msgs::String msgErrorStates;
		ros::Publisher pubErrorStates;			/**< publishes the modules error codes  */


		void jointVelCallback(const sensor_msgs::JointState::ConstPtr& msg);
		void commandsCallback(const std_msgs::String::ConstPtr& msg);

		void MotionGeneration();
		void CommunicationHW();
		void CommunicationROS();

		void ComputeCurrentJointReplayVel(robotState cp, robotState tp, double * vel);

	

	public:
		cpr_mover();
		~cpr_mover();
		void init();
		void mainLoop();

		
		
};



}



#endif
