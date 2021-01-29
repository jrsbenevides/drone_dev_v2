/*
 * droneSystem.h
 *
 *  Created on: 28/12/2020
 *      Author: jrsbenevides
 */

#ifndef NETWORKESTIMATOR_H_
#define NETWORKESTIMATOR_H_


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include "angles/angles.h"
#include "drone/operations.h"
#include "drone/definitions.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/UInt8.h"
#include <cmath>
#include <sstream>
#include <string>
#include <iostream>

// using namespace std;
using namespace DRONE::Types;

namespace DRONE {

	class Estimator {

	    ros::NodeHandle n;
	    ros::Publisher  odomGlobal_publisher;
		ros::Subscriber odomRcv_subscriber;
	    ros::Subscriber joy_subscriber;


	  private:

	  	bool    isControlStarted;
		bool    isFirstTimePass;
		bool 	isReadyToSend;
		bool 	isReadyCompControl;

		int 	isCMHEenabled;
		int 	bfSize;

		double 	PI;
		double 	t;
	  
	  public:

	  	bool flagEnter, flagDebug;

	    int counter; //PARA DEBUG APENAS 
		int	nOfAgents;
		Vector8d estPose[5]; // 5 = nOfAgents


		int _EMPTY 		;
		int _RECEIVED 	;
		int _ESTIMATED	;
		int _DONE 		;

	  	double stepT, nextTimeToSend, updateRate;

		Vector8d K;

		Matrix8d A,R,S;
		Matrix8x4 B;

		Matrix4d Rotation;

		Matrix2d F,Q, P[5];  // 5 = nOfAgents

		struct GeneralParameters {   
			double t1;
			double tn;
			double tnbar;
			double sigmat;	
		}; 

	  	struct Buffer {   // Declare BUFFER struct type
			int index;   // Declare member types
			double tsSensor;
			double tsArrival;
			double tGSendCont;
			Vector8d data;	
			VectorQuat upre;
			VectorQuat upost;
		};   // Define object of type BUFFER

		GeneralParameters genParam[5]; //[] = nOfAgents

		Buffer bfTemp[5]; //[] = bfSize

		Buffer bfStruct[5][5][2]; //nOfAgents,bfSize, 2

		VectorFive rcvArray;
		VectorFive rcvArrayBuffer;

		Matrix2x5 estParam; // alpha;beta in each column

		Estimator();
		~Estimator();
		
		void 	initEstimator(void);
		void 	ComputeEstimation(void);
		void 	joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		void 	odomRcvCallback(const nav_msgs::Odometry::ConstPtr& odomRaw);
		void 	loadTopics(ros::NodeHandle &n);
		void 	loadSettings(ros::NodeHandle &n);
		bool 	AddPkt2Buffer(const Buffer& pkt, const int agent);
		void 	UpdateBuffer(const Buffer& pkt, const int agent,const int i);
		void 	updateEKF(const int agent);
		int 	nextAgentToCompute(void);
		int 	nextAgentToSend(void);
		void 	updateModel(void);
		void 	isSinsideTrapezoid(Vector2d& s, const Vector2d& sOld, int agent);
		
		void 		setK(const Vector8d& Kvalue);
		void 		setBuffer(const Buffer& msg);
		void 		setEstimatePose(const Vector8d& x,const int agent);
		void 		setFlagReadyToSend(const bool& flag);
		void 		setFlagComputeControl(const bool& flag);
		void 		setRcvArrayZero(void);
		void 		setCmdAgentDone(const int& agent);

		Vector8d 	getEstimatePose(const int agent);
		Vector8d 	getK(void);
		double 		getThisTimeSend(void);
		Buffer 		getBuffer(const int index);
		bool 		getFlagReadyToSend(void);
		bool 		getFlagComputeControl(void);
		

	};
} // namespace DRONE


#endif /* BEBOP_DEV_INCLUDE_DRONE_DRONESYSTEM_H_ */
