/*
 * droneSystem.h
 *
 *  Created on: 28/12/2020
 *      Author: jrsbenevides
 */

#ifndef NETWORKESTIMATOR_H_
#define NETWORKESTIMATOR_H_

#define _NOFAGENTS 1
#define _BFSIZE 5
#define _BUFMAXSIZE 3

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include "angles/angles.h"
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include "drone/operations.h"
#include "drone/definitions.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/UInt8.h"
#include <cmath>
#include <sstream>
#include <string>
#include <iostream>


typedef Matrix<bool, _NOFAGENTS,  1> 		 VectorBoolAgent;

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
		bool 	isFlagEnableMAS;
		bool 	flagSentToken;
		bool 	flagTickStart;
		bool 	flagEmergencyStop;
		bool 	flagReuseEstimation;

		int 	isCMHEenabled;

		double 	PI;
		double 	t;
	  
	  public:
		VectorBoolAgent 	isOdomStarted;
	  	bool flagEnter, flagDebug;

	    int counter; //PARA DEBUG APENAS 
		int	nOfAgents,bfSize,bufMaxSize;
		VectorQuat pose0[_NOFAGENTS];
		Matrix3d RotGlobal[_NOFAGENTS];
		double yaw0[_NOFAGENTS];
		Vector8d estPose[_NOFAGENTS]; // 1 = nOfAgents


		int _EMPTY 		;
		int _RECEIVED 	;
		int _ESTIMATED	;
		int _DONE 		;

	  	double stepT, nextTimeToSend, updateRate,coeffUpdRate;

		Vector8d K;

		Matrix8d A,R,S;
		Matrix8x4 B;

		Matrix4d Rotation,K2;

		Matrix2d F,Q, P[_NOFAGENTS];  // 5 = nOfAgents

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

		GeneralParameters genParam[_NOFAGENTS]; //[] = nOfAgents

		Buffer bfTemp[_BFSIZE]; //[] = bfSize

		Buffer bfTempPending[_BFSIZE][_BUFMAXSIZE]; //[] = bfSize,tamMaxBuffer

		Buffer bfStruct[_NOFAGENTS][_BFSIZE][2]; //nOfAgents,bfSize, 2

		Matrix<double,_BFSIZE,1> rcvArray;	  	//[] = bfSize
		Matrix<double,_BFSIZE,1> rcvArrayBuffer; 	//[] = bfSize

		Matrix<double,2,_NOFAGENTS>  estParam; // alpha;beta in each column for every agent

		Estimator();
		~Estimator();
		
		void 		initEstimator(void);
		void 		ComputeEstimation(void);
		void 		ComputeEstimation_identGlobal(void);
		void 		joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		void 		odomRcvCallback(const nav_msgs::Odometry::ConstPtr& odomRaw);
		void 		loadTopics(ros::NodeHandle &n);
		void 		loadSettings(ros::NodeHandle &n);
		bool 		AddPkt2Buffer(const Buffer& pkt, const int agent);
		void 		UpdateBuffer(const Buffer& pkt, const int agent,const int i);
		void 		updateEKF(const int agent);
		void 		updateEKF_identGlobal(const int agent);
		int 		nextAgentToCompute(void);
		int 		nextAgentToSend(void);
		void 		updateModel(void);
		Vector2d 	isSinsideTrapezoid(const Vector2d& s, const Vector2d& sOld, int agent);
		void 		PresentDebug(void);
		
		void 		setK(const Vector8d& Kvalue);
		void 		setBuffer(const Buffer& msg);
		void 		setBufferNext(const Buffer& msg);
		void 		setEstimatePose(const Vector8d& x,const int agent);
		void 		setFlagReadyToSend(const bool& flag);
		void 		setFlagComputeControl(const bool& flag);
		void 		setRcvArrayZero(void);
		void 		setCmdAgentDone(const int& agent, const Vector4d& input);
		void 		setToken(const bool& flag);
		void 		ResetForEstimPause(void);
		void 		setZeroAllBuffers(void);
		void 		setIsFlagEnable(const bool& value);
		void 		setReuseEstimate(const bool& value);
		void 		setFlagEmergencyStop(const bool& value);
		void 		setIsOdomStarted(const bool& value,const int& agent);
		void 		ZeroIsOdomStarted(void);
		void 		setPoseZero(const VectorQuat& poseValue, const int& agent);
		void 		setPosition(Vector3axes& position, const int& agent);
		double 		setOrientation (const VectorQuat& orientationValue, const int& agent);

		Vector8d 	getEstimatePose(const int agent);
		Vector8d 	getK(void);
		double 		getThisTimeSend(void);
		Buffer 		getBuffer(const int index);
		bool 		getFlagReadyToSend(void);
		bool 		getFlagComputeControl(void);
		bool 		getFlagEmergencyStop(void);
		bool 		getFlagEnter(void);
		bool 		getIsFlagEnable(void);
		bool 		getReuseEstimate(void);
		bool 		getIsOdomStarted(const int& agent);
		Matrix4d 	getK2(void);
		double 		getUpdateRate(void);
		
	};
} // namespace DRONE


#endif /* BEBOP_DEV_INCLUDE_DRONE_DRONESYSTEM_H_ */
