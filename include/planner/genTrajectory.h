/*
 * droneSystem.h
 *
 *  Created on: 16/09/2019
 *      Author: jrsbenevides
 */

#ifndef GENTRAJECTORY_H_
#define GENTRAJECTORY_H_


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include "angles/angles.h"
#include "drone/operations.h"
#include "drone/definitions.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/UInt8.h"
#include "drone/drone.h"
#include <cmath>
#include <sstream>
#include <string>
#include <iostream>

using namespace std;

namespace DRONE {

	class Planner {

	    ros::NodeHandle n;
	    ros::Publisher  waypoint_publisher;
	    ros::Publisher ackMessage_publisher;
	    ros::Subscriber joy_subscriber;
	    ros::Subscriber targetMsg_subscriber;
	    ros::Subscriber ackControl_subscriber;

	  private:

	  	bool    isControlStarted;
		bool    isFirstTimePass;	  
		bool 	flagGlobalPlanner;
		bool 	flagAbort;
		bool	flagAckParam;
		bool 	initOrbWithScale;

		int 	autoMode;
		
		int	 PLANNER_IDLE;
		int	 PLANNER_GO;
		int	 PLANNER_ABORT;
		int	 PLANNER_APROX;
		int	 PLANNER_ORBKILL;
		int	 PLANNER_PARAM;
		int	 PLANNER_GOPXL;
		int	 PLANNER_LAND;


		double 	PI;
		double 	t;
		double 	wAng;
	  
	  public:
		
		double 		startTime;
		double 		amplitude;
		double 		velMed;
		string 		trajectory;
		VectorFive 	poseDesired;
		Vector3axes cTx, cTy,cTz, cTyaw;
		double		tFinal;

		uint8_t		ackMsgGlobal;

		Planner();
		~Planner ();
		
		void initPlanner(void);
		void setTrajectory(const string& trajectoryInput);
		void setIsControlStarted(bool state);
		void setIsFirstTimePass(bool state);
		void setFlagGlobalPlanner(bool state);
		void setFlagAbort(bool state);
		void setFlagAckParam(bool state);
		void setStartTime(double timeValue);
		void setposeDesired(VectorFive poseDesiredValue);
		void setTrajectoryCoefficients(void);
		bool getIsControlStarted(void);
		bool getIsFirstTimePass(void);
		bool getFlagGlobalPlanner(void);
		bool getFlagAbort(void);
		bool getFlagAckParam(void);
		double getStartTime(void);
		VectorFive getposeDesired(void);
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		void planCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void ackControlCallback(const std_msgs::UInt8::ConstPtr& msg);
		void orbSlamNoScaleCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void loadTopics(ros::NodeHandle &n);
		void loadSettings(ros::NodeHandle &n);
		void refreshWang(void);
		void TrajPlanner(void);
		Vector12x1 getPlanTrajectory(const int& agent,const double& timeStamp);
		void angle2quatZYX(VectorQuat& q, const double& yaw, const double& pitch, const double& roll);
	};
} // namespace DRONE


#endif /* BEBOP_DEV_INCLUDE_DRONE_DRONESYSTEM_H_ */
