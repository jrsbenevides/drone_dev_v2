/*
 * droneSystem.h
 *
 *  Created on: 16/06/2017
 *      Author: roberto
 */

#ifndef DRONESYSTEM_H_
#define DRONESYSTEM_H_


#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <string>

#include "drone/drone.h"
#include "network/networkEstimator.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include <geometry_msgs/PoseArray.h>
#include "sensor_msgs/Joy.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/UInt8.h"

#include "Eigen/Dense"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>


using namespace std;


namespace DRONE {

	class System {

	    ros::NodeHandle n;

		ros::Publisher cmd_vel_publisher;
		ros::Publisher cmd_global_publisher;
		ros::Publisher transfPosition_publisher;
		ros::Publisher ackControl_publisher;

		ros::Subscriber odom_subscriber;
		ros::Subscriber waypoint_subscriber;
		ros::Subscriber fix_subscriber;
		ros::Subscriber joy_subscriber;
		ros::Subscriber vicon_subscriber;
		ros::Subscriber orbslam_subscriber;
		ros::Subscriber targetMsg_subscriber;
		ros::Subscriber ackGlobal_subscriber;
		
	  private:
	  
	  	double wAng;	

	  public:
		
		System();
		~System ();

		geometry_msgs::PoseArray cmdArray;
		
		long int count;
		long int countEKF;
		int 	 countVicon;
		
		bool 	 flagEnable;
		bool 	 flagControllerStarted;
		bool 	 flagTwist;
		bool 	 flagGlobalPlanner;
		bool 	 flagGoPixel;
		bool 	 flagAckParam;
		bool 	 isOrbSlamDead;
		
		double 	 PI;
		double 	 vxAmpl;
		double 	 vyAmpl;
		double 	 vzAmpl;
		double 	 angAmpl;
		double 	 f;
		double 	 amplitude;
		double 	 velMed;

		int 	 autoMode;
		int 	 flagEmergency;

		//DEBUG
		int		contaEnvio;

		uint8_t  ackControlGlobal;

		int	 PLANNER_IDLE;
		int	 PLANNER_GO;
		int	 PLANNER_ABORT;
		int	 PLANNER_APROX;
		int  PLANNER_ORBLOST;
		int	 PLANNER_ORBKILL;
		int	 PLANNER_PARAM;
		int	 PLANNER_GOPXL;
		int	 PLANNER_LAND;

		int 	 flagZeroAuto;
		int 	 autoCounter;
		int 	 COUNTER_DELAY;
		int 	 COUNTER_DELAY_IMU;
		
		string 	 trajectory;
		string   controlSelect;
		string   sensorSelect;

		Drone    drone;
		
		Estimator network;

		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
		void waypointCallback(const nav_msgs::Odometry::ConstPtr& waypoint); //alterado
		void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& vicon);
		void orbSlamCallback(const nav_msgs::Odometry::ConstPtr& orbslam);
		void planCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void statusPlanCallback(const std_msgs::UInt8::ConstPtr& msg);
		void updateParameters(const string& sensorSelectInput);

		void globalToLocalPosition(const Vector3axes& positionValue, const VectorQuat& orientationValue, const Vector3axes& linearVelValue,const Vector3axes& angularVelValue);
		void initDroneSystemParam(void);
		void loadTopics(ros::NodeHandle &n);
		void loadSettings(ros::NodeHandle &n);
		void setTrajectory(const string& trajectoryInput);
		void setControlSelect(const string& controlSelectInput);
		void setSensorSelect(const string& sensorSelectInput);
		void setAmplitude(const double& amplitudeValue);
		void setVelMed(const double& velMedValue);
		void setFlagGlobalPlanner(bool state);
		void setFlagGoPixel(bool state);
		void setFlagAckParam(bool state);
		void setIsOrbSlamDead(bool state);
		bool getFlagAckParam(void);
		bool getFlagGlobalPlanner(void);
		bool getFlagGoPixel(void);
		bool getIsOrbSlamDead(void);
		void bootVicon(const double& timeValue);
		void bootOrbSlam(const double& timeValue);
		void testTimeout(void);
		void ncs();
		void control();
		void MAScontrol();
		Vector4d MASControlInput(const int& agent);
		
	};		
}


#endif /* BEBOP_DEV_INCLUDE_DRONE_DRONESYSTEM_H_ */
