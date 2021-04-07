/*
 * genTrajectory.cpp
 *
 *  Created on: Sep 1, 2017
 *      Author: João Benevides
 *      Modified: João Benevides - Sep 16th 2019
 */

#include "planner/genTrajectory.h"

namespace DRONE {

	Planner::Planner() {

		initPlanner();
		
		loadTopics(n);
		loadSettings(n);

		setTrajectoryCoefficients();

		refreshWang();

   		srand (time(NULL)); /* initialize random seed: */
	}

	Planner::~Planner () {

	}

	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	/* ########################################                 SETTERS                 ##########################################*/
	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/

	void Planner::setIsControlStarted(bool state){
		isControlStarted = state;
	}

	void Planner::setIsFirstTimePass(bool state){
		isFirstTimePass = state;
	}

	void Planner::setFlagGlobalPlanner(bool state){
		flagGlobalPlanner = state;
	}

	void Planner::setFlagAbort(bool state){
		flagAbort = state;
	}

	void Planner::setFlagAckParam(bool state){
		flagAckParam = state;
	}
	void Planner::setposeDesired(VectorFive poseDesiredValue){
		
		poseDesired = poseDesiredValue;
	
	}

	void Planner::setStartTime(double timeValue){
		
		startTime = timeValue;
	
	}
	

	void Planner::setTrajectory(const string& trajectoryInput){

		string DEFAULT_TRAJECTORY = "straightLine";

		if(trajectoryInput.compare("eightShape") == 0){
			trajectory = "eightShape";
		} else if(trajectoryInput.compare("circleXY") == 0){
			trajectory = "circleXY";
		} else if(trajectoryInput.compare("circleZXY") == 0){
			trajectory = "circleZXY";
		} else if(trajectoryInput.compare("ident") == 0){
			trajectory = "ident";
		} else if(trajectoryInput.compare("straightLine") == 0){
			trajectory = "straightLine";
		} else if(trajectoryInput.compare("wayPoint") == 0){
			trajectory = "wayPoint";
		} else if(trajectoryInput.compare("goToPixelCenter") == 0){
			trajectory = "goToPixelCenter";			
		} else {
			trajectory = DEFAULT_TRAJECTORY;
		}
	}

	void Planner::setTrajectoryCoefficients(void){
		double x,y,z,yaw,tf;

		double tf3,tf4,tf5;

		Vector3axes pDes;

		x 	= poseDesired(0);
		y 	= poseDesired(1);
		z 	= poseDesired(2);
		yaw = poseDesired(3);
		tf 	= poseDesired(4);
		
		pDes << poseDesired.head(3);

		tFinal = tf;

		tf3 	 = tf*tf*tf;
		tf4 	 = tf3*tf;
		tf5 	 = tf4*tf;
		
		cTx(0) 	 = 10*x/tf3;
		cTx(1) 	 = -15*x/tf4;
		cTx(2) 	 = 6*x/tf5;

		cTy(0) 	 = 10*y/tf3;
		cTy(1) 	 = -15*y/tf4;
		cTy(2) 	 = 6*y/tf5;

		cTz(0) 	 = 10*z/tf3;
		cTz(1) 	 = -15*z/tf4;
		cTz(2) 	 = 6*z/tf5;

		cTyaw(0) = 10*yaw/tf3;
		cTyaw(1) = -15*yaw/tf4;
		cTyaw(2) = 6*yaw/tf5;		

		if((pDes.norm()<0.01)&&(yaw > 0.05))
			ackMsgGlobal = (ackMsgGlobal & (~0x30))|0x30; //ha rotacao!
		else
			ackMsgGlobal = (ackMsgGlobal & (~0x30))|0x00; //nao ha rotacao!

	}

	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	/* ########################################                 GETTERS                 ##########################################*/
	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	
	bool Planner::getIsControlStarted(void){
		return isControlStarted;
	}

	VectorFive Planner::getposeDesired(void){
		
		return poseDesired;
	}

	bool Planner::getIsFirstTimePass(void){
		return isFirstTimePass;
	}

	bool Planner::getFlagGlobalPlanner(void){
		return flagGlobalPlanner;
	}

	bool Planner::getFlagAbort(void){
		return flagAbort;
	}	

	bool Planner::getFlagAckParam(void){
		return flagAckParam;
	}	

	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	/* #####################################            REGULAR FUNCTIONS                 ########################################*/
	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: initPlanner
	*	  Created by: jrsbenevides
	*  Last Modified: jrsbenevides
	*
	*  	 Description: 1. Initialize essential functions for trajectory tracking;
	*				  2. Initialize parameters and default values;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Planner::initPlanner(void){

		cout << "Starting Trajectory Planner ...\n" << endl;

		autoMode			= 0;
		PI 					= 3.141592653589793;
   		t 			  		= 0.0;
		amplitude 	  		= 0.4;
		velMed        		= 0.5; 
		wAng 		  		= 2*PI*velMed/(6.097*amplitude);
		trajectory    		= "straightLine";
		startTime 	  		= ros::Time::now().toSec();
		poseDesired			= poseDesired.Zero();

		ackMsgGlobal		= 0x00;

		PLANNER_IDLE 	= 0;
		PLANNER_GO   	= 10;
		PLANNER_LAND	= 30;
		PLANNER_ORBKILL = 44;
		PLANNER_APROX  	= 55;
		PLANNER_PARAM 	= 77;
		PLANNER_GOPXL 	= 88;
		PLANNER_ABORT	= 100;

		initOrbWithScale = false;
		
		setIsControlStarted(false);
		setIsFirstTimePass(true);
		setFlagGlobalPlanner(true);
		setFlagAckParam(false);
		setFlagAbort(false);
	}	

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: load Topics
	*	  Created by: jrsbenevides
	*  Last Modified: jrsbenevides
	*
	*  	 Description: 1. Define the ROS Topics and its types of messages;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Planner::loadTopics(ros::NodeHandle &n) {

		waypoint_publisher 	  = n.advertise<nav_msgs::Odometry>("/drone/waypoint", 1);
		joy_subscriber 	   	  = n.subscribe<sensor_msgs::Joy>("/drone/joy", 1, &Planner::joyCallback, this);
		ackMessage_publisher  = n.advertise<std_msgs::UInt8>("statusPlanning", 1);
		targetMsg_subscriber  = n.subscribe<geometry_msgs::PoseStamped>("planejamento", 1, &Planner::planCallback, this);
		ackControl_subscriber = n.subscribe<std_msgs::UInt8>("statusAutoControl", 1, &Planner::ackControlCallback, this);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: load Settings 
	*	  Created by: jrsbenevides
	*  Last Modified: jrsbenevides
	*
	*  	 Description: 1. Loads config parameters and loads them into the program by substituting previously created variables.
	*                    Those can be edited in "config/bebopParameters.yaml"
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Planner::loadSettings(ros::NodeHandle &n) {

		if (n.getParam("/drone/amplitude",amplitude)) {
			cout << "amplitude = " << amplitude << endl;
		}

		if (n.getParam("/drone/velMed",velMed)) {
			cout << "velMed = " << velMed << endl;
		}

		string trajectory;
		if (n.getParam("/drone/trajectory",trajectory)) {
			setTrajectory(trajectory);
			cout << "trajectory = " << trajectory << endl;

		}
		
		vector<double> poseDesired;
		if (n.getParam("/drone/poseDesired",poseDesired)) {
			setposeDesired(VectorFive::Map(&poseDesired[0],5));
			cout << "poseDesired = " << getposeDesired() << endl;
		}	
		if (n.getParam("/drone/autoMode",autoMode)) {
			cout << "autoMode = " << autoMode << endl;
		}	
	}

	// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// /* 		Function: ackControlStatus Callback
	// *	  Created by: jrsbenevides
	// *  Last Modified: 
	// *
	// *  	 Description: According to the ack message sent from controller to planner, enable autonomous flight for planner.
	// *                 CONTROL ON : 0xFF;
	// *                 CONTROL OFF: 0x00;
	// */
	// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		

	// void Planner::ackControlStatusCallback(const std_msgs::UInt8::ConstPtr& ackControlStatus){
	// 	if(autoMode == 0)
	// 	  /*Enables "Automatic Control Mode" while pressing this button*/
	// 	  if(ackControlStatus.data == 0x00){
	// 	    setIsControlStarted(false);
	// 	  }
	// 	  else if(ackControlStatus.data == 0xFF){
	// 		setIsControlStarted(true);
	// 	  }
	// 	}
	// }

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: joyCallback
	*	  Created by: jrsbenevides
	*  Last Modified: 
	*
	*  	 Description: Unused buttons were used to special functions. These features are described below:
	*				  1. Button [14] = 'DIGITAL DOWN' = is responsible for resetting flag isOdomStarted. It allows the resetting of local
						 frame. The flag is raised again by a function "setPosition0()" after reset is done.
	*				  2. Button [6]  = 'SELECT' =  when kept pressed, allows controller to run.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		

	void Planner::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
		if(autoMode == 0){
		  /*Enables "Automatic Control Mode" while pressing this button*/
		  if(joy->buttons[6]){
		    setIsControlStarted(true);
		  }
		  else{
			setIsControlStarted(false);
		  }
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: refreshWang
	*	  Created by: jrsbenevides
	*  Last Modified: 
	*
	*  	 Description: Based on the chosen trajectory, this will update wAng, which should be the desired angular frequency.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		

	void Planner::refreshWang(void){

		if(trajectory.compare("eightShape") == 0){
			wAng = 2*PI*velMed/(6.097*amplitude);
		}
		else {
			wAng = velMed/amplitude;
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: angle2quatZYX
	*	  Created by: rsinoue
	*  Last Modified: 
	*
	*  	 Description: 1. Converts Euler angles into quaternions
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		

	void Planner::angle2quatZYX(VectorQuat& q, const double& yaw, const double& pitch, const double& roll){
		// q = [w, x, y, z]'
		// Avaliar depois se vale a pena otimizar...
		//Verificado em 16/02/18
		q(0) = cos(0.5*yaw)*cos(0.5*pitch)*cos(0.5*roll) + sin(0.5*yaw)*sin(0.5*pitch)*sin(0.5*roll);
		q(1) = cos(0.5*yaw)*cos(0.5*pitch)*sin(0.5*roll) - sin(0.5*yaw)*sin(0.5*pitch)*cos(0.5*roll);
		q(2) = cos(0.5*yaw)*sin(0.5*pitch)*cos(0.5*roll) + sin(0.5*yaw)*cos(0.5*pitch)*sin(0.5*roll);
		q(3) = sin(0.5*yaw)*cos(0.5*pitch)*cos(0.5*roll) - cos(0.5*yaw)*sin(0.5*pitch)*sin(0.5*roll);	
	}	

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: TrajPlanner
	*	  Created by: jrsbenevides
	*  Last Modified: 
	*
	*  	 Description: Trajectory Planner
	*		   Steps: 1. Starts time count right after activating automatic control. This will also start 
	*			 OBS: Improvements for 'ident' trajectory  are coming for next version.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		

	void Planner::TrajPlanner(void)
	{
		std_msgs::UInt8 ackMsg;
		nav_msgs::Odometry mGoal;
		VectorQuat quatDesired;
		quatDesired = quatDesired.Zero();
		float t2,t3,t4,t5;
		double yaw_desired; 

	    if(getIsControlStarted()){
	    	if(getIsFirstTimePass()){
	    		cout << "Starting Clock Now..." << endl;
	    		startTime = ros::Time::now().toSec();
				setIsFirstTimePass(false);
	    	}

	    	t = ros::Time::now().toSec() - startTime;

	    	if(trajectory.compare("eightShape") == 0){
				
	    		cout << "Eight-Shaped Trajectory" << endl;

				mGoal.pose.pose.position.x = amplitude*sin(wAng*t);
				mGoal.pose.pose.position.y = 0.5*amplitude*sin(2*wAng*t);
				mGoal.pose.pose.position.z = 0.0;

				mGoal.twist.twist.linear.x = wAng*amplitude*cos(wAng*t);
				mGoal.twist.twist.linear.y = wAng*amplitude*cos(2*wAng*t);
				mGoal.twist.twist.linear.z = 0.0;

				mGoal.pose.pose.orientation.x = 0.0;
				mGoal.pose.pose.orientation.y = 0.0;
				mGoal.pose.pose.orientation.z = 0.0;
				mGoal.pose.pose.orientation.w = 1.0;

				mGoal.twist.twist.angular.x    = 0.0;
				mGoal.twist.twist.angular.y    = 0.0;
				mGoal.twist.twist.angular.z    = 0.0;			
			}
			else if(trajectory.compare("circleXY") == 0){

				cout << "circle XY - Trajectory" << endl;
			
				mGoal.pose.pose.position.x = amplitude*cos(wAng*t);    
				mGoal.pose.pose.position.y = amplitude*sin(wAng*t);
				mGoal.pose.pose.position.z = 0;

				mGoal.twist.twist.linear.x = -wAng*amplitude*sin(wAng*t);
				mGoal.twist.twist.linear.y = wAng*amplitude*cos(wAng*t);
				mGoal.twist.twist.linear.z = 0;

				mGoal.pose.pose.orientation.x = 0.0;
				mGoal.pose.pose.orientation.y = 0.0;
				mGoal.pose.pose.orientation.z = 0.0;
				mGoal.pose.pose.orientation.w = 1.0;

				mGoal.twist.twist.angular.x    = 0.0;
				mGoal.twist.twist.angular.y    = 0.0;
				mGoal.twist.twist.angular.z    = 0.0;	

			} else if(trajectory.compare("ident") == 0){

				cout << "Ident - Trajectory" << endl;

				if(t < 40)
				{
					mGoal.pose.pose.position.x = 0.5*amplitude*(cos(wAng*t)+cos(t));    
					mGoal.pose.pose.position.y = 0.5*amplitude*(sin(wAng*t)+sin(t)); 
					mGoal.pose.pose.position.z = 0;

					mGoal.twist.twist.linear.x = -0.5*amplitude*(wAng*sin(wAng*t)+sin(t));
					mGoal.twist.twist.linear.y = 0.5*amplitude*(wAng*cos(wAng*t)+cos(t));
					mGoal.twist.twist.linear.z = 0;

					mGoal.pose.pose.orientation.x = 0.0;
					mGoal.pose.pose.orientation.y = 0.0;
					mGoal.pose.pose.orientation.z = 0.0;
					mGoal.pose.pose.orientation.w = 1.0;

					mGoal.twist.twist.angular.x    = -0.5*amplitude*(wAng*wAng*cos(wAng*t)+cos(t));
					mGoal.twist.twist.angular.y    = -0.5*amplitude*(wAng*wAng*sin(wAng*t)+sin(t));
					mGoal.twist.twist.angular.z    = 0.0;

				} else if(t < 80){
										
					mGoal.pose.pose.position.x = 0;    
					mGoal.pose.pose.position.y = 0;
					mGoal.pose.pose.position.z = 0.5*amplitude*(sin(wAng*(t-40)+sin(t-40)));

					mGoal.twist.twist.linear.x = 0;
					mGoal.twist.twist.linear.y = 0;
					mGoal.twist.twist.linear.z = 0.5*amplitude*(wAng*cos(wAng*(t-40))+cos(t-40));

					mGoal.pose.pose.orientation.x = 0.0;
					mGoal.pose.pose.orientation.y = 0.0;
					mGoal.pose.pose.orientation.z = 0.0;
					mGoal.pose.pose.orientation.w = 1.0;

					mGoal.twist.twist.angular.x    = 0;
					mGoal.twist.twist.angular.y    = 0;
					mGoal.twist.twist.angular.z    = -0.5*amplitude*(wAng*wAng*sin(wAng*(t-40))+sin(t-40));

				} else{
										
					mGoal.pose.pose.position.x = 0;    
					mGoal.pose.pose.position.y = 0;
					mGoal.pose.pose.position.z = 0;

					mGoal.twist.twist.linear.x = 0;
					mGoal.twist.twist.linear.y = 0;
					mGoal.twist.twist.linear.z = 0;

					yaw_desired = angles::normalize_angle(0.5*1.05*(sin(wAng*(t-80))+sin(t-80)));
					
					angle2quatZYX(quatDesired, yaw_desired, 0.0 , 0.0);

					mGoal.pose.pose.orientation.w = quatDesired(0);
					mGoal.pose.pose.orientation.x = quatDesired(1);
					mGoal.pose.pose.orientation.y = quatDesired(2);
					mGoal.pose.pose.orientation.z = quatDesired(3);

					mGoal.twist.twist.angular.x    = -0.5*1.05*(wAng*wAng*sin(wAng*(t-80))+sin(t-80));
					mGoal.twist.twist.angular.y    = 0.0;
					mGoal.twist.twist.angular.z    = 0.5*1.05*(wAng*cos(wAng*(t-80))+cos(t-80));
				}							
			} else if(trajectory.compare("circleZXY") == 0){

				cout << "circle ZXY- Trajectory" << endl;

				mGoal.pose.pose.position.x = amplitude*cos(wAng*t);    
				mGoal.pose.pose.position.y = amplitude*sin(wAng*t);
				mGoal.pose.pose.position.z = amplitude*sin(wAng*t);

				mGoal.twist.twist.linear.x = -wAng*amplitude*sin(wAng*t);
				mGoal.twist.twist.linear.y = wAng*amplitude*cos(wAng*t);
				mGoal.twist.twist.linear.z = wAng*amplitude*cos(wAng*t);				

				mGoal.pose.pose.orientation.x = 0.0;
				mGoal.pose.pose.orientation.y = 0.0;
				mGoal.pose.pose.orientation.z = 0.0;
				mGoal.pose.pose.orientation.w = 1.0;

				mGoal.twist.twist.angular.x    = 0.0;
				mGoal.twist.twist.angular.y    = 0.0;
				mGoal.twist.twist.angular.z    = 0.0;		

			} else if(trajectory.compare("straightLine") == 0){

				cout << "straightLine - Trajectory" << endl;
				if(t <= poseDesired(4)){
					
					t2 = t*t;
					t3 = t2*t;
					t4 = t3*t;
					t5 = t4*t;
					
					mGoal.pose.pose.position.x = cTx(0)*t3 + cTx(1)*t4 + cTx(2)*t5; 
					mGoal.pose.pose.position.y = cTy(0)*t3 + cTy(1)*t4 + cTy(2)*t5;
					mGoal.pose.pose.position.z = cTz(0)*t3 + cTz(1)*t4 + cTz(2)*t5;

					mGoal.twist.twist.linear.x = 3*cTx(0)*t2 + 4*cTx(1)*t3 + 5*cTx(2)*t4; 
					mGoal.twist.twist.linear.y = 3*cTy(0)*t2 + 4*cTy(1)*t3 + 5*cTy(2)*t4;
					mGoal.twist.twist.linear.z = 3*cTz(0)*t2 + 4*cTz(1)*t3 + 5*cTz(2)*t4;

					yaw_desired = angles::normalize_angle(cTyaw(0)*t3 + cTyaw(1)*t4 + cTyaw(2)*t5);
					
					angle2quatZYX(quatDesired, yaw_desired, 0.0 , 0.0);

					mGoal.pose.pose.orientation.w = quatDesired(0);
					mGoal.pose.pose.orientation.x = quatDesired(1);
					mGoal.pose.pose.orientation.y = quatDesired(2);
					mGoal.pose.pose.orientation.z = quatDesired(3);

					mGoal.twist.twist.angular.x = 0; 
					mGoal.twist.twist.angular.y = 0;
					mGoal.twist.twist.angular.z = 3*cTyaw(0)*t2 + 4*cTyaw(1)*t3 + 5*cTyaw(2)*t4;

				} else {

					mGoal.pose.pose.position.x = poseDesired(0); 
					mGoal.pose.pose.position.y = poseDesired(1);
					mGoal.pose.pose.position.z = poseDesired(2);

					mGoal.twist.twist.linear.x = 0; 
					mGoal.twist.twist.linear.y = 0;
					mGoal.twist.twist.linear.z = 0;

					yaw_desired = angles::normalize_angle(poseDesired(3));
					
					angle2quatZYX(quatDesired, yaw_desired, 0.0 , 0.0);

					mGoal.pose.pose.orientation.w = quatDesired(0);
					mGoal.pose.pose.orientation.x = quatDesired(1);
					mGoal.pose.pose.orientation.y = quatDesired(2);
					mGoal.pose.pose.orientation.z = quatDesired(3);

					mGoal.twist.twist.angular.x    = 0.0;
					mGoal.twist.twist.angular.y    = 0.0;
					mGoal.twist.twist.angular.z    = 0.0;
				}
			} else if(trajectory.compare("wayPoint") == 0){

				cout << "wayPoint - Trajectory" << endl;
				
				// Position Desired
				mGoal.pose.pose.position.x = poseDesired(0); 
				mGoal.pose.pose.position.y = poseDesired(1);
				mGoal.pose.pose.position.z = poseDesired(2);

				// Linear Velocity Desired
				mGoal.twist.twist.linear.x = 0; 
				mGoal.twist.twist.linear.y = 0;
				mGoal.twist.twist.linear.z = 0;

				// Quaternioin Orientation Desired
				mGoal.pose.pose.orientation.x = 0.0;
				mGoal.pose.pose.orientation.y = 0.0;
				mGoal.pose.pose.orientation.z = 0.0;
				mGoal.pose.pose.orientation.w = 1.0;

				// Angular Velocity Desired
				mGoal.twist.twist.angular.x    = 0.0;
				mGoal.twist.twist.angular.y    = 0.0;
				mGoal.twist.twist.angular.z    = 0.0;
			} else if(trajectory.compare("goToPixelCenter") == 0){

				cout << "Centralizando Pixel - Trajectory" << endl;
				
				// Position Desired
				mGoal.pose.pose.position.x = poseDesired(0); 
				mGoal.pose.pose.position.y = poseDesired(1);
				mGoal.pose.pose.position.z = poseDesired(2);

				// Linear Velocity Desired
				mGoal.twist.twist.linear.x = 0; 
				mGoal.twist.twist.linear.y = 0;
				mGoal.twist.twist.linear.z = 0;

				// Quaternioin Orientation Desired
				mGoal.pose.pose.orientation.x = 0.0;
				mGoal.pose.pose.orientation.y = 0.0;
				mGoal.pose.pose.orientation.z = 0.0;
				mGoal.pose.pose.orientation.w = 1.0;

				// Angular Velocity Desired
				mGoal.twist.twist.angular.x    = 0.0;
				mGoal.twist.twist.angular.y    = 0.0;
				mGoal.twist.twist.angular.z    = 0.0;
			}
	    } else{
	    	setIsFirstTimePass(true);
	    }

	    cout << "Current Time: " << t << endl;

	    if(autoMode == 1){
	    	if(getIsControlStarted()){  //Controle habilitado e pronto para usar!
		    	if (t <= tFinal){
					ackMsgGlobal = (ackMsgGlobal & (~0x0F))|0x0F;
					cout << "BUSY"<< endl;
				}
				else{
					ackMsgGlobal = (ackMsgGlobal & (~0x0F))|0x0A;
					cout << "FINISHED"<< endl;
				}
	    	}
	    	else{
	    		if(getFlagAbort()){
	    			ackMsgGlobal = (ackMsgGlobal & (~0x0F))|0x05;
	    			cout << "ABORTED" << endl;
	    		}
	    		else{
	    			ackMsgGlobal = (ackMsgGlobal & (~0x0F))|0x00;
	    			cout << "IDLE"<< endl;
	    		}
	    	}
			ackMsg.data = ackMsgGlobal;
			ackMessage_publisher.publish(ackMsg);	
	    }
		

		waypoint_publisher.publish(mGoal); //verificar!!!!
		
	}


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: getPlanTrajectory
	*	  Created by: jrsbenevides
	*  Last Modified: 
	*
	*  	 Description: Trajectory Planner for a multi-agent system
	*		   Steps: 1. Time count is "started" with setTimeStart()
	*				  2. For a given agent, in a given time, calculates the waypoints, including velocity and acceleration in the form
	*                 waypoint = x_d, y_d, z_d, psi_d, dx_d, dy_d, dz_d, dpsi_d, d2x_d, d2y_d, d2z_d, d2psi_d;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		

	Vector12x1 Planner::getPlanTrajectory(const int& agent,const double& timeValue)	{

	std_msgs::UInt8 ackMsg;
	nav_msgs::Odometry mGoal;
	Vector12x1 waypoint;
	VectorQuat quatDesired;
	quatDesired = quatDesired.Zero();
	float t2,t3,t4,t5;
	double yaw_desired; 

	t = timeValue - startTime;
	

	if(trajectory.compare("eightShape") == 0){
		
		cout << "Eight-Shaped Trajectory" << endl;

		waypoint << amplitude*sin(wAng*t),
					0.5*amplitude*sin(2*wAng*t),
					0.0,
					0.0,
					wAng*amplitude*cos(wAng*t),
					wAng*amplitude*cos(2*wAng*t),
					0.0,
					0.0,
					-wAng*wAng*amplitude*sin(wAng*t),
					-2*wAng*wAng*amplitude*sin(2*wAng*t),
					0.0,
					0.0;
	}
	else if(trajectory.compare("circleXY") == 0){

		cout << "circle XY - Trajectory" << endl;
	
		waypoint << amplitude*cos(wAng*t),
					amplitude*sin(wAng*t),
					0.0,
					0.0,
					-wAng*amplitude*sin(wAng*t),
					wAng*amplitude*cos(wAng*t),
					0.0,
					0.0,
					-wAng*wAng*amplitude*cos(wAng*t),
					-wAng*wAng*amplitude*sin(wAng*t),
					0.0,
					0.0;

	} else if(trajectory.compare("ident") == 0){

		cout << "Ident - Trajectory" << endl;

		if(t < 40){
			waypoint << 0.5*amplitude*(cos(wAng*t)+cos(t)),
						0.5*amplitude*(sin(wAng*t)+sin(t)),
						0.0,
						0.0,
						-0.5*amplitude*(wAng*sin(wAng*t)+sin(t)),
						0.5*amplitude*(wAng*cos(wAng*t)+cos(t)),
						0.0,
						0.0,
						-0.5*amplitude*(wAng*wAng*cos(wAng*t)+cos(t)),
						-0.5*amplitude*(wAng*wAng*sin(wAng*t)+sin(t)),
						0.0,
						0.0;

		} 
		else if(t < 80){

			waypoint << 0.0,
						0.0,
						0.5*amplitude*(sin(wAng*(t-40)+sin(t-40))),
						0.0,
						0.0,
						0.0,
						0.5*amplitude*(wAng*cos(wAng*(t-40))+cos(t-40)),
						0.0,
						0.0,
						0.0,
						-0.5*amplitude*(wAng*wAng*sin(wAng*(t-40))+sin(t-40)),
						0.0;								

		} else{
			waypoint << 0.0,
						0.0,
						0.0,
						angles::normalize_angle(0.5*1.05*(sin(wAng*(t-80))+sin(t-80))),
						0.0,
						0.0,
						0.0,
						0.5*1.05*(wAng*cos(wAng*(t-80))+cos(t-80)),
						0.0,
						0.0,
						0.0,
						-0.5*1.05*(wAng*wAng*sin(wAng*(t-80))+sin(t-80));						
		}							
	} else if(trajectory.compare("circleZXY") == 0){

		cout << "circle ZXY- Trajectory" << endl;
		
		waypoint << amplitude*cos(wAng*t),
					amplitude*sin(wAng*t),
					amplitude*sin(wAng*t),
					-wAng*amplitude*sin(wAng*t),
					wAng*amplitude*cos(wAng*t),
					wAng*amplitude*cos(wAng*t),
					0.0,
					0.0,
					-wAng*wAng*amplitude*cos(wAng*t),
					-wAng*wAng*amplitude*sin(wAng*t),
					-wAng*wAng*amplitude*sin(wAng*t),
					0.0;	

	} else if(trajectory.compare("straightLine") == 0){

		cout << "straightLine - Trajectory" << endl;
		if(t <= poseDesired(4)){
			
			t2 = t*t;
			t3 = t2*t;
			t4 = t3*t;
			t5 = t4*t;

			waypoint << cTx(0)*t3 + cTx(1)*t4 + cTx(2)*t5,
						cTy(0)*t3 + cTy(1)*t4 + cTy(2)*t5,
						cTz(0)*t3 + cTz(1)*t4 + cTz(2)*t5,
						angles::normalize_angle(cTyaw(0)*t3 + cTyaw(1)*t4 + cTyaw(2)*t5),
						3*cTx(0)*t2 + 4*cTx(1)*t3 + 5*cTx(2)*t4,
						3*cTy(0)*t2 + 4*cTy(1)*t3 + 5*cTy(2)*t4,
						3*cTz(0)*t2 + 4*cTz(1)*t3 + 5*cTz(2)*t4,
						3*cTyaw(0)*t2 + 4*cTyaw(1)*t3 + 5*cTyaw(2)*t4,
						6*cTx(0)*t   + 12*cTx(1)*t2   + 20*cTx(2)*t3,
						6*cTy(0)*t   + 12*cTy(1)*t2   + 20*cTy(2)*t3,
						6*cTz(0)*t   + 12*cTz(1)*t2   + 5*cTz(2)*t3,
						6*cTyaw(0)*t + 12*cTyaw(1)*t2 + 5*cTyaw(2)*t3;	

		} else {

			waypoint << poseDesired(0),
						poseDesired(1),
						poseDesired(2),
						angles::normalize_angle(poseDesired(3)),
						0.0,
						0.0,
						0.0,
						0.0,
						0.0,
						0.0,
						0.0,
						0.0;
		}
	} else if(trajectory.compare("wayPoint") == 0){ //No Angle

		cout << "wayPoint - Trajectory" << endl;
		
		waypoint << poseDesired(0),
					poseDesired(1),
					poseDesired(2),
					0.0,
					0.0,
					0.0,
					0.0,
					0.0,
					0.0,
					0.0,
					0.0,
					0.0;

	} else if(trajectory.compare("goToPixelCenter") == 0){

		cout << "Centralizando Pixel - Trajectory" << endl;
		
		waypoint << poseDesired(0),
					poseDesired(1),
					poseDesired(2),
					0.0,
					0.0,
					0.0,
					0.0,
					0.0,
					0.0,
					0.0,
					0.0,
					0.0;
	}

	cout << "Current Time: " << t << endl;

	if(autoMode == 1){
		if(getIsControlStarted()){  //Controle habilitado e pronto para usar!
			if (t <= tFinal){
				ackMsgGlobal = (ackMsgGlobal & (~0x0F))|0x0F;
				cout << "BUSY"<< endl;
			}
			else{
				ackMsgGlobal = (ackMsgGlobal & (~0x0F))|0x0A;
				cout << "FINISHED"<< endl;
			}
		}
		else{
			if(getFlagAbort()){
				ackMsgGlobal = (ackMsgGlobal & (~0x0F))|0x05;
				cout << "ABORTED" << endl;
			}
			else{
				ackMsgGlobal = (ackMsgGlobal & (~0x0F))|0x00;
				cout << "IDLE"<< endl;
			}
		}
		ackMsg.data = ackMsgGlobal;
		ackMessage_publisher.publish(ackMsg);	
	}
		

	// waypoint_publisher.publish(mGoal); //verificar!!!!

	return waypoint;
		
}	

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: ackControl Callback
	*	  Created by: jrsbenevides
	*  Last Modified: 
	*
	*  	 Description: Receives instructions from control node on autoMode ON.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

	void Planner::ackControlCallback(const std_msgs::UInt8::ConstPtr& msg){
		
		uint8_t controlMsg,msgControl, msgOrbScale, msgAckParam;
		// VectorFive poseDesiredGlobal;
		if(autoMode == 1){
			
			controlMsg = msg->data;
			msgControl  = controlMsg & 0x03;
			msgOrbScale = controlMsg & 0x0C;
			//Control?
			if(msgControl == 0x00){
				cout << "CONTROL CB: OFF ####" << endl;
				setIsControlStarted(false);
			}
			else if(msgControl == 0x03){
				
				if(!getFlagAbort()){
					cout << "CONTROL CB: ON OOOO" << endl;
					setIsControlStarted(true);
				} else{
					cout << "CONTROL CB: OFF ####" << endl;
					setIsControlStarted(false);
				}
			}
			//Orbslam?
			if(msgOrbScale == 0x0C){
				initOrbWithScale = true;
				ackMsgGlobal = (ackMsgGlobal & (~0xC0))|0xC0;
			}
			else if(msgOrbScale == 0x00){
				if(initOrbWithScale == true){
					ackMsgGlobal = (ackMsgGlobal & (~0xC0))|0x40;	
				}
				else{	
					ackMsgGlobal = (ackMsgGlobal & (~0xC0))|0x00;		
				}				
			}
			else if(msgOrbScale == 0x04){
				ackMsgGlobal = (ackMsgGlobal & (~0xC0))|0x80;			
			}
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: planCallback
	*	  Created by: jrsbenevides
	*  Last Modified: 
	*
	*  	 Description: Receives instructions from global trajectory planner on autoMode ON.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

	void Planner::planCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	
		int flagInt;
		VectorFive poseDesiredGlobal;

		if(autoMode == 1){

			flagInt = round(msg->pose.orientation.z);

			if(flagInt == PLANNER_IDLE){
				cout << "Idle" << endl;
				setIsControlStarted(false);
				setFlagGlobalPlanner(true);
				setFlagAckParam(false);
				setFlagAbort(false);
			}
			else if(flagInt == PLANNER_GO){
				if(getFlagGlobalPlanner()){
					setFlagGlobalPlanner(false);
					cout << "Setting Parameters..." << endl;
					poseDesiredGlobal << msg->pose.position.x,
										 msg->pose.position.y,
										 msg->pose.position.z,
										 msg->pose.orientation.x,
										 msg->pose.orientation.y;								 								 								 			
					setposeDesired(poseDesiredGlobal);
					setTrajectoryCoefficients();
					setFlagAckParam(false);
					setFlagAbort(false);  //Verificar se vai ficar assim no FIM, com interrupção do ORBSLAM
				}
				// setIsControlStarted(true); //Only after control is OK
			}
			else if(flagInt == PLANNER_ABORT){
				cout << "Abort" << endl;
				setIsControlStarted(false);
				setFlagGlobalPlanner(true);
				setFlagAckParam(false);
				setFlagAbort(true);
			}
		}
	}
}

// int main(int argc, char **argv)
// {
 
// 	ros::init(argc, argv, "genTrajectory");
	
//  	try {

// 		DRONE::Planner wptNode;

// 		ros::Rate loop_rate(50);

// 		while (ros::ok())
// 	   	{
// 		    wptNode.TrajPlanner();
// 			ros::spinOnce(); 
// 			loop_rate.sleep();
// 	   	}
		
// 		ros::spin();
// 	}
// 	catch (const std::exception &e) {
// 		ROS_FATAL_STREAM("An error has occurred: " << e.what());
// 		exit(1);
// 	}

//   	return 0;
// }


