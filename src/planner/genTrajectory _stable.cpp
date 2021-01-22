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

	void Planner::setposeDesired(VectorFive poseDesiredValue){
		
		poseDesired = poseDesiredValue;
	
	}

	void Planner::setTrajectory(const string& trajectoryInput){

		string DEFAULT_TRAJECTORY = "circleXY";

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
		} else {
			trajectory = DEFAULT_TRAJECTORY;
		}
	}

	void Planner::setTrajectoryCoefficients(void){
		double x, y, z, yaw, tf,tf3,tf4,tf5;
		x 		 = poseDesired(0);
		y 		 = poseDesired(1);
		z 		 = poseDesired(2);
		yaw 	 = poseDesired(3);
		tf 		 = poseDesired(4);
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
		
		PI 					= 3.141592653589793;
   		t 			  		= 0.0;
		amplitude 	  		= 0.4;
		velMed        		= 0.5; 
		wAng 		  		= 2*PI*velMed/(6.097*amplitude);
		trajectory    		= "circle";
		startTime 	  		= ros::Time::now().toSec();
		poseDesired			= poseDesired.Zero();
		setIsControlStarted(false);
		setIsFirstTimePass(true);
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

		waypoint_publisher = n.advertise<nav_msgs::Odometry>("/drone/waypoint", 1);
		joy_subscriber 	   = n.subscribe<sensor_msgs::Joy>("/drone/joy", 1, &Planner::joyCallback, this);
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
	}

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
	  /*Enables "Automatic Control Mode" while pressing this button*/
	  if(joy->buttons[6]){
	    setIsControlStarted(true);
	  }
	  else{
		setIsControlStarted(false);
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

				} else {

					mGoal.pose.pose.position.x = poseDesired(0); 
					mGoal.pose.pose.position.y = poseDesired(1);
					mGoal.pose.pose.position.z = poseDesired(2);

					mGoal.twist.twist.linear.x = 0; 
					mGoal.twist.twist.linear.y = 0;
					mGoal.twist.twist.linear.z = 0;
				}
				
				mGoal.pose.pose.orientation.x = 0.0;
				mGoal.pose.pose.orientation.y = 0.0;
				mGoal.pose.pose.orientation.z = 0.0;
				mGoal.pose.pose.orientation.w = 1.0;

				mGoal.twist.twist.angular.x    = 0.0;
				mGoal.twist.twist.angular.y    = 0.0;
				mGoal.twist.twist.angular.z    = 0.0;

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
			}
	    } else{

	    	setIsFirstTimePass(true);

			mGoal.pose.pose.position.x = 0.0;
			mGoal.pose.pose.position.y = 0.0;
	    }

	    cout << "Current Time: " << t << endl;

		waypoint_publisher.publish(mGoal);
	}

} //NAMESPACE DRONE


int main(int argc, char **argv)
{
 
	ros::init(argc, argv, "genTrajectory");

 	try {

		DRONE::Planner wptNode;

		ros::Rate loop_rate(50);

		while (ros::ok())
	   	{
		    wptNode.TrajPlanner();
			ros::spinOnce(); 
			loop_rate.sleep();
	   	}
		
		ros::spin();
	}
	catch (const std::exception &e) {
		ROS_FATAL_STREAM("An error has occurred: " << e.what());
		exit(1);
	}

  	return 0;
}

