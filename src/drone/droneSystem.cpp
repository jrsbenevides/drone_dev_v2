/*
 * droneSystem.cpp
 *
 *  Created on: 16/06/2017
 *      Authors: rsinoue and jrsbenevides
 *      Modified: João Benevides - Sep/26/2019
 */

// System Class

#include "drone/droneSystem.h"

namespace DRONE {

	System::System() {

		initDroneSystemParam();

		loadTopics(n);
		
		loadSettings(n);
	}

	System::~System () {

	}

	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	/* ########################################                 SETTERS                 ##########################################*/
	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/



	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: setTrajectory
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Sets to 'trajectory' global variable the one defined in config parameters file.
	*				  (Notice that, in case name was wrongly assigned, DEFAULT_TRAJECTORY will be chosen instead)
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		

	void System::setTrajectory(const string& trajectoryInput){

		string DEFAULT_TRAJECTORY = "straightLine";

		if(trajectoryInput.compare("eightShape") == 0){
			trajectory = "eightShape";
		} else if(trajectoryInput.compare("circleXY") == 0){
			trajectory = "circleXY";
		} else if(trajectoryInput.compare("circleZXY") == 0){
			trajectory = "circleZXY";
		} else if(trajectoryInput.compare("straightLine") == 0){
			trajectory = "straightLine";
		} else if(trajectoryInput.compare("ident") == 0){
			trajectory = "ident";	
		} else if(trajectoryInput.compare("wayPoint") == 0){
			trajectory = "wayPoint";
		} else {
			trajectory = DEFAULT_TRAJECTORY;
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: setControlSelect
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Sets to 'controlSelect' global variable the one setted in config parameters file.
	*				  (Notice that, in case name was wrongly assigned, DEFAULT_CONTROLLER will be chosen instead)
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		

	void System::setControlSelect(const string& controlSelectInput){

		string DEFAULT_CONTROLLER = "RLQR";

		if(controlSelectInput.compare("PID") == 0){
			controlSelect = "PID";
		} else if(controlSelectInput.compare("FL") == 0){
			controlSelect = "FL";
		} else if(controlSelectInput.compare("RLQR") == 0){
			controlSelect = "RLQR";
		} else if(controlSelectInput.compare("SLQR") == 0){
			controlSelect = "SLQR";			
		} else if(controlSelectInput.compare("RecursiveLQR") == 0){
			controlSelect = "RecursiveLQR";			
		} else {
			controlSelect = DEFAULT_CONTROLLER;
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: setSensorSelect
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Sets to 'sensorSelect' global variable the one setted in config parameters file.
	*				  (Notice that, in case name was wrongly assigned, DEFAULT_SENSOR will be chosen instead)
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

	void System::setSensorSelect(const string& sensorSelectInput){

		string DEFAULT_SENSOR = "IMU";

		if(sensorSelectInput.compare("IMU") == 0){
			sensorSelect = "IMU";
			updateParameters("IMU");
		} else if(sensorSelectInput.compare("VICON") == 0){
			sensorSelect = "VICON";
			updateParameters("VICON");
		} else if(sensorSelectInput.compare("ORBSLAM") == 0){
			sensorSelect = "ORBSLAM";			
			updateParameters("ORBSLAM");
		} else {
			sensorSelect = DEFAULT_SENSOR;
			updateParameters(DEFAULT_SENSOR);
		}
	}

	void System::updateParameters(const string& sensorSelectInput){
		string DEFAULT_SENSOR = "IMU";

		if(sensorSelectInput.compare("IMU") == 0){
			drone.setK(drone.getKimu());
		} else if(sensorSelectInput.compare("VICON") == 0){
			drone.setK(drone.getKvicon());
			network.setFlagVicon(true); //tells the network class that vicon is the current sensor
		} else if(sensorSelectInput.compare("ORBSLAM") == 0){
			drone.setK(drone.getKimu());
		} else {
			drone.setK(drone.getKimu());
		}	
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: setAmplitude
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Sets to 'amplitude' global variable the one setted in config parameters file.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

	void System::setAmplitude(const double& amplitudeValue) {
		amplitude = amplitudeValue;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: setVelMed
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Sets to 'velMed' global variable the one setted in config parameters file.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	
	void System::setVelMed(const double& velMedValue) {
		velMed = velMedValue;
	}

	void System::setFlagGlobalPlanner(bool state){
		flagGlobalPlanner = state;
	}

	void System::setFlagGoPixel(bool state){
		flagGoPixel = state;
	}

	void System::setFlagAckParam(bool state){
		flagAckParam = state;
	}

	void System::setIsOrbSlamDead(bool state){
		isOrbSlamDead = state;
	}

	bool System::getFlagAckParam(void){
		return flagAckParam;
	}

	bool System::getFlagGlobalPlanner(void){
		return flagGlobalPlanner;
	}

	bool System::getFlagGoPixel(void){
		return flagGoPixel;
	}

	bool System::getIsOrbSlamDead(void){
		return isOrbSlamDead;
	}


	

	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	/* #####################################            REGULAR FUNCTIONS                 ########################################*/
	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: testTimeout
	*	  Created by: jrsbenevides on Sep 25th 2019
	*  Last Modified: 
	*
	*  	 Description: Tracks timeout in order to manage ORBSLAM odometry acquisition
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void System::testTimeout(void){
		double timeOut = ros::Time::now().toSec() - drone.getTimeOrbSlam();
		// cout << "time out:" << timeOut << endl;
		if(timeOut > 2.0){
			// cout << "############ ESTOUROU O TIMEOUT ############" << endl;
			drone.setIsOrbSlamReady(false);
			drone.setIsOrbSlamStarted(false);
			ackControlGlobal = (ackControlGlobal & (~0x0C))|0x00;
		} else{
			ackControlGlobal = (ackControlGlobal & (~0x0C))|0x0C;
		}
	}
	

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: initDroneSystemParam
	*	  Created by: jrsbenevides
	*  Last Modified: jrsbenevides
	*
	*  	 Description: 1. Initialize essential functions for drone operation;
	*				  2. Initialize parameters and default values;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void System::initDroneSystemParam(void){

		geometry_msgs::Pose p;

		contaEnvio = 0;

		cout << "Starting Drone Node" << endl;

		// Sets flag in order to halt Vicon acquisition and make sure it will be properly initialized once/if called.
		drone.setIsViconStarted(false);
		// Sets flag in order to halt OrbSlam acquisition and make sure it will be properly initialized once/if called.
		drone.setIsOrbSlamStarted(false);
		// Sets flag in order to ignore OrbSlam data until it has converged.
		drone.setIsOrbSlamReady(false);
		// Sets initial time
		drone.setTimeNow(0);
		// Sets default control status as: NOT running.
		// drone.setIsFlagEnable(false);
		drone.setIsFlagEnable(true); // #################################################################################### FOR DEBUG ONLY!!
		// Sets to execute first run on AutoMode
		setFlagGlobalPlanner(true);
		// Sets block to enable pixel pursuit as TRUE (default)
		setFlagGoPixel(true);
		// Sets Ack flag from controller to land as false (default)
		setFlagAckParam(false);
		//Sets OrbslamDead as false (default), meaning that it is still valid for use.
		setIsOrbSlamDead(false);

		flagMonitorSelect = false;

		ackControlGlobal = 0x00;
		countVicon		 = 1;

		//Starting or Default Values
		autoMode   		= 0; //DEFAULT autoValue = Not Running
		count 	   		= 0;
		countEKF		= 1;
		PI 		 		= 3.141592653589793;
		vxAmpl 			= 0;
		vyAmpl 			= 0;
		vzAmpl 			= 0;
		angAmpl 		= 0;
		f 				= 0;
		amplitude 		= 0.8;
		velMed      	= 0.1; //m/s
		trajectory		= "straightLine";
		controlSelect	= "RLQR";
		flagTwist 		= true;

		PLANNER_IDLE 	= 0;
		PLANNER_GO   	= 10;
		PLANNER_LAND	= 30;
		PLANNER_ORBLOST = 44;
		PLANNER_ORBKILL = 47;
		PLANNER_APROX  	= 55;
		PLANNER_PARAM 	= 77;
		PLANNER_GOPXL 	= 88;
		PLANNER_ABORT	= 100;

		flagZeroAuto 	= 0;
		autoCounter	 	= 0;
		COUNTER_DELAY	= 20; //Tested for the VICON system = 50
		COUNTER_DELAY_IMU = 1;


		wAng 			= velMed/amplitude;
		flagEmergency = 0;

		// Starting cmdArray msg (PoseArray type) to be published
		p.position.x = 0;
		p.position.y = 0;
		p.position.z = 0;
		p.orientation.x = 0;
		p.orientation.y = 0;
		p.orientation.z = 0;
		p.orientation.w = 0;

		for(int i = 0; i<network.nOfAgents;i++){
			cmdArray.poses.push_back(p);
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: load Topics
	*	  Created by: rsinoue
	*  Last Modified: jrsbenevides
	*
	*  	 Description: 1. Define the ROS Topics and its types of messages;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void System::loadTopics(ros::NodeHandle &n) {
		
		// log_publisher	 	  	 = n.advertise<drone_dev::Num>("log_debug",1);
		cmd_global_publisher	 = n.advertise<geometry_msgs::PoseArray>("cmd_global",1);
		joy_subscriber 			 = n.subscribe<sensor_msgs::Joy>("/drone/joy", 1, &System::joyCallback, this);
		


		//No use for NCS
		// odom_subscriber 		 = n.subscribe<nav_msgs::Odometry>("/drone/odom", 1, &System::odomCallback, this);
		cmd_vel_publisher 		 = n.advertise<geometry_msgs::Twist>("/drone/cmd_vel",1);
		
		// waypoint_subscriber 	 = n.subscribe<nav_msgs::Odometry>("/drone/waypoint", 1, &System::waypointCallback, this);
		// orbslam_subscriber 	 	 = n.subscribe<nav_msgs::Odometry>("/scale/log", 1, &System::orbSlamCallback, this);
		// vicon_subscriber 	 	 = n.subscribe<geometry_msgs::TransformStamped>("/vicon/bebop/bebop", 1, &System::viconCallback, this);
		// targetMsg_subscriber 	 = n.subscribe<geometry_msgs::PoseStamped>("planejamento", 1, &System::planCallback, this);
		// ackGlobal_subscriber     = n.subscribe<std_msgs::UInt8>("statusPlanning",1, &System::statusPlanCallback, this);
		// ackControl_publisher 	 = n.advertise<std_msgs::UInt8>("statusAutoControl", 1);

		waypoint_publisher 	  = n.advertise<nav_msgs::Odometry>("/drone/waypoint", 1);
		transfPosition_publisher = n.advertise<nav_msgs::Odometry>("/drone/transf_position",1);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: load Settings 
	*	  Created by: rsinoue
	*  Last Modified: jrsbenevides
	*
	*  	 Description: 1. Loads config parameters and loads them into the program by substituting previously created variables.
	*                    Those can be edited in "config/bebopParameters.yaml"
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void System::loadSettings(ros::NodeHandle &n) {

		if (n.getParam("/drone/vxAmpl",vxAmpl)) {
			cout << "vxAmpl = " << vxAmpl << endl;
		}

		if (n.getParam("/drone/vyAmpl",vyAmpl)) {
			cout << "vyAmpl = " << vyAmpl << endl;
		}

		if (n.getParam("/drone/vzAmpl",vzAmpl)) {
			cout << "vzAmpl = " << vzAmpl << endl;
		}

		if (n.getParam("/drone/angAmpl",angAmpl)) {
			cout << "angAmpl = " << angAmpl << endl;
		}
		if (n.getParam("/drone/f",f)) {
			cout << "f = " << f << endl;
		}
		if (n.getParam("/drone/autoMode",autoMode)) {
			cout << "autoMode = " << autoMode << endl;
		}

		int isEKFonline;
		if (n.getParam("/drone/isEKFonline",isEKFonline)) {
			drone.setIsEKFonline(isEKFonline);
			cout << "isEKFonline = " << drone.getIsEKFonline() << endl;
		} else
		{
			ROS_ERROR("Failed to get param 'isEKFonline'");
		}

		int updateRateEKF;
		if (n.getParam("/drone/updateRateEKF",updateRateEKF)) {
			drone.setUpdateRateEKF(updateRateEKF);
			cout << "updateRateEKF = " << drone.getUpdateRateEKF() << endl;
		} else
		{
			ROS_ERROR("Failed to get param 'updateRateEKF'");
		}

		// vector<double> K;
		// if (n.getParam("/drone/K",K)) {
		// 	drone.setK(Vector8d::Map(&K[0],8));
		// 	cout << "K = [" << drone.getK().transpose() << " ]" << endl;

		// }

		vector<double> Kvicon;
		if (n.getParam("/drone/Kvicon",Kvicon)) {
			drone.setKvicon(Vector8d::Map(&Kvicon[0],8));
			cout << "Kvicon = [" << drone.getKvicon().transpose() << " ]" << endl;
		}

		vector<double> Kimu;
		if (n.getParam("/drone/Kimu",Kimu)) {
			drone.setKimu(Vector8d::Map(&Kimu[0],8));
			cout << "Kimu = [" << drone.getKimu().transpose() << " ]" << endl;
		}

		vector<double> Kp;
		if (n.getParam("/drone/Kp",Kp)) {
			Matrix4d KpMatrix;
			KpMatrix = KpMatrix.Zero();
			KpMatrix.diagonal() = Vector4d::Map(&Kp[0],4);
			drone.setKp(KpMatrix);
			cout << "Kp = [" <<  drone.getKp() << " ]" << endl;

		}

		vector<double> Kd;
		if (n.getParam("/drone/Kd",Kd)) {
			Matrix4d KdMatrix;
			KdMatrix = KdMatrix.Zero();
			KdMatrix.diagonal() = Vector4d::Map(&Kd[0],4);
			drone.setKd(KdMatrix);
			cout << "Kd = [" << drone.getKd() << " ]" << endl;

		}

		vector<double> Ki;
		if (n.getParam("/drone/Ki",Ki)) {
			Matrix4d KiMatrix;
			KiMatrix = KiMatrix.Zero();
			KiMatrix.diagonal() = Vector4d::Map(&Ki[0],4);
			drone.setKi(KiMatrix);
			cout << "Ki = [" << drone.getKi() << " ]" << endl;

		}

		vector<double> threshold;
		if (n.getParam("/drone/threshold",threshold)) {
			drone.setThreshold(Vector4d::Map(&threshold[0],4));
			cout << "INICIALIZANDO THRESHOLD COMO = " << drone.getThreshold() << endl;
		}

		vector<double> inputRange;
		if (n.getParam("/drone/inputRange",inputRange)) {
			drone.setInputRange(Vector4d::Map(&inputRange[0],4));
			cout << "inputRange = " << drone.getInputRange() << endl;
		}
		
		double amplitude;
		if (n.getParam("/drone/amplitude",amplitude)) {
			setAmplitude(amplitude);
			cout << "Amplitude = " << amplitude << endl;
		}

		double velMed;
		if (n.getParam("/drone/velMed",velMed)) {
			setVelMed(velMed);
			cout << "Vel. Media = " << velMed << endl;
		}

		string trajectory;
		if (n.getParam("/drone/trajectory",trajectory)) {
			setTrajectory(trajectory);
			cout << "trajectory = " << trajectory << endl;

		}

		string controlSelect;
		if (n.getParam("/drone/controlSelect",controlSelect)) {
			setControlSelect(controlSelect);
			cout << "controlSelect = " << controlSelect << endl;
		}

		string sensorSelect;
		if (n.getParam("/drone/sensorSelect",sensorSelect)) {
			setSensorSelect(sensorSelect);
			cout << "sensorSelect = " << sensorSelect << endl;
		}

		vector<double> poseDesired;
		if (n.getParam("/drone/poseDesired",poseDesired)) {
			drone.setposeDesired(VectorFive::Map(&poseDesired[0],5));
			cout << "poseDesired = " << drone.getposeDesired() << endl;
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: boot Vicon
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Receives a value of time ("timeValue") as an argument;
	*				  2. Verifies if selected sensor is set to "VICON";
	*				  3. Sets the given timeValue as timeOrigin;
	*				  4. Resets flag in order to reset coordinate frame.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void System::bootVicon(const double& timeValue){
		if(sensorSelect.compare("VICON") == 0){
			//Sets current time value as t0
			drone.setTimeOrigin(timeValue);
			// Sets flag to reset coordinate frame
			drone.setIsOdomStarted(false);		   	
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: boot OrbSlam
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Receives a value of time ("timeValue") as an argument;
	*				  2. Verifies if selected sensor is set to "ORBSLAM";
	*				  3. Sets the given timeValue as timeOrigin;
	*				  4. Resets flag in order to reset coordinate frame.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void System::bootOrbSlam(const double& timeValue){
		if(sensorSelect.compare("ORBSLAM") == 0){
			//Sets current time value as t0
			drone.setTimeOrigin(timeValue);
			// Sets flag to reset coordinate frame
			// drone.setIsOdomStarted(false);		   	
		}
	}	


	/*DEBUG FUNCTION*/
	double System::getTimeShifted(const double& timeValue){
		double timeShifted;
		timeShifted = timeValue - planner.getStartTime();
		return timeShifted;
	}


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: ncs (Networked Control System)
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Estimates state in a NCS subject to network-induced delay and packet loss
	*				  2. Computes optimal control
	*                 3. Sends to Multi-Agent System
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// //Original Version 
	// void System::ncs(){
	// 	int agent;
	// 	Vector4d input;

	// 	network.ComputeEstimation(); 				// Tries to compute an estimate
	// 	if(network.getFlagComputeControl()){		//Obtains K for this buffer interval
	// 		MAScontrol();
	// 		network.setFlagComputeControl(false);
	// 	}
	// 	agent = network.nextAgentToSend();			//Mount input to send => u = K*(q-qd) based on the available received data
	// 	if(agent>=0){
	// 		input = drone.MASControlInput(network.getEstimatePose(agent));
	// 		cmdArray.poses[agent].position.x = input(0);
	// 		cmdArray.poses[agent].position.y = input(1);
	// 		cmdArray.poses[agent].position.z = input(2);
	// 		cmdArray.poses[agent].orientation.x = input(3);
	// 		network.setCmdAgentDone(agent); //network.rcvArray(agent) = 15;
	// 	}
	// 	if(network.getFlagReadyToSend()){			//Publishes information to broadcast
	// 		cout << "envio" << endl;
	// 		cmd_global_publisher.publish(cmdArray);
	// 		network.setFlagComputeControl(true);
	// 		network.setRcvArrayZero(); 				//Resets array for receiving new messages
	// 		//devemos tratar as exceções. Caso tenha enviado mensagem sem ter computado input (foi com o input antigo) -> corrigir os valores de input corretos no buffer	
	// 	}
	// }	
	
	// // Debug Version 
	void System::ncs(){
		
		int agent;
		VectorQuat input;
		Vector12x1 vecDesired;
		double tTempo;

		if((drone.getIsFlagEnable())){
				
			if(flagMonitorSelect == false){ //Detects rising edge
				//Verifica se já houve alguma mudança alguma vez...novo
				// if(network.getReuseEstimate()){
				// 	cout << "faz algo" << endl;
				// }
				planner.setStartTime(ros::Time::now().toSec());
				flagMonitorSelect = true;
			}

			if((!network.getFlagEmergencyStop())&&(!network.getFlagEnter())){

				// network.ComputeEstimation_identGlobal(); 				// Tries to compute an estimate (test with new formulation)
				network.ComputeEstimation(); 				// Tries to compute an estimate

				if(network.getFlagComputeControl()){		//Obtains K for this buffer interval
					drone.setF2(network.getK2());
					MAScontrol();							//Only working for RLQR -> updateRLQRGain	
					network.setFlagComputeControl(false);
				}
				
				agent = network.nextAgentToSend();			//Mount input to send => u = K*(q-qd) based on the available received data

				if(agent>=0){
					vecDesired = planner.getPlanTrajectory(agent,network.getThisTimeSend());
					input = drone.MASControlInput(network.getEstimatePose(agent),vecDesired);
					cmdArray.poses[agent].position.x = input(0);
					cmdArray.poses[agent].position.y = input(1);
					cmdArray.poses[agent].position.z = input(2);
					cmdArray.poses[agent].orientation.x = input(3);
					network.setCmdAgentDone(agent, input); //network.rcvArray(agent) = 15; Coloco o input como a saída do sistema upost => não deixo nenhuma nova mensagem daquele agente entrar no calculo (por enquanto)
					cout << "Calculada a entrada referente ao pacote " << network.bfStruct[agent][0][0].index << " do agente " << agent << endl;
				}
			
				if(network.getFlagReadyToSend()){			//Publishes information to broadcast
					contaEnvio++;
					cout << "\n###############################" << endl;
					cout << "####### Envio Pacote " <<  contaEnvio <<  " ########" << endl;
					cout << "###############################\n" << endl;
					cmd_global_publisher.publish(cmdArray);
					tTempo = ros::Time::now().toSec();
					network.pubMyLog(0); //If we want it to only pub the last message (as in identification, replace param 0 to bfSize-1)
					cout << "Levei " << ros::Time::now().toSec() - tTempo << " s para mandar essas mensagens" << endl;
					network.setFlagComputeControl(true);
					network.setRcvArrayZero(); 				//Resets array for receiving new messages
					network.setToken(true);					//Indicates to the network package that message has been sent already
					//devemos tratar as exceções. Caso tenha enviado mensagem sem ter computado input (foi com o input antigo) -> corrigir os valores de input corretos no buffer	
				}
			}
		} else{
			if(flagMonitorSelect == true){ //Detects falling edge
				network.ResetForEstimPause(); 				//Resets functions for an eventual new estimation
				network.setReuseEstimate(true); 			//Informs the system that there was already an estimation
				contaEnvio = 0;
			}
			flagMonitorSelect = false;
		}
	}		

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: updateTrajectory 
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Gets the desired waypoint at that moment and stores it into global variable waypoint of the type Odometry
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void System::updateTrajectory(const Vector12x1& wptVector){

		//Position
		waypoint.pose.pose.position.x 		= wptVector(0);
		waypoint.pose.pose.position.y 		= wptVector(1);
		waypoint.pose.pose.position.z 		= wptVector(2);
		waypoint.pose.pose.orientation.w 	= wptVector(3);			

		//Velocity
		waypoint.pose.pose.orientation.x 	= wptVector(4);
		waypoint.pose.pose.orientation.y 	= wptVector(5);				
		waypoint.pose.pose.orientation.z 	= wptVector(6);
		waypoint.twist.twist.linear.x 		= wptVector(7);

		//Acceleration
		waypoint.twist.twist.linear.y 		= wptVector(8);
		waypoint.twist.twist.linear.z 		= wptVector(9);						
		waypoint.twist.twist.angular.x 		= wptVector(10);
		waypoint.twist.twist.angular.y 		= wptVector(11);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: updateStateNow 
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Gets the current state at that moment and stores it into global variable stateNow
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void System::updateStateNow(const Vector8d& stateVector){

		//reminder: Velocity,Position

		//Velocity
		stateNow.pose.pose.orientation.x 	= stateVector(0);
		stateNow.pose.pose.orientation.y 	= stateVector(1);				
		stateNow.pose.pose.orientation.z 	= stateVector(2);
		stateNow.twist.twist.linear.x 		= stateVector(3);

		//Position
		stateNow.pose.pose.position.x 		= stateVector(4);
		stateNow.pose.pose.position.y 		= stateVector(5);
		stateNow.pose.pose.position.z 		= stateVector(6);
		stateNow.pose.pose.orientation.w 	= stateVector(7);			

	}


	void System::printLogBuffer(void){
		
		cout << "\nLOG BUFFER:" << endl;
		for(int i=0;i<_BFSIZE;i++){
			cout << "position:" 	<< i << endl;
			cout << "tsSensor:" 	<< getTimeShifted(network.bfStruct[0][i][0].tsSensor) << endl;
			cout << "tsArrival:" 	<< getTimeShifted(network.bfStruct[0][i][0].tsArrival) << endl;
			cout << "data:" 		<< network.bfStruct[0][i][0].data << endl;
			cout << "upre:" 		<< network.bfStruct[0][i][0].upre << endl;
			cout << "upost:" 		<< network.bfStruct[0][i][0].upost << endl;
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: ncsVicon (Networked Control System)
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Estimates state in a NCS subject to network-induced delay and packet loss
	*				  2. Computes optimal control
	*                 3. Sends to Multi-Agent System
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void System::ncsVicon(){
		
		int agent;
		VectorQuat input;
		Vector12x1 vecDesired;
		double tTempo, timeNow = ros::Time::now().toSec();
		static bool flagManage = true;

		geometry_msgs::Twist cmdValue;

		if((drone.getIsFlagEnable())){
			// if((!network.getFlagEmergencyStop())&&(!network.getFlagEnter())){ //makes sure a vicon message has arrived already
			if(!network.getFlagEnter()){ //Without EmergencyStop
				if(flagMonitorSelect == false){ //Detects rising edge
					planner.setStartTime(timeNow);
					network.timeStart = timeNow;
					network.getThisTimeSend(); //first time
					flagMonitorSelect = true;
				}

				//Debug
				tTempo = ros::Time::now().toSec();
				cout << "\nContagem: " << contaEnvio << endl;
				cout << "Time Now: " << getTimeShifted(tTempo) << endl;
				cout << "Time Next: " << getTimeShifted(network.getTimeNext()) << endl;

				network.ComputeEstimation_identGlobal(); 				// Tries to compute an estimate

				if(network.getFlagComputeControl()){		//Obtains K for this buffer interval
					drone.setOrientationParameters(network.getCurrentYaw(0),0);
					// drone.setF2(network.getK2());
					MAScontrol();							//Only working for RLQR -> updateRLQRGain	
					network.setFlagComputeControl(false);
				}
				
				agent = network.nextAgentToSend();			//Mount input to send => u = K*(q-qd) based on the available received data

				if(agent>=0){
					drone.setOrientationParameters(network.getCurrentYaw(agent),agent);
					vecDesired = planner.getPlanTrajectory(agent,network.getThisTimeSend());
					cout << "Pose Desejada:" << vecDesired.transpose() << endl;
					updateTrajectory(vecDesired);
					input = drone.MASControlInput(network.getEstimatePose(agent),vecDesired);
					updateStateNow(network.getEstimatePose(agent));
					cmdValueGlobal.linear.x  = input(0);
					cmdValueGlobal.linear.y  = input(1);
					cmdValueGlobal.linear.z  = input(2);
					cmdValueGlobal.angular.x = 0;            
					cmdValueGlobal.angular.y = 0;            
					cmdValueGlobal.angular.z = input(3);
					// cmdValueRepeat = cmdValue;
					inputRepeat = input;
					cout << "input: " << input.transpose() << endl;
					network.setCmdAgentDone(agent, input); //network.rcvArray(agent) = 15; Coloco o input como a saída do sistema upost => não deixo nenhuma nova mensagem daquele agente entrar no calculo (por enquanto)
					// cout << "Calculada a entrada referente ao pacote " << network.bfStruct[agent][0][0].index << " do agente " << agent << endl;
				}

				//Testing if can send already
				network.checkSendingConditions();
			
				if(network.getFlagReadyToSend()){			//Publishes information to broadcast
					contaEnvio++;
					cout << "\n###############################" << endl;
					cout << "####### Envio Pacote " <<  contaEnvio <<  " ########" << endl;
					cout << "###############################\n" << endl;
					cmd_vel_publisher.publish(cmdValueGlobal);
					transfPosition_publisher.publish(stateNow);
					waypoint_publisher.publish(waypoint);
					tTempo = ros::Time::now().toSec();
					network.setLastTimeSent(tTempo);
					// network.pubMyLog(0); //If we want it to only pub the last message (as in identification, replace param 0 to bfSize-1)
					// cout << "Levei " << ros::Time::now().toSec() - tTempo << " s para mandar essas mensagens" << endl;
					cout << "Enviei, às " << getTimeShifted(tTempo) << " a mensagem, capturada no tempo " << getTimeShifted(network.bfStruct[0][4][0].tsArrival) << "\nE programada pra ser enviada em " << getTimeShifted(network.getTimeNext()) << endl;
					printLogBuffer();
					network.setFlagComputeControl(true);
					network.setFlagReadyToSend(false);
					network.setRcvArrayZeroTotal(); 		//Resets array for receiving new messages
					network.setToken(true);					//Indicates to the network package that message has been sent already
					network.getThisTimeSend(); 				//debug
					cout << "\n\n###############################\n\n" << endl;
					//devemos tratar as exceções. Caso tenha enviado mensagem sem ter computado input (foi com o input antigo) -> corrigir os valores de input corretos no buffer	
				} else	{
					cmdValue.linear.x  = inputRepeat(0);
					cmdValue.linear.y  = inputRepeat(1);
					cmdValue.linear.z  = inputRepeat(2);
					cmdValue.angular.x = 0;            
					cmdValue.angular.y = 0;            
					cmdValue.angular.z = inputRepeat(3);
					cmd_vel_publisher.publish(cmdValue);
				}	
			}
		} else{
			if(flagMonitorSelect == true){ 					//Detects falling edge
				network.ResetForEstimPause(); 				//Resets functions for an eventual new estimation
				network.setReuseEstimate(true); 			//Informs the system that there was already an estimation
				contaEnvio = 0;
			}
			flagMonitorSelect = false;
			cout << "Time Now: " << getTimeShifted(ros::Time::now().toSec()) << endl;
		}
	}				

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: control
	*	  Created by: jrsbenevides
	*  Last Modified: 13 Sep 2019
	*
	*  	 Description: 1. This is the main control function. It depends on flagEnable (select button to run);
	*				  2. Gets current position;
	*				  3. Checks which controller to use. In case of PID, a reset of integral error is necessary and 
	*					 flagControllerStarted for taking ;
	*				  4. Gets the provided input publishes it.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	void System::control() {

	  geometry_msgs::Twist cmd_vel_msg;
	  std_msgs::UInt8 ackControl_msg;
	  Vector3axes 	position;
	  Vector4d		xTemp,input;

	  // This if is enabled by user through joystick (DEFAULT = hold SELECT button)
	  if(drone.getIsFlagEnable()&&(flagEmergency == 0)){ //flagEnable == true

		position = drone.getPosition();

		if(controlSelect.compare("PID") == 0){

			cout << "### PID ###" << endl;
			
			// Enters only during first loop after holding joystick button responsible for "flagEnable".
			if(flagControllerStarted == true){ 
		  		drone.setXIntError(xTemp.Zero()); // sets integral PID error as zero.
		  		flagControllerStarted = false;
	  		}
		
		 	input = drone.getPIDControlLaw();
		
		} else if(controlSelect.compare("FL") == 0){

			cout << "### Feedback Linearization ###" << endl;
			
			input = drone.getFLControlLaw();

		} else if(controlSelect.compare("RLQR") == 0){

			// cout << "### Robust LQR ###" << endl;

			input = drone.getRobustControlLaw();

		} else if(controlSelect.compare("SLQR") == 0){

			cout << "### Standard LQR ###" << endl;

			input = drone.getLQRControlLaw();

		} else if(controlSelect.compare("RecursiveLQR") == 0){

			cout << "### Recursive LQR ###" << endl;

			input = drone.getRecursiveLQRControlLaw();

		} else if(controlSelect.compare("pixelPD") == 0){

			cout << "### PIXEL PD ###" << endl;

			input = drone.getPixelPDControlLaw();

		} else {

			cout << "\n ERROR: Please select a valid controller" << endl;

		}

		drone.inputSaturation(input);

		// cout << "Input:" << input << endl;
		
	     cmd_vel_msg.linear.x  = input(0);
	     cmd_vel_msg.linear.y  = input(1);
	     cmd_vel_msg.linear.z  = input(2);
	     cmd_vel_msg.angular.x = 0;            
	     cmd_vel_msg.angular.y = 0;            
	     cmd_vel_msg.angular.z = input(3);

	     // Publish input controller
	     cmd_vel_publisher.publish(cmd_vel_msg);
		}
		else{

		flagControllerStarted = true; //Keeps track of flagEnable activation in order to zero PID integral error.

		 cmd_vel_msg.linear.x  = 0;
	     cmd_vel_msg.linear.y  = 0;
	     cmd_vel_msg.linear.z  = 0;
	     cmd_vel_msg.angular.x = 0;            
	     cmd_vel_msg.angular.y = 0;            
	     cmd_vel_msg.angular.z = 0;

	     // Publish input controller
	     cmd_vel_publisher.publish(cmd_vel_msg);

		}

		if(autoMode == 1){
			if(drone.getIsFlagEnable()){
				ackControlGlobal = (ackControlGlobal & (~0x03))|0x03;
			}
			else{
				ackControlGlobal = (ackControlGlobal & (~0x03))|0x00;
			}
			ackControl_msg.data = ackControlGlobal;
			ackControl_publisher.publish(ackControl_msg);
		}

	  	// // Timeout to check if OrbSlam is running
	  	// if(sensorSelect.compare("ORBSLAM") == 0){
	  	// 	testTimeout();	
	  	// }
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: MAScontrol == MAS Control (Multi-Agent System Control)
	*	  Created by: jrsbenevides
	*  Last Modified: 13 Sep 2019
	*
	*  	 Description: 1. This is the main control function. It depends on flagEnable (select button to run);
	*				  2. Gets current position;
	*				  3. Checks which controller to use. In case of PID, a reset of integral error is necessary and 
	*					 flagControllerStarted for taking ;
	*				  4. Gets the provided input publishes it.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	void System::MAScontrol() {

	  geometry_msgs::Twist cmd_vel_msg;
	  std_msgs::UInt8 ackControl_msg;
	  Vector3axes 	position;
	  VectorQuat	xTemp,input;

	  // This if is enabled by user through joystick (DEFAULT = hold SELECT button)
	  if(drone.getIsFlagEnable()&&(flagEmergency == 0)){ //flagEnable == true

		if(controlSelect.compare("PID") == 0){

			cout << "### PID ###" << endl;
			
			// Enters only during first loop after holding joystick button responsible for "flagEnable".
			if(flagControllerStarted == true){ 
		  		drone.setXIntError(xTemp.Zero()); // sets integral PID error as zero.
		  		flagControllerStarted = false;
	  		}
		
		 	input = drone.getPIDControlLaw();
		
		} else if(controlSelect.compare("FL") == 0){

			cout << "### Feedback Linearization ###" << endl;
			
			input = drone.getFLControlLaw();

		} else if(controlSelect.compare("RLQR") == 0){

			cout << "### Robust LQR ###" << endl;

			drone.updateRLQRGain();

		} else if(controlSelect.compare("SLQR") == 0){

			cout << "### Standard LQR ###" << endl;

			input = drone.getLQRControlLaw();

		} else if(controlSelect.compare("RecursiveLQR") == 0){

			cout << "### Recursive LQR ###" << endl;

			input = drone.getRecursiveLQRControlLaw();

		} else if(controlSelect.compare("pixelPD") == 0){

			cout << "### PIXEL PD ###" << endl;

			input = drone.getPixelPDControlLaw();

		} else {

			cout << "\n ERROR: Please select a valid controller" << endl;

		}

		// drone.inputSaturation(input);

		// // cout << "Input:" << input << endl;
		
	    //  cmd_vel_msg.linear.x  = input(0);
	    //  cmd_vel_msg.linear.y  = input(1);
	    //  cmd_vel_msg.linear.z  = input(2);
	    //  cmd_vel_msg.angular.x = 0;            
	    //  cmd_vel_msg.angular.y = 0;            
	    //  cmd_vel_msg.angular.z = input(3);

	    //  // Publish input controller
	    //  cmd_vel_publisher.publish(cmd_vel_msg);
		}
		else{

		flagControllerStarted = true; //Keeps track of flagEnable activation in order to zero PID integral error.

		//  cmd_vel_msg.linear.x  = 0;
	    //  cmd_vel_msg.linear.y  = 0;
	    //  cmd_vel_msg.linear.z  = 0;
	    //  cmd_vel_msg.angular.x = 0;            
	    //  cmd_vel_msg.angular.y = 0;            
	    //  cmd_vel_msg.angular.z = 0;

	    //  // Publish input controller
	    //  cmd_vel_publisher.publish(cmd_vel_msg);

		}

		// if(autoMode == 1){
		// 	if(drone.getIsFlagEnable()){
		// 		ackControlGlobal = (ackControlGlobal & (~0x03))|0x03;
		// 	}
		// 	else{
		// 		ackControlGlobal = (ackControlGlobal & (~0x03))|0x00;
		// 	}
		// 	ackControl_msg.data = ackControlGlobal;
		// 	ackControl_publisher.publish(ackControl_msg);
		// }

	  	// // Timeout to check if OrbSlam is running
	  	// if(sensorSelect.compare("ORBSLAM") == 0){
	  	// 	testTimeout();	
	  	// }
	}

	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	/* #######################################                 CALLBACKS                 #########################################*/
	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: planCallback
	*	  Created by: jrsbenevides
	*  Last Modified: 
	*
	*  	 Description: Receives instructions from global trajectory planner on autoMode ON.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

	void System::planCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	
		int flagInt;
		Vector3axes pixelWaypoint;

		if(autoMode == 1){
			flagInt = round(msg->pose.orientation.z);

			if(flagInt == PLANNER_IDLE){
				cout << "Idle" << endl;
				drone.setIsFlagEnable(false);
				setFlagGlobalPlanner(true);
				flagZeroAuto 	= 0;
				autoCounter	 	= 0;
				drone.zeroSumPixelError();
			}
			else if(flagInt == PLANNER_GO){
				if(getFlagGlobalPlanner()){
					setFlagGlobalPlanner(false);
					cout << "Resetting Frame..." << endl;
					drone.setIsOdomStarted(false); //Zera odometria
					cout << "Enabling Auto Control..." << endl;
				}
				// drone.setIsFlagEnable(true);  //DEBUG OFFLINE SOMENTE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! COMENTAR!!!!!!!!!!!!
			}
			else if(flagInt == PLANNER_ABORT){
				cout << "Abort" << endl;
				drone.setIsFlagEnable(false);
				setFlagGlobalPlanner(true);
				flagZeroAuto 	= 0;
				autoCounter	 	= 0;
				drone.zeroSumPixelError();
			}	
			else if(flagInt == PLANNER_ORBLOST){
				drone.setIsOrbSlamReady(false);
			}
			else if(flagInt == PLANNER_ORBKILL){
				setIsOrbSlamDead(true);
			}	
		}
	}


	void System::statusPlanCallback(const std_msgs::UInt8::ConstPtr& msg){

		uint8_t statusMsg, controlMsg, yawMsg;
		
		if(autoMode == 1){
			
			controlMsg = msg->data;
			statusMsg  = controlMsg & 0x0F;
			yawMsg     = controlMsg & 0x30;

			if(statusMsg == 0x0A){ //chegou...pode parar
				// cout << "CHEGOU... 0.05input" << endl;
				drone.setFreezeConstant(0.05);
			} else {
				if(yawMsg == 0x30){ //significa que é yaw
					// cout << "YAW!!!" << endl;
					drone.setFreezeConstant(0.5);
				}
				else {// Voo normal de ponto a ponto
					// cout << "NORMAL!" << endl;
					drone.setFreezeConstant(0.7);
				}
			}
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

	void System::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) 
	{
		if (joy->buttons[14]) {
		drone.setIsOdomStarted(false);
		network.ZeroIsOdomStarted();
		// cout << "RESET BUTTON!!!" << endl;
		} 

		if(joy->buttons[6]){
		drone.setIsFlagEnable(true);
		network.setIsFlagEnable(true);
		// cout << "select ON" << endl;
		}
		else{
		drone.setIsFlagEnable(false);
		network.setIsFlagEnable(false);
		// cout << "select Off" << endl;
		}	
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: odomCallback
	*	  Created by: rsinoue
	*  Last Modified: jrsbenevides
	*
	*  	 Description: General function for acquiring odometry data. Inertial measurement gives global position but local 
	*				  velocity. This is handled inside set functions, that converts it all to global with respect to the 
	*				  defined global coordinate.
	*		   Steps:
	*				  1. It is enabled only when 'IMU' is selected as sensor;
	*				  2. Gets current time, position, orientation, linear and angular velocity;
	*				  3. Uses current position and orientation to set new coordinate frame in case that flag allows it do so;
	*				  4. Sets time, position, orientation, linear and angular velocity in local terms;
	*				  5. Gets current local position and orientation in order to call function globalToLocalPosition that 
	*					 publishes this info.
	*
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void System::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)	{
		if( (sensorSelect.compare("IMU") == 0) || ( (!drone.getIsOrbSlamReady()) && (sensorSelect.compare("ORBSLAM") == 0) ) || (getIsOrbSlamDead()) ){
			
			Vector3axes position, positionLocal, angularVel, linearVel, rpy;
			
			VectorQuat  orientation, orientationLocal;

			double 		timeNow, yawOdom;

			updateParameters("IMU");

			position 	<< odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z;
			linearVel 	<< odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z;
			orientation << odom->pose.pose.orientation.w, 
						   odom->pose.pose.orientation.x, 
						   odom->pose.pose.orientation.y, 
						   odom->pose.pose.orientation.z;
			angularVel 	<< odom->twist.twist.angular.x, odom->twist.twist.angular.y, odom->twist.twist.angular.z;
			
			timeNow 	= odom->header.stamp.toSec();

			if((getFlagGlobalPlanner() == false)&&(autoMode == 1)&&(flagZeroAuto == 1)){
				if(autoCounter == COUNTER_DELAY_IMU){
					if(getFlagGoPixel()){
						drone.setIsFlagEnable(true);
						cout << "###################### flagSetada ###################### " << endl;
					}
				}
				if(autoCounter <= COUNTER_DELAY_IMU){
					if(autoCounter == 0){
						drone.setDesiredZero(); //Zero desiredPosition, velocity acceleration
					}
					autoCounter++;
				}
			}	

			/*Reset frame location*/
			if (!drone.getIsOdomStarted()) {
				Conversion::quat2angleZYX(rpy,orientation);
				yawOdom = angles::normalize_angle(rpy(2));				//Added normalize...check if it works!!!
				drone.setPosition0(position,yawOdom);
				if((getFlagGlobalPlanner() == false)&&(autoMode == 1)){
					cout << "Frame Reset...Enabling Control..." << endl;
					flagZeroAuto = 1;
				}
			}

			// /*Reset frame location*/
			// if (!drone.getIsOdomStarted()) {
			// 	Conversion::quat2angleZYX(rpy,orientation);
			// 	yawOdom = angles::normalize_angle(rpy(2));				//Added normalize...check if it works!!!
			// 	drone.setPosition0(position,yawOdom);
			// }
			
			drone.setPosition(position);
			drone.setOrientation(orientation);
			drone.setAngularVel(angularVel);
			drone.setLinearVel(linearVel);
			drone.setTimeNow(timeNow);
			
			// cout << "Pose updated (IMU)" << endl;
			// cout << "angular Velocity: twist: " << angularVel.transpose() << endl;

			/*Envia mensagens de position corrigidas toda vez que uma nova mensagem de odom chega*/
			/*tipo: nav_msgs::Odometry - Desabilitado para a competição*/

			positionLocal 	 = drone.getPosition();
			orientationLocal = drone.getOrientation();

			globalToLocalPosition(positionLocal, orientationLocal, linearVel, angularVel);

			if( (!drone.getIsOrbSlamReady()) && (!getIsOrbSlamDead()) ){
				ackControlGlobal = (ackControlGlobal & (~0x0C))|0x00;
			}
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: globalToLocal
	*	  Created by: jrsbenevides
	*  Last Modified:
	*
	*  	 Description: 1. Gets position and orientation values (local) and publishes them in a topic in order to be later
	*				  	 assessed by the plotControl node.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

	void System::globalToLocalPosition(const Vector3axes& positionValue, const VectorQuat& orientationValue, const Vector3axes& linearVelValue,const Vector3axes& angularVelValue){

		nav_msgs::Odometry goalPosition;

		goalPosition.pose.pose.position.x 		= positionValue(0);
		goalPosition.pose.pose.position.y 		= positionValue(1);
		goalPosition.pose.pose.position.z 		= positionValue(2);

		goalPosition.pose.pose.orientation.w 	= orientationValue(0);				
		goalPosition.pose.pose.orientation.x 	= orientationValue(1);				
		goalPosition.pose.pose.orientation.y 	= orientationValue(2);				
		goalPosition.pose.pose.orientation.z 	= orientationValue(3);

		goalPosition.twist.twist.linear.x 		= linearVelValue(0);
		goalPosition.twist.twist.linear.y 		= linearVelValue(1);
		goalPosition.twist.twist.linear.z 		= linearVelValue(2);						
		
		goalPosition.twist.twist.angular.x 		= angularVelValue(0);
		goalPosition.twist.twist.angular.y 		= angularVelValue(1);
		goalPosition.twist.twist.angular.z 		= angularVelValue(2);

		transfPosition_publisher.publish(goalPosition);

	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: waypointCallback
	*	  Created by: jrsbenevides
	*  Last Modified: Sep 13th 2019
	*
	*  	 Description: Gets desired trajectories. Desired acceleration is computed based on the functions.
	*
	*	       Steps: 1. Gets position and velocity desired;
	*				  2. Because acceleration does not come in the message we need to calculate it in order to compute it
	*					 in case of eightShape and circle trajectories;
	*				  3. Checks which trajectory was chosen and computes acceleration accordingly;
	*				  4. Sets desired position, orientation, velocity and acceleration. (Notice: with respect to local frame).
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

	void System::waypointCallback(const nav_msgs::Odometry::ConstPtr& waypoint){
		
		Vector3axes eulerAngles,positionToGo, dpositionToGo, d2positionToGo;
		VectorQuat quatAngles;

		double yawAngle, yawDesired, dYawDesired, d2YawDesired;

		positionToGo <<  waypoint->pose.pose.position.x,
						 waypoint->pose.pose.position.y,
						 waypoint->pose.pose.position.z;

		dpositionToGo << waypoint->twist.twist.linear.x,
						 waypoint->twist.twist.linear.y,
						 waypoint->twist.twist.linear.z;

		quatAngles 		<< 	waypoint->pose.pose.orientation.w,
						  	waypoint->pose.pose.orientation.x,
						  	waypoint->pose.pose.orientation.y,
						  	waypoint->pose.pose.orientation.z;

		Conversion::quat2angleZYX(eulerAngles,quatAngles);

		yawDesired  = eulerAngles(2);						 


		if(trajectory.compare("eightShape") == 0){	//Lemniscate of Gerono					 
		
			wAng = 2*PI*velMed/(6.097*amplitude); //Based on total arc length of this shape
			
			d2positionToGo << -wAng*wAng*(waypoint->pose.pose.position.x),
							  -4*wAng*wAng*(waypoint->pose.pose.position.y),
							  0;			  										  
		}
		else if(trajectory.compare("circleXY") == 0){ //Circle in the xy plane
		
			wAng = velMed/amplitude;
			
			d2positionToGo << -wAng*wAng*(waypoint->pose.pose.position.x),
							  -wAng*wAng*(waypoint->pose.pose.position.y),
							  0;		
					  					
		}		
		else if(trajectory.compare("circleZXY") == 0){ //Adds a circular component on z axis on the circleXY
		
			wAng = velMed/amplitude;
			
			d2positionToGo << -wAng*wAng*(waypoint->pose.pose.position.x),
							  -wAng*wAng*(waypoint->pose.pose.position.y),											  
							  -wAng*wAng*(waypoint->pose.pose.position.z);				
		}
		else if(trajectory.compare("ident") == 0){ //For identifying the dynamic model
			
			if((yawDesired == 0)&&(flagTwist == true)){ //Significa que usamos o twist angular como aceleração linear desejada
				
				d2positionToGo << waypoint->twist.twist.angular.x,
							  	  waypoint->twist.twist.angular.y,											 
							  	  waypoint->twist.twist.angular.z;	

				dYawDesired = 0.0;		

				cout << "PART ONE 11111111" << endl;					  	  
			}
			else{ //Significa que usamos o twistangular z como vel ang desejada e twistangular x como acel ang desejada

				flagTwist = false;

				d2positionToGo << 0.0,0.0,0.0;	

				dYawDesired  = waypoint->twist.twist.angular.z;	
				d2YawDesired = waypoint->twist.twist.angular.x;	

				cout << "PART TWO 2222222" << endl;
			}
		}
		else if(trajectory.compare("straightLine") == 0){

			d2positionToGo << 	0,0,0;
		}
		else if(trajectory.compare("wayPoint") == 0){

			d2positionToGo << 	0,0,0;
		}

		//Acquire desired angular velocity and acceleration (Does NOT apply for "ident")
		if(trajectory.compare("ident") != 0){
		  	
		  	dYawDesired = waypoint->twist.twist.angular.z;
		  	d2YawDesired = 0.0;
		}
	
		drone.setPositionDesired(positionToGo);
		drone.setYawDesired(yawDesired);

		drone.setdPositionDesired(dpositionToGo);
		drone.setdYawDesired(dYawDesired);
		
		drone.setd2PositionDesired(d2positionToGo);
		drone.setd2YawDesired(d2YawDesired);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: viconCallback
	*	  Created by: jrsbenevides
	*  Last Modified: Sep 13th 2019
	*
	*  	 Description: General function for acquiring odometry data from VICON Motion Tracking System. Vicon measurement gives 
	*				  global position but no velocity. A standard Kalman Filter was designed for velocity estimation.
	*		   Steps:
	*  	 			  1. It is enabled only when 'VICON' is selected as sensor;
	*				  2. Gets current time, position and orientation;
	*				  3. Sets position now, because we need to compute velocity;
	*				  4. Because Vicon sensor does not deliver either linear or angular velocity, we need estimate them through a
	*					 simple Kalman Filter, which is performed by DvKalman and DwKalman functions
	*				  5. Uses current position and orientation to set new coordinate frame in case that flag allows it do to so;
	*				  6. Sets orientation, linear and angular velocity in local terms;
	*				  7. Does the 'ViconBoot' when passing the callback for the first time;
	*				  8. Sets current time;
	*				  9. Gets current local position and orientation in order to call function globalToLocalPosition.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

	void System::viconCallback(const geometry_msgs::TransformStamped::ConstPtr& vicon){

		if((countVicon % 5) == 0){
			countVicon = 1;
			if(sensorSelect.compare("VICON") == 0){

				updateParameters("VICON");

				Vector3axes positionVicon, positionNow, positionLocal, angularVel, linearVel, rpy;
				VectorQuat orientationVicon, orientationLocal,orientationNow;

				double time, timePast, yawVicon;

				timePast 		= drone.getTimeNow(); 		//Stores last known time as past.

				time			= drone.getThisTime(vicon->header.stamp.toSec()); // returns time in sec after start
				
				positionVicon	<< vicon->transform.translation.x, vicon->transform.translation.y, vicon->transform.translation.z;
				orientationVicon 	<< vicon->transform.rotation.w, vicon->transform.rotation.x, vicon->transform.rotation.y, vicon->transform.rotation.z;

				drone.setPosition(positionVicon);
				positionNow		= drone.getPosition();
				linearVel 		= drone.DvKalman(positionNow,time,timePast); //global terms instead of local (IMU)
				
				drone.setOrientation(orientationVicon);
				orientationNow	= drone.getOrientation();
				angularVel 		= drone.DwKalman(orientationNow,time,timePast);

				if((getFlagGlobalPlanner() == false)&&(autoMode == 1)&&(flagZeroAuto == 1)){
					if(autoCounter == COUNTER_DELAY){
						if(getFlagGoPixel()){
							drone.setIsFlagEnable(true);
							cout << "###################### flagSetada ###################### " << endl;
						}
					}
					if(autoCounter <= COUNTER_DELAY){
						if(autoCounter == 0){
							drone.setDesiredZero(); //Zero desiredPosition, velocity acceleration
						}
						autoCounter++;
					}
				}	

				/*Reset frame location*/
				if (!drone.getIsOdomStarted()) {
					Conversion::quat2angleZYX(rpy,orientationVicon);
					yawVicon = rpy(2);
					drone.setPosition0(positionVicon,yawVicon);
					drone.setOrientation(orientationVicon);
					if((getFlagGlobalPlanner() == false)&&(autoMode == 1)){
						cout << "Frame Reset...Enabling Control..." << endl;
						flagZeroAuto = 1;
					}
				}
				
				drone.setAngularVel(angularVel);
				drone.setLinearVelVicon(linearVel);

				//This will reset coordinate frame and set initial time as zero
				if(!drone.getIsViconStarted()){
					bootVicon(time); 
					drone.setIsViconStarted(true);
				}

				drone.setTimeNow(time); //Stores this time as current!

				//This will publish transformed positions and orientations in order to easier evaluate control performance (transf_position) 
				globalToLocalPosition(positionNow, orientationNow,linearVel,angularVel);
			}
		}else{
			countVicon++;
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: orbSlamCallback
	*	  Created by: jrsbenevides
	*  Last Modified: Sep 13th 2019
	*
	*  	 Description: General function for acquiring odometry data from OrbSLAM algorithm. OrbSLAM gives 
	*				  global position but no velocity. A standard Kalman Filter was designed for velocity estimation.
	*		   Steps:
	*  	 			  1. It is enabled only when 'ORBSLAM' is selected as sensor;
	*				  2. Gets current time, position and orientation;
	*				  3. Sets position now, because we need to compute velocity;
	*				  4. Because OrbSlam sensor does not deliver either linear or angular velocities, we need estimate them through a
	*					 simple Kalman Filter, which is performed by DvKalman and DwKalman functions
	*				  5. Uses current position and orientation to set new coordinate frame in case that flag allows it do to so;
	*				  6. Sets orientation, linear and angular velocity in local terms;
	*				  7. Does the 'OrbSlam boot' when passing the callback for the first time;
	*				  8. Sets current time;
	*				  9. Gets current local position and orientation in order to call function globalToLocalPosition.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

void System::orbSlamCallback(const nav_msgs::Odometry::ConstPtr& orbslam){

	if(!getIsOrbSlamDead()){

		Vector3axes positionOrb, positionNow, positionZero, angularVel, linearVel, rpy;
		
		VectorQuat  orientationOrb, orientationNow, orientationZero;

		double 		timeNow, yawOrbSlam;
		double 		time, timePast, yawVicon;

		if(sensorSelect.compare("ORBSLAM") == 0){

			//Updates dynamic model
			updateParameters("ORBSLAM");

			if(!drone.getIsOrbSlamReady()){
				cout << "FIRST READING ORBSLAM" << endl;
				drone.setIsOrbSlamReady(true);//Raises flag that disables odometry acquisition from IMU	
				if(drone.getIsFirstOrbSlamRun()){ //Checks if this is first run
					cout << "INITIALIZING ORBSLAM" << endl;
					// positionZero	= drone.getPosition(); //This position should be the zero position;
					// orientationZero	= drone.getOrientation(); //Zero orientation
					// Conversion::quat2angleZYX(rpy,orientationZero); 
					// yawOrbSlam = angles::normalize_angle(rpy(2));
					// drone.setPosition0(positionZero,yawOrbSlam);
					// drone.setIsOdomStarted(false); //Makes sure that frame will not be reset again during this first run
					drone.setIsFirstOrbSlamRun(false); //Closes this if-condition
				}
			}

			// cout << "READING ORBSLAM" << endl;


			// position 	<< odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z;
			// linearVel 	<< odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z;
			// orientation << odom->pose.pose.orientation.w, 
			// 			   odom->pose.pose.orientation.x, 
			// 			   odom->pose.pose.orientation.y, 
			// 			   odom->pose.pose.orientation.z;
			// angularVel 	<< odom->twist.twist.angular.x, odom->twist.twist.angular.y, odom->twist.twist.angular.z;


			positionOrb 	<< orbslam->pose.pose.position.x, orbslam->pose.pose.position.y, orbslam->pose.pose.position.z;
			orientationOrb 	<< orbslam->pose.pose.orientation.w,
						   	   orbslam->pose.pose.orientation.x,
						   	   orbslam->pose.pose.orientation.y,
						   	   orbslam->pose.pose.orientation.z;
			

			timePast 		= drone.getTimeNow(); 		//Stores last known time as past.

			time			= drone.getThisTime(orbslam->header.stamp.toSec()); // returns time in sec after start

			drone.setPosition(positionOrb);
			// positionNow		= drone.getPosition();
			// linearVel 		= drone.DvKalman(positionNow,time,timePast); 
			linearVel 		<< orbslam->twist.twist.linear.x, orbslam->twist.twist.linear.y, orbslam->twist.twist.linear.z;
			
			drone.setOrientation(orientationOrb);
			// orientationNow	= drone.getOrientation();
			// angularVel 		= drone.DwKalman(orientationNow,time,timePast);
			angularVel 		<< orbslam->twist.twist.angular.x, orbslam->twist.twist.angular.y, orbslam->twist.twist.angular.z;

			if((getFlagGlobalPlanner() == false)&&(autoMode == 1)&&(flagZeroAuto == 1)){
				if(autoCounter == COUNTER_DELAY_IMU){
					if(getFlagGoPixel()){
						drone.setIsFlagEnable(true);
						cout << "###################### flagSetada ###################### " << endl;
					}
				}
				if(autoCounter <= COUNTER_DELAY_IMU){
					if(autoCounter == 0){
						drone.setDesiredZero(); //Zero desiredPosition, velocity acceleration
					}
					autoCounter++;
				}
			}	

			/*Reset frame location*/
			if (!drone.getIsOdomStarted()) {
				Conversion::quat2angleZYX(rpy,orientationOrb);
				yawOrbSlam = rpy(2);
				drone.setPosition0(positionOrb,yawOrbSlam);
				drone.setOrientation(orientationOrb);
				if((getFlagGlobalPlanner() == false)&&(autoMode == 1)){
					cout << "Frame Reset...Enabling Control..." << endl;
					flagZeroAuto = 1;
				}
			}
			
			drone.setAngularVel(angularVel);
			drone.setLinearVel(linearVel);

			// cout << "Vel ORBSLAM:" << linearVel << endl;

			//This will set initial time as zero and reset coordinate frame as the last pose value from IMU
			if(!drone.getIsOrbSlamStarted()){
				bootOrbSlam(time); 
				drone.setIsOrbSlamStarted(true);
			}

			drone.setTimeNow(time); //Stores this time as current!

			drone.setTimeOrbSlam();

			ackControlGlobal = (ackControlGlobal & (~0x0C))|0x0C;

			//This will publish transformed positions and orientations in order to easier evaluate control performance (transf_position) 
			// globalToLocalPosition(positionNow, orientationNow,linearVel,angularVel);
		}
	} else{
		ackControlGlobal = (ackControlGlobal & (~0x0C))|0x04;
	}
}

}