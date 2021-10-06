/*
 * 	networkEstimator.cpp
 *
 *  Created on: Dec 28, 2020
 *      Author: João Benevides
 *      Modified: jrsbenevides - August 28th 2021
 */

#include "network/networkEstimator.h"

namespace DRONE {

	Estimator::Estimator() {

		initEstimator();
		
		loadTopics(n);
		loadSettings(n);

   		srand (time(NULL)); /* initialize random seed: */
	}

	Estimator::~Estimator () {

	}

	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	/* ########################################                 SETTERS                 ##########################################*/
	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/

	void Estimator::setBuffer(const Buffer& msg){
		bfTemp[msg.index] = msg;
	}

	void Estimator::setBufferNext(const Buffer& msg){
		int index;
		index = rcvArrayBuffer(msg.index);
		if(index<_BUFMAXSIZE){
			bfTempPending[msg.index][index] = msg;
		}
	}


	void Estimator::setK(const Vector8d& Kvalue){
		K = Kvalue;
	}	
	
	void Estimator::setEstimatePose(const Vector8d& x,const int agent){
		estPose[agent] << x;
	}

	void Estimator::setFlagReadyToSend(const bool& flag){
		isReadyToSend = flag;
	}
	
	void Estimator::setFlagComputeControl(const bool& flag){
		isReadyCompControl = flag;
	}

	void Estimator::setRcvArrayZero(void){
		for(int i=0;i<nOfAgents;i++){
			if(rcvArray(i)>_RECEIVED){
				rcvArray(i) = _EMPTY;
			}
		}
		cout << "rcvArray reset and = " << rcvArray.transpose() << endl;
	}

	void Estimator::setRcvArrayZeroTotal(void){
		for(int i=0;i<nOfAgents;i++){
				rcvArray(i) = _EMPTY;
		}
		cout << "rcvArray reset and = " << rcvArray.transpose() << endl;
	}

	void Estimator::setCmdAgentDone(const int& agent, const VectorQuat& input){
		rcvArray(agent) = _DONE;
		bfStruct[agent][bfSize-1][0].upost = input;
	}

	void Estimator::setToken(const bool& flag){
		flagSentToken = flag;
	}
	
	void Estimator::setCurrentYaw(const double& yawValue, const int& agent){
		yawNow[agent] = yawValue;
	}
	
	void Estimator::ResetForEstimPause(void){

		setFlagComputeControl(true);
		setToken(false);					
		rcvArray = rcvArray.Zero();
		rcvArrayBuffer = rcvArrayBuffer.Zero();
		setZeroAllBuffers();
		tGlobalSendCont      = -1;
		flagEnter			= true;
	}

	void Estimator::setZeroAllBuffers(void){

		Buffer msgDrone;
		// Initialize a void message in the Buffer;
		msgDrone.index 		= 0;
		msgDrone.tsSensor   = 0;
		msgDrone.tsArrival  = 0;
		msgDrone.tGSendCont = 0;
		msgDrone.upost      << 0,0,0,0;
		msgDrone.upre      	<< 0,0,0,0;
		msgDrone.data 		<< 0,0,0,0,0,0,0,0;
		
		for (int i = 0;i < nOfAgents ;i++){
			for (int j = 0;j < bfSize ;j++){
				for (int k = 0;k < 2 ;k++){
						bfStruct[i][j][k] = msgDrone;
				}
			}
		}
		
		for (int i = 0;i < bfSize ;i++){
			bfTemp[i] = msgDrone;
			for (int j = 0;j < _BUFMAXSIZE ;j++){
				bfTempPending[i][j] = msgDrone;
			}
		}
	}

	void Estimator::setIsFlagEnable(const bool& value){
		isFlagEnableMAS = value;
	}
	
	void Estimator::setReuseEstimate(const bool& value){
		flagReuseEstimation = value;
	}

	void Estimator::setFlagEmergencyStop(const bool& value){
		flagEmergencyStop = value;
	}	

	void Estimator::setIsOdomStarted(const bool& value,const int& agent){
		isOdomStarted(agent) = value;
	}	

	void Estimator::ZeroIsOdomStarted(void){
		isOdomStarted = isOdomStarted.Zero();
	}	

	void Estimator::setPoseZero(const VectorQuat& poseValue, const int& agent){
		
		yaw0[agent] = poseValue(3);

		RotGlobal[agent].block<2,2>(0,0) << cos(yaw0[agent]), -sin(yaw0[agent]),
									 		sin(yaw0[agent]),  cos(yaw0[agent]);
		pose0[agent] = poseValue;
	}	

	void Estimator::setPosition(Vector3axes& position, const int& agent){
		position 		= RotGlobal[agent].transpose()*(position - pose0[agent].head(3));
	}	

	void Estimator::setOrientation(const VectorQuat& orientationValue, double& yaw, const int& agent, VectorQuat& velocity){
	
		Vector3axes rpy;
				
		VectorQuat orientationRaw;

		Matrix4d Rot;

		Rot = Rot.Identity();

		orientationRaw = orientationValue;

	    Conversion::quat2angleZYX(rpy,orientationRaw);

	    yaw = angles::normalize_angle(rpy(2)-yaw0[agent]);
		
		Rot.block<2,2>(0,0) << 	cos(yaw), -sin(yaw),
								sin(yaw),  cos(yaw);		

		velocity = Rot*velocity;
	}	

	void Estimator::setOrientationVicon(const VectorQuat& orientationValue, double& yaw, const int& agent){
	
		Vector3axes rpy;
	    Conversion::quat2angleZYX(rpy,orientationValue);

	    yaw = angles::normalize_angle(rpy(2)-yaw0[agent]);
				
	}

	void Estimator::setFlagVicon(const bool& value){
		flagVicon = value;
	}

	void Estimator::setKalmanX(const Vector8d& value) {
		x_kalman = value;
	}

	void Estimator::setKalmanP(const Matrix8d& value) {
		P_kalman = value;
	}

	void Estimator::setDropProbability(const double& value){
		dropProbability = value;
	}
	
	void Estimator::setTimeNext(const double& value){
		tGlobalSendCont = value;
	}
	
	void Estimator::setLastTimeSent(const double& value){
		lastTimeSent = value;
	}


	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	/* ########################################                 GETTERS                 ##########################################*/
	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/


	double Estimator::getTimeShifted(const double& timeValue){
		return timeValue - timeStart;
	}

	bool Estimator::getToken(void){
		return flagSentToken;
	}

	double Estimator::getLastTimeSent(void){
		return lastTimeSent;
	}

	double Estimator::getTimeNext(void){
		return tGlobalSendCont;
	}

	Matrix4d Estimator::getK2(void){
		return K2;
	}

	Estimator::Buffer Estimator::getBuffer(const int index){
		return bfTemp[index];
	}

	Vector8d Estimator::getK(void){
		return K;
	}

	bool Estimator::getFlagReadyToSend(void){
		return isReadyToSend;
	}

	bool Estimator::getFlagComputeControl(void){
		return isReadyCompControl;
	}

	bool Estimator::getFlagEmergencyStop(void){
		return flagEmergencyStop;
	}	

	bool Estimator::getFlagEnter(void){
		return flagEnter;
	}

	bool Estimator::getIsFlagEnable(void){
		return isFlagEnableMAS;
	}

	bool Estimator::getReuseEstimate(void){
		return flagReuseEstimation;
	}	

	bool Estimator::getIsOdomStarted(const int& agent){

		return isOdomStarted(agent);
	}	

	double Estimator::getUpdateRate(void){
		return updateRate;
	}
	
	bool Estimator::getFlagVicon(void){
		return flagVicon;
	}

	Vector8d Estimator::getKalmanX(void){
		return x_kalman;
	}

	Matrix8d Estimator::getKalmanP(void){
		return P_kalman;
	}

	double Estimator::getCurrentYaw(const int& agent){
		return yawNow[agent];
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: getThisTimeSend
	*	  Created by: jrsbenevides
	*  Last Modified: jrsbenevides
	*
	*  	 Description: 1. Checks current time
	*				  2. Based on the predefined value of nexTimeToSend (started at -1, and changed to zero as soon as first message strikes)
	*					 it will change the value of timeSend based on a time basis defined by the updateRate.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	double Estimator::getThisTimeSend(void){
		
		double timeNow;

		timeNow = ros::Time::now().toSec();

		if(timeNow >= tGlobalSendCont - updateRate*coeffUpdRate){ //We passed already
			if(tGlobalSendCont > 0){						//After every iteration
				if(getToken() ==  true){
					cout << "Nova Computação de tNext..." << endl;
					cout << "Antiga: " << getTimeShifted(tGlobalSendCont) <<  endl;
					tGlobalSendCont += updateRate; //Do no update next Time to Send unless previous message was already sent
					cout << "Nova: " << getTimeShifted(tGlobalSendCont) <<  endl;

					setToken(false);
					
				}
			} else if(tGlobalSendCont == 0) {			//Only on first iteration.
				// tGlobalSendCont = timeNow + updateRate;
				cout << "Nova Computação de tNext..." << endl;
				cout << "Antiga: " << getTimeShifted(tGlobalSendCont) <<  endl;
				tGlobalSendCont = bfTemp[0].tsArrival + updateRate; // FOR MAS: CHANGE 0 TO FIRST MSG INDEX
				cout << "Nova: " << getTimeShifted(tGlobalSendCont) <<  endl;
			}
		}

		if(tGlobalSendCont>0){
			flagTickStart = true;
		}
		return tGlobalSendCont;
	}


	Vector8d Estimator::getEstimatePose(const int agent){
		return estPose[agent];
	}
	
	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	/* #####################################            REGULAR FUNCTIONS                 ########################################*/
	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/


	void Estimator::checkSendingConditions(void){
		// Checks if it is ready to go timewise and based on starting condition

		double tNow,tCompare;

		tNow = ros::Time::now().toSec();
		tCompare = tGlobalSendCont - updateRate*coeffUpdRate;
		
		if(flagTickStart == true){ //It means that tGlobalSendCont SHOULD have a meaningful computing value
			
			// if(tNow > tCompare - 0.005){ //0.005 tá carteado, mas ta valendo...em caso de tuning, deve ser um valor menor q o updateRate do droneMain
			// 	while(tNow < tCompare){
			// 		tNow = ros::Time::now().toSec();
			// 	}
			// }
			
			if(tNow >= tCompare){ //Computation time,send, receiving and implementing = updateRate*0.1 = user definedlegalgeasdasdasdasdadadasdasdasdadasdadadadadadadadadad
				
				if((getToken() == false)&&(tNow - getLastTimeSent() > 0.3*updateRate)){
					cout << "flagSentToken = false" << endl;
					setFlagReadyToSend(true);
				} else {
					cout << "flagSentToken = true" << endl;
					setFlagReadyToSend(false);
				}
			}
		}
	}


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: DvKalman
	*	  Created by: jrsbenevides
	*  Last Modified: August 30th 2021 
	*
	*  	 Description: Estimates linear velocity from Vicon position - Kalman Filter.
	*				  1. Updates A matrix based on current sampling time;
	* 				  2. Reads variable and error covariance estimate from previous time instant;
	*				  3. Performs simple Kalman Filter;	
	* 				  4. Stores variable and error covariance estimate from this time instant;		  
	* 				  5. Returns estimated velocity.		  
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	VectorQuat Estimator::DvKalman(const VectorQuat& poseCurrent, const double& timeNow, const double& timePast){
		
		double dt;
		VectorQuat estLinearVel, z;

		Matrix4d 	aux;
		Matrix8d 	P, Pp;
		Vector8d 	x, xp;
		Matrix8x4	K;

		z = poseCurrent;

		//ATTENTION: estimate x = [position;velocity];

		dt 		= timeNow - timePast;
		// cout << "Tempo de Kalman: " << dt << endl;
		// dt 		= 0.05;

		A_kalman.block<4,4>(0,4) << dt,  0,  0,  0,
									 0, dt,  0,  0,
									 0,  0, dt,  0,
									 0,  0,  0, dt;
	
		x = getKalmanX();
		P = getKalmanP();

		xp 	= A_kalman*x;
		Pp 	= A_kalman*P*A_kalman.transpose()  + Q_kalman;
		aux = H_kalman*Pp*H_kalman.transpose() + R_kalman;
		K 	= Pp*H_kalman.transpose()*aux.inverse();
		x 	= xp + K*(z - H_kalman*xp);
		P 	= Pp - K*H_kalman*Pp;

		setKalmanX(x);
		setKalmanP(P);

		estLinearVel 	<<  x.tail(4); //Get the last 4 elements, which is exactly the estimated velocity
		
		return estLinearVel;
	}


	void Estimator::pubMyLog(const int& valStart){ //If valStart

		drone_dev::BufferType bff;
		
		if(bfStruct[0][0][0].index >= thrCompEstimation){
			for(int i = valStart;i<bfSize;i++){
				bff.index 		= bfStruct[0][i][0].index;
				bff.tsSensor 	= bfStruct[0][i][0].tsSensor;
				bff.tsArrival 	= bfStruct[0][i][0].tsArrival;
				bff.tGSendCont 	= bfStruct[0][i][0].tGSendCont;
				bff.alpha		= estParam(0);
				bff.beta		= estParam(1);
				for(int k = 0;k<8;k++){
					bff.data[k] 	= bfStruct[0][i][0].data(k);
					if(k<4){
						bff.upre[k] 	= bfStruct[0][i][0].upre(k);
						bff.upost[k] 	= bfStruct[0][i][0].upost(k);
					}
				}
				log_publisher.publish(bff);
			}
		}		
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: initEstimator
	*	  Created by: jrsbenevides
	*  Last Modified: jrsbenevides
	*
	*  	 Description: 1. Initialize essential functions for trajectory tracking;
	*				  2. Initialize parameters and default values;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Estimator::initEstimator(void){

		srand(time(NULL));

		GeneralParameters paramZero;
		
		rcvArray = rcvArray.Zero();
		rcvArrayBuffer = rcvArrayBuffer.Zero();

		rcvViconFlag = true;

		cout << "Starting Estimator Node ...\n" << endl;

		
		counter = 0; //para testes apenas
		
		_EMPTY 		= 0;
		_RECEIVED 	= 1;
		_ESTIMATED	= 7;
		_DONE 		= 15;

		nLossMax 			= 2; //default value
		lossCount			= 0; //counts how many dropouts in a row;
		setDropProbability(0);

		//Initialize flags

		setFlagVicon(false);
		flagEnter			= true;
		flagDebug 			= true;
		flagTickStart 		= false;
		ZeroIsOdomStarted();
		setFlagEmergencyStop(false);
		setIsFlagEnable(false); 
		setFlagReadyToSend(false);
		setFlagComputeControl(true);
		setToken(false);
		setReuseEstimate(false);
		
		
		
		updateRate          = 0.05; //20Hz
		coeffUpdRate		= 0.1; //0.0217;
		isCMHEenabled		= 0;
		nOfAgents			= _NOFAGENTS;
		bfSize              = _BFSIZE;
		thrCompEstimation 	= 50; //5000*bfSize;
		PI 					= 3.141592653589793;
   		t 			  		= 0.0;
		stepT				= 1; //number of steps in integration
		tGlobalSendCont     = -1;
		K 					<<  1.74199, 0.94016, 1.54413, 0.89628, 3.34885, 3.29467, 6.51209, 3.92187;
		Rotation 			= Rotation.Identity();

		loadTopics(n);
		loadSettings(n);

		//Update Model
		updateModel(0);


		//dvKalman Zero
		A_kalman				= A_kalman.Identity();
		P_kalman				= P_kalman.Identity();
		Q_kalman				= 15*Q_kalman.Identity();
		
		Q_kalman.block<4,4>(0,0) <<  1,  0,  0,  0,
									 0,  1,  0,  0,
									 0,  0,  1,  0,
									 0,  0,  0,  1;

		R_kalman				= 0.1*R_kalman.Identity();

		H_kalman				<< H_kalman.Zero();

		H_kalman.block<4,4>(0,0) <<  1,  0,  0,  0,
									 0,  1,  0,  0,
									 0,  0,  1,  0,
									 0,  0,  0,  1;

		x_kalman				= x_kalman.Zero();

		//Mount zero class
		paramZero.sigmat = 0.01;
		paramZero.t1 	 = 0.0;
		paramZero.tn 	 = 0.0;
		paramZero.tnbar	 = 0.0;

		

		//EKF Parameters
		F = F.Identity();
		Q = 1*Q.Identity();			 
		R = 0.1*R.Identity();
		for(int i = 0;i<nOfAgents;i++){
			P[i] << P[i].Identity();

			//pose zero parameters
			RotGlobal[i] << RotGlobal[i].Identity();
			pose0[i]  << pose0[i].Zero();
			yaw0[i] = 0.0;
			yawNow[i] = 0.0;
			genParam[i] = paramZero;
		}

		setZeroAllBuffers();
		setLastTimeSent(0);


		countRcvMsg = 0;
		timeStart = ros::Time::now().toSec();
	}	

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: load Topics
	*	  Created by: jrsbenevides
	*  Last Modified: jrsbenevides
	*
	*  	 Description: 1. Define the ROS Topics and its types of messages;
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		void Estimator::loadTopics(ros::NodeHandle &n) {

		joy_subscriber 	   	 = n.subscribe<sensor_msgs::Joy>("/drone/joy", 1, &Estimator::joyCallback, this);
		// log_publisher	 	 = n.advertise<drone_dev::BufferType>("log_debug",1);
		odomRcv_subscriber   = n.subscribe<nav_msgs::Odometry>("/odom_global", _NOFAGENTS, &Estimator::odomRcvCallback, this);
		vicon_subscriber 	 = n.subscribe<geometry_msgs::TransformStamped>("/vicon/bebop/bebop", 1, &Estimator::viconRcvCallback, this);
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

	void Estimator::loadSettings(ros::NodeHandle &n) {

		if (n.getParam("/drone/isCMHEenabled",isCMHEenabled)) {
			cout << "isCMHEenabled = " << isCMHEenabled << endl;
		}

		if (n.getParam("/drone/nOfAgents",nOfAgents)) {
			cout << "Number of Agents = " << nOfAgents << endl;
		}

		if (n.getParam("/drone/bfSize",bfSize)) {
			cout << "Buffer Size = " << bfSize << endl;
		}
		
		vector<double> K;
		if (n.getParam("/drone/K",K)) {
			setK(Vector8d::Map(&K[0],8));
			cout << "K = [" << getK().transpose() << " ]" << endl;
		}

		double dropProbability;
		if (n.getParam("/drone/dropProbability",dropProbability)) {
			setDropProbability(dropProbability);
			cout << "dropProbability = " << dropProbability << endl;
		}

		if (n.getParam("/drone/nLossMax",nLossMax)) {
			cout << "nLossMax = " << nLossMax << endl;
		}


	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: updateModel
	*	  Created by: jrsbenevides
	*  Last Modified: jrsbenevides
	*
	*  	 Description: 1. Update Robot Model Based on the current orientation (TBD) - FALTA LER 
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Estimator::updateModel(const double& yaw){

		Matrix4d Ac,Bc;

		double sinyaw,cosyaw;

		sinyaw = sin(yaw);
		cosyaw = cos(yaw);

		Rotation.block<2,2>(0,0) << cosyaw, -sinyaw,
				 	 	 	 	 	sinyaw,  cosyaw;

		Ac << K(1)*cosyaw, -K(3)*sinyaw, 	0,      0,
			  K(1)*sinyaw,  K(3)*cosyaw, 	0,      0,
			  			0,			  0, K(5),  	0,
			  			0,            0,    0,   K(7);

		Ac = -1*Ac;				  	 

		Bc << K(0)*cosyaw, -K(2)*sinyaw, 	0,      0,
			  K(0)*sinyaw,  K(2)*cosyaw, 	0,      0,
						0,		  	  0, K(4),   	0,
						0,            0,    0,   K(6);

		//Ajeitar carregamento de A,B
		K2 = Ac*Rotation.transpose();

		A << K2, MatrixXd::Zero(4,4),
			 MatrixXd::Identity(4,4), MatrixXd::Zero(4,4);
		B << Bc,
			 MatrixXd::Zero(4,4);

		//DEBUG
		// cout << "A: " << A << endl;
		// cout << "B: " << B << endl;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: nextAgentToCompute
	*	  Created by: jrsbenevides
	*  Last Modified: jrsbenevides
	*
	*  	 Description: 1. Returns the index of the agent with pending messages to be treated
	*                    Returns -1 in case none of the elements in rcvArray is _RECEIVED
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	int Estimator::nextAgentToCompute(void){

		bool flag = false;
		int agent = -1;
		
		for(int i=0;(i<nOfAgents)&&(flag==false);i++){
			if(rcvArray(i)==_RECEIVED){
				agent = i;
				flag=true;
			}
		}
		return agent;
	}


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: nextAgentToSend
	*	  Created by: jrsbenevides
	*  Last Modified: jrsbenevides
	*
	*  	 Description: 1. Returns the index of the agent in the line to organize and send
	*                    Returns -1 in case none of the elements in rcvArray is _ESTIMATED
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	int Estimator::nextAgentToSend(void){

		bool flag = false;
		int agent = -1;
		
		for(int i=0;(i<nOfAgents)&&(flag==false);i++){
			if(rcvArray(i)==_ESTIMATED){
				agent = i;
				flag=true;
			}
		}
		return agent;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: AddPkt2Buffer - EYEDEBUG OK
	*	  Created by: jrsbenevides
	*  Last Modified: jrsbenevides
	*
	*  	 Description: 1. Adds (or discards) package to general buffer;
	*				  2. Returns true/false if OK/NG
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	bool Estimator::AddPkt2Buffer(const Buffer& pkt, const int agent){

		int i = bfSize - 1; // 0 to N-1 => size N
		bool terminalCondition = false;

		cout << "Adicionando pkt ao buffer" << endl;

		while((terminalCondition == false)&&(i>=0)){
			if(pkt.tsSensor > bfStruct[agent][i][0].tsSensor){
				UpdateBuffer(pkt,agent,i);
				terminalCondition = true;
			}
			else{
				// cout << "eita" << endl;
				// cout << "O tempo de sensor no espaco " << i << " era de: " << bfStruct[agent][i][0].tsSensor << endl;
				// cout << "O tempo de sensor no pkt era de: " << pkt.tsSensor << endl;
				// cout << "E a diferença era de: " << pkt.tsSensor - bfStruct[agent][i][0].tsSensor << endl;
				i--;
			}
		}

		// if(terminalCondition == true){
		// 	cout << "Sucesso >> ";
		// 	if(pkt.tsSensor == bfStruct[agent][i][0].tsSensor)
		// 		cout << "total" << endl;
		// 	else
		// 		cout << "parcial" << endl;

		// }

		//indexBf = i;  //
		return terminalCondition;
		
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: UpdateBuffer - EYEDEBUG OK
	*	  Created by: jrsbenevides
	*  Last Modified: jrsbenevides
	*
	*  	 Description: 1. Succesfully replaces, on a single agent, the package pkt at position i and reorganizes the buffer.
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Estimator::UpdateBuffer(const Buffer& pkt, const int& agent,const int& i){
		
		int curIndex;
		VectorQuat linearVel;
		Vector8d data;
		Matrix3d RotNow;
		
		// Copies this row and saves it.
		for(int k=0; k < bfSize ; k++){
			bfStruct[agent][k][1] = bfStruct[agent][k][0];
		}

		// Stores current buffer index in order to increment at the end.
		curIndex = bfStruct[agent][0][0].index;

		// cout << "O agente eh " << agent << " e o pacote eh " << curIndex << " e i = " << i << endl; //DEBUG!!!

		// Shifts old packages to accomodate new package
		if(i > 0){
			for(int k = 0;k < i; k++){
				bfStruct[agent][k][0] = bfStruct[agent][k+1][0];
			}
		}
		
		// Stores new package
		bfStruct[agent][i][0] = pkt;
		
		// Updates buffer index
		bfStruct[agent][0][0].index = curIndex + 1;
		// cout << "Agora o agente eh " << agent << " e o pacote eh " << bfStruct[agent][0][0].index << endl; //DEBUG!!!

		// if(getFlagVicon()){ //If Vicon is selected, now is the time to compute velocity input
		// 	//To get velocity...now we will search in the current buffer
		// 	data 						<< pkt.data;	
		// 	linearVel 					= DvKalman(data.tail(4),bfStruct[agent][i][0].tsSensor,bfStruct[agent][i][1].tsSensor);
		// 	cout << "Linear Velocity: " << linearVel << endl;
		// 	// RotNow << cos(data(7)), -sin(data(7)),0,
		// 	// 		  sin(data(7)),  cos(data(7)),0,
		// 	// 		  			 0,				0,1;
		// 	// data.head(4) 				<< Rot.transpose()*linearVel; //Transforming back to local 
		// 	data.head(4) 				<< linearVel; //Transforming back to local 
		// 	bfStruct[agent][i][0].data 	<< data;	

		// 	// if(i == (bfSize-1)){ //If this is the most recent message
		// 		setCurrentYaw(data(7),agent);
		// 	// }							
		// }

		//Fills upre and upost with known input data
		if(i>0){
			bfStruct[agent][i][0].upre = bfStruct[agent][i-1][0].upost;
		}else{
			bfStruct[agent][i][0].upre = bfStruct[agent][i][1].upost; //In the case of the first (i==0), we copy from the dropped element
		}
		if(i<bfSize-1){
			bfStruct[agent][i][0].upost = bfStruct[agent][i+1][0].upre;
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

	void Estimator::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
		// if(isCMHEenabled == 1){
		//   /*Enables "Automatic Control Mode" while pressing this button*/
		//   if(joy->buttons[6]){
		//     cout << "on" << endl;
		//   }
		//   else{
		// 	cout << "off" << endl;
		//   }
		// }
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: updateEKF
	*	  Created by: jrsbenevides
	*  Last Modified: 
	*
	*  	 Description: Performs the prediction and update steps on the EKF estimator
	*	 Status: Debugged on March 25th
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		
	void Estimator::updateEKF(const int agent){
		
		Vector2d sOld,s;
		Vector8d q,x,h,zk,y;
		VectorQuat uPre,uPost;
		double tBar,t1bar,t2bar,t1, deltaT, dPdAlpha, dPdBeta, dDeltaT1dAlpha, dDeltaT1dBeta, dDeltaT2dAlpha, dDeltaT2dBeta;
		Matrix8d Ak;
		Matrix8x4 Bk;
		Matrix2x3 HDerivMask;
		Matrix8x3 HMtxMask;
		Matrix8x2 Hk;
		Matrix2x8 K;
		
		s << estParam.block<2,1>(0,agent);
		for(int k =0 ; k<(bfSize-1); k++){
			// Keeps old estimation temporarily
			sOld << s;

			// Building Variable Elements
			q << bfStruct[agent][k][0].data;
			tBar = (1/s(0))*bfStruct[agent][k][0].tGSendCont - (s(1)/s(0));
			deltaT = (tBar - bfStruct[agent][k][0].tsSensor)/stepT;
			Ak = s(0)*A;
			Bk = s(0)*B;
			x << q;
			uPre = bfStruct[agent][k][0].upre;
			for(int j = 0;j<stepT;j++){
				x = (MatrixXd::Identity(8,8) + deltaT*Ak)*x + deltaT*Bk*uPre;
			}

			// Dynamics update - PART 2 - From arrival to next sampling- Using uComp
			deltaT = (bfStruct[agent][k+1][0].tsSensor - tBar)/stepT;
			uPost = bfStruct[agent][k][0].upost;
			for(int j = 0;j<stepT;j++){
				x = (MatrixXd::Identity(8,8) + deltaT*Ak)*x + deltaT*Bk*uPost;
			}
			h << x;
			zk << bfStruct[agent][k+1][0].data;
			// Hk parts: dh/d(alpha) = 
			t1bar = bfStruct[agent][k][0].tsSensor;
			t2bar = bfStruct[agent][k+1][0].tsSensor;
			t1    = bfStruct[agent][k][0].tGSendCont;
			// dP/d(alpha) = AQUI TEM ESPAÇO PRA OTIMIZAR CODIGO!!
			dPdAlpha = (t1 - s(1))*(t1bar + t2bar) - 2*s(0)*(t1bar*t2bar);
			dPdBeta  = 2*(t1 - s(1))-s(0)*(t1bar + t2bar); 

			dDeltaT1dAlpha = -t1bar;
			dDeltaT1dBeta  = -1;
			dDeltaT2dAlpha = t2bar;
			dDeltaT2dBeta  = 1;
			HDerivMask << dPdAlpha, dDeltaT1dAlpha, dDeltaT2dAlpha,
						  dPdBeta,  dDeltaT1dBeta,  dDeltaT2dBeta;
			HMtxMask   << (A*A*q + A*B*uPre), (A*q+B*uPre), (A*q+B*uPost);
			Hk = HMtxMask*HDerivMask.transpose();						  
				
			//Kalman Steps
			P[agent] = F*P[agent]*F.transpose() + Q;
			y = zk - h;
			S << Hk*P[agent]*Hk.transpose() + R;
			K << P[agent]*Hk.transpose()*S.inverse();
			s = s + K*y;
			s = isSinsideTrapezoid(s,sOld,agent,k);
			P[agent] = (MatrixXd::Identity(2,2) - K*Hk)*P[agent];
		}
		estParam.block<2,1>(0,agent) << s; 
	}

	void Estimator::updateEKF_2D(const int agent){
		
		Vector2d sOld,s,uPre2d,uPost2d;
		VectorQuat h2d,zk2d,y2d,q2d;
		Vector8d q,x,h,zk,y;
		VectorQuat uPre,uPost;
		double tBar,t1bar,t2bar,t1, deltaT, dPdAlpha, dPdBeta, dDeltaT1dAlpha, dDeltaT1dBeta, dDeltaT2dAlpha, dDeltaT2dBeta;
		Matrix8d Ak;
		Matrix8x4 Bk;
		Matrix2x3 HDerivMask;
		Matrix4x3 HMtxMask;
		Matrix2x8 K;
		Matrix4d  R2d,S2d,A2d;
		Matrix4x2 Hk2d,B2d;
		Matrix2x4 K2d;


		//Load q just to recover yaw angle
		q << bfStruct[agent][bfSize-1][0].data;

		//Update matrices A and B
		updateModel(q(7)); //7 = index for last received yaw

		A2d << 	A.block<2,2>(0,0),A.block<2,2>(0,4),
				A.block<2,2>(4,0),A.block<2,2>(4,4);  //Block of size (p,q), starting at (i,j) 	matrix.block<p,q>(i,j);

		B2d << 	B.block<2,2>(0,0),
				B.block<2,2>(4,0);
		
		R2d = R2d.Identity();

		s << estParam.block<2,1>(0,agent);
		for(int k =0 ; k<(bfSize-1); k++){
			// Keeps old estimation temporarily
			sOld << s;

			// Building Variable Elements
			q << bfStruct[agent][k][0].data;
			q2d 	<< 	q.head(2),
						q.segment(4,2); 
			tBar = (1/s(0))*getTimeShifted(bfStruct[agent][k][0].tGSendCont) - (s(1)/s(0));
			deltaT = (tBar - getTimeShifted(bfStruct[agent][k][0].tsSensor))/stepT;
			Ak = s(0)*A;
			Bk = s(0)*B;
			x << q;
			uPre = bfStruct[agent][k][0].upre;
			uPre2d << uPre.head(2);
			for(int j = 0;j<stepT;j++){
				x = (MatrixXd::Identity(8,8) + deltaT*Ak)*x + deltaT*Bk*uPre;
			}

			// Dynamics update - PART 2 - From arrival to next sampling- Using uComp
			deltaT = (getTimeShifted(bfStruct[agent][k+1][0].tsSensor - tBar))/stepT;
			uPost = bfStruct[agent][k][0].upost;
			uPost2d << uPost.head(2);
			for(int j = 0;j<stepT;j++){
				x = (MatrixXd::Identity(8,8) + deltaT*Ak)*x + deltaT*Bk*uPost;
			}
			h << x;
			zk << bfStruct[agent][k+1][0].data;

			h2d 	<< 	h.head(2),
						h.segment(4,2); //vector.segment(i,n) == Block containing n elements, starting at position i
			zk2d 	<< 	zk.head(2),
						zk.segment(4,2);
			// Hk parts: dh/d(alpha) = 
			t1bar = getTimeShifted(bfStruct[agent][k][0].tsSensor);
			t2bar = getTimeShifted(bfStruct[agent][k+1][0].tsSensor);
			t1    = getTimeShifted(bfStruct[agent][k][0].tGSendCont);
			// dP/d(alpha) = AQUI TEM ESPAÇO PRA OTIMIZAR CODIGO!!
			dPdAlpha = (t1 - s(1))*(t1bar + t2bar) - 2*s(0)*(t1bar*t2bar);
			dPdBeta  = 2*(t1 - s(1))-s(0)*(t1bar + t2bar); 

			dDeltaT1dAlpha = -t1bar;
			dDeltaT1dBeta  = -1;
			dDeltaT2dAlpha = t2bar;
			dDeltaT2dBeta  = 1;
			HDerivMask << dPdAlpha, dDeltaT1dAlpha, dDeltaT2dAlpha,
						  dPdBeta,  dDeltaT1dBeta,  dDeltaT2dBeta;
			HMtxMask   << (A2d*A2d*q2d + A2d*B2d*uPre2d), (A2d*q2d+B2d*uPre2d), (A2d*q2d+B2d*uPost2d);
			Hk2d = HMtxMask*HDerivMask.transpose();						  
				
			// //Kalman Steps
			P[agent] = F*P[agent]*F.transpose() + Q;
			y2d = zk2d - h2d;
			S2d << Hk2d*P[agent]*Hk2d.transpose() + R2d;

			K2d << P[agent]*Hk2d.transpose()*S2d.inverse();
			s = s + K2d*y2d;
			s = isSinsideTrapezoid(s,sOld,agent,k);
			P[agent] = (MatrixXd::Identity(2,2) - K2d*Hk2d)*P[agent];
		}
		estParam.block<2,1>(0,agent) << s; 
	}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: updateEKF
	*	  Created by: jrsbenevides
	*  Last Modified: 
	*
	*  	 Description: Performs the prediction and update steps on the EKF estimator
	*	 Status: Debugged on March 25th
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		

	void Estimator::updateEKF_identGlobal(const int agent){
		
		Vector2d sOld,s;
		Vector8d q,x,h,zk,y;
		VectorQuat uPre,uPost;
		double tBar,t1bar,t2bar,t1, deltaT, dPdAlpha, dPdBeta, dDeltaT1dAlpha, dDeltaT1dBeta, dDeltaT2dAlpha, dDeltaT2dBeta;
		Matrix8d Ak;
		Matrix8x4 Bk;
		Matrix2x3 HDerivMask;
		Matrix8x3 HMtxMask;
		Matrix8x2 Hk,Hnew;
		Matrix2x8 K;

		
		s << estParam.block<2,1>(0,agent);
		cout << "ESTIM era " << s.transpose() << endl;
		for(int k =0 ; k<(bfSize-1); k++){
			// Keeps old estimation temporarily
			sOld << s;

			// Building Variable Elements
			q << bfStruct[agent][k][0].data;
			tBar = bfStruct[agent][k][0].tGSendCont;
			deltaT = (tBar - (s(0)*bfStruct[agent][k][0].tsSensor + s(1)))/stepT;
			Ak = A;
			Bk = B;
			x << q;
			uPre = bfStruct[agent][k][0].upre;
			for(int j = 0;j<stepT;j++){
				x = (MatrixXd::Identity(8,8) + deltaT*Ak)*x + deltaT*Bk*uPre;
			}

			// Dynamics update - PART 2 - From arrival to next sampling- Using uComp
			deltaT = (s(0)*bfStruct[agent][k+1][0].tsSensor + s(1) - tBar)/stepT;
			uPost = bfStruct[agent][k][0].upost;
			for(int j = 0;j<stepT;j++){
				x = (MatrixXd::Identity(8,8) + deltaT*Ak)*x + deltaT*Bk*uPost;
			}
			h << x;
			zk << bfStruct[agent][k+1][0].data;
			// Hk parts: dh/d(alpha) = 
			t1bar = bfStruct[agent][k][0].tsSensor;
			t2bar = bfStruct[agent][k+1][0].tsSensor;
			t1    = bfStruct[agent][k][0].tGSendCont;
			// dP/d(alpha) = AQUI TEM ESPAÇO PRA OTIMIZAR CODIGO!!
			dPdAlpha = (t1 - s(1))*(t1bar + t2bar) - 2*s(0)*(t1bar*t2bar);
			dPdBeta  = 2*(t1 - s(1))-s(0)*(t1bar + t2bar); 

			dDeltaT1dAlpha = -t1bar;
			dDeltaT1dBeta  = -1;
			dDeltaT2dAlpha = t2bar;
			dDeltaT2dBeta  = 1;
			HDerivMask << dPdAlpha, dDeltaT1dAlpha, dDeltaT2dAlpha,
						  dPdBeta,  dDeltaT1dBeta,  dDeltaT2dBeta;
			HMtxMask   << (A*A*q + A*B*uPre), (A*q+B*uPre), (A*q+B*uPost);
			Hnew = Hnew.Zero();
			Hnew.block<8,1>(0,0) = (t2bar - t1bar)*A*q;
			Hk = HMtxMask*HDerivMask.transpose() + Hnew;						  
				
			//Kalman Steps
			P[agent] = F*P[agent]*F.transpose() + Q;
			y = zk - h;
			S << Hk*P[agent]*Hk.transpose() + R;
			K << P[agent]*Hk.transpose()*S.inverse();
			s << s + K*y;
			s = isSinsideTrapezoid(s,sOld,agent,k);
			P[agent] = (MatrixXd::Identity(2,2) - K*Hk)*P[agent];
		}
		estParam.block<2,1>(0,agent) << s; 
		cout << "ESTIM agora eh " << s.transpose() << endl;
	}	

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: isSinsideTrapezoid
	*	  Created by: jrsbenevides
	*  Last Modified: 
	*
	*  	 Description: Checks if the computed parameters lie inside the boundaries
	*/		   
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		

	Vector2d Estimator::isSinsideTrapezoid(const Vector2d& s, const Vector2d& sOld, const int& agent, const int& iter){

		double alpha, beta;
		Vector2d sNew;

		alpha = s(0);
		beta  = s(1);

		sNew = sOld;

		// if((alpha >= 1 - genParam[agent].sigmat) && (alpha <= 1 + genParam[agent].sigmat)){ // 		%Alpha lies inside boundaries for alpha
		// 	if((beta > getTimeShifted(bfStruct[agent][iter][0].tsArrival)-alpha*getTimeShifted(bfStruct[agent][iter+1][0].tsSensor)) && (beta < getTimeShifted(bfStruct[agent][iter][0].tsArrival)-alpha*getTimeShifted(bfStruct[agent][iter][0].tsSensor))){ //Beta lies inside boundaries for beta
		// 		sNew = s;
		// 	} 
		// }		
		
		if((beta > getTimeShifted(bfStruct[agent][iter][0].tsArrival)-alpha*getTimeShifted(bfStruct[agent][iter+1][0].tsSensor)) && (beta < getTimeShifted(bfStruct[agent][iter][0].tsArrival)-alpha*getTimeShifted(bfStruct[agent][iter][0].tsSensor))){ //Beta lies inside boundaries for beta
			sNew(1) = s(1); //Updates beta only
		} 

		return sNew;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: ComputeEstimation
	*	  Created by: jrsbenevides
	*  Last Modified: 
	*
	*  	 Description: EKF Estimator for Networked Control System
	*		   Steps: 	DEVO COLOCAR AQUI UNS CONDICIONAIS PRA SABER SE AS MENSAGENS JÁ CHEGARAM E SE JA PODE PROCESSAR, SENAO SO BYPASSA*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		

	void Estimator::ComputeEstimation(void){
		
		// Declare and start local variables
		bool status = false, flagExitSearch;
		int agent = 0,tambuf;
		double tBar, deltaT, tGlobalSendCont,tempValue,minVal,maxVal;
		
		Vector2d sEst;
		VectorQuat uComp, uPre;
		Vector8d x;
		
		Matrix8d Ak;
		 
		Matrix8x4 Bk;

		tGlobalSendCont = getThisTimeSend();

		if(rcvArray.maxCoeff() > _EMPTY){

			agent = nextAgentToCompute(); // Checks which agent should be handled next. -1 if waiting for computation. 
			
			if(agent>=0){
				
				bfTemp[agent].tGSendCont = tGlobalSendCont; //APRIMORAR AS CONDIÇÕES PARA ESSA DETERMINAÇÃO (msg adiantada, atrasada, etc...)
				
				cout  << "Tratando: Agente: " << agent << " e pkt: " << bfStruct[agent][0][0].index << endl; //DEBUG!!! 

				status = AddPkt2Buffer(bfTemp[agent],agent);
			
				if(status == true){ // 		If buffer received this new info:
					cout  << "Sucesso: Agente: " << agent << " e novo tamanho preenchido: " << bfStruct[agent][0][0].index << endl; //DEBUG!!! 
					if(bfStruct[agent][0][0].index >= thrCompEstimation){ //Buffer is ready to estimate
						if(bfStruct[agent][0][0].index == thrCompEstimation){ //%Initial guess for estimate
							if(true){  //Initial guess will be skipped if there was already a prior estimation (was getReuseEstimate()==false)
								
								for(int i=0;i<bfSize-1;i++){
									if(i==0){
										minVal = bfStruct[agent][i][0].tsArrival-bfStruct[agent][i][0].tsSensor;
										maxVal = bfStruct[agent][i][0].tsArrival-bfStruct[agent][i+1][0].tsSensor;
									} else{
										if(bfStruct[agent][i][0].tsArrival-bfStruct[agent][i][0].tsSensor < minVal){
											minVal = bfStruct[agent][i][0].tsArrival-bfStruct[agent][i][0].tsSensor;
										}
										if(bfStruct[agent][i][0].tsArrival-bfStruct[agent][i+1][0].tsSensor > maxVal){
											maxVal = bfStruct[agent][i][0].tsArrival-bfStruct[agent][i+1][0].tsSensor;
										}
									}
								}
								
								// estParam.block<2,1>(0,agent) << 1.0,
								// 								0.5*(bfStruct[agent][bfSize-1][0].tsArrival-bfStruct[agent][bfSize-1][0].tsSensor-bfStruct[agent][0][0].tsSensor);
								estParam.block<2,1>(0,agent) << 1.0,
																0.5*(minVal + maxVal);
								genParam[agent].t1     = bfStruct[agent][0][0].tsSensor;
								genParam[agent].tn     = bfStruct[agent][bfSize-1][0].tsArrival;
								genParam[agent].tnbar  = bfStruct[agent][bfSize-1][0].tsSensor;
								genParam[agent].sigmat = 0.01;	
								cout << "PRIMEIRISSIMO ESTIMADO!!: " <<  estParam.block<2,1>(0,agent) << endl;
							}
						}
						updateEKF(agent);
					}
					// else if(bfStruct[agent][0][0].index == 1){
					// 	bfStruct[agent][bfSize-1][0].upre << 0.0, 0.0, 0.0, 0.0;
					// }
					cout <<  "Montado o pacote " << bfStruct[agent][0][0].index <<  " do agente " << agent <<  endl;
				} else {
					//CASO NAO CONSIGA ADICIONAR AO PACOTE
					cout << "Pacote eh velho demais...esperar liberar o envio pra mandar de novo" << endl;
				}

				rcvArray(agent) = _ESTIMATED; // 7 == finished

				// 	% ####################################################
				// 	% ###########   STEP 1.3 Prediction Step  ############
				// 	% ####################################################
					
				// 	%Based on current estimate for s = [alpha,beta] for each agent, we
				// 	%predict the current state based on the last received sampled data  

				// cout << "Entrei aqui!" << endl;    
							
				if(bfStruct[agent][0][0].index > thrCompEstimation){ //Talvez aumentar o limiar para um valor maior que bfSize apresente um resultado melhor...quando estabilizar.
					sEst << estParam.block<2,1>(0,agent);

					cout << "Estimativa alpha, beta = " << sEst.transpose() << endl;
					
					tBar = (1/sEst(0))*tGlobalSendCont - (sEst(1)/sEst(0));
					deltaT = (tBar - bfStruct[agent][bfSize-1][0].tsSensor)/stepT;
					Ak   = sEst(0)*A;
					Bk   = sEst(0)*B;
					x    = bfStruct[agent][bfSize-1][0].data;
					uPre = bfStruct[agent][bfSize-1][0].upre;

					for(int j = 0; j<stepT; j++){
						x = (MatrixXd::Identity(8,8)+deltaT*Ak)*x + deltaT*Bk*uPre;
					}
				} else{
					x     = bfStruct[agent][bfSize-1][0].data;
					// uComp = bfStruct[agent][bfSize-1][0].upre;
				}	

				// Make it available for the controller
				setEstimatePose(x,agent);
							
			} else{
				//AGUARDANDO CHEGADA DE MENSAGEM OU FINALIZACAO DE CALCULO.
				// cout << "Buffer temporario já está vazio ou aguardando liberacao" << endl;

				if(rcvArrayBuffer.maxCoeff() > _EMPTY){ // There is something inside pending buffer!!
					// if(tGlobalSendCont - ros::Time::now().toSec() > availableTime){ //Is it worth treating now or not? based on available time (about to send?)
					flagExitSearch = false;
					
					cout << "Hora de recuperar a info de buffer pendente" << endl;

					for(int i=0;(i<nOfAgents)&&(!flagExitSearch);i++){
						tambuf = rcvArrayBuffer(i); //Amount of stored packages for a determined agent
						if(tambuf>0){
							if(rcvArray(i) == _EMPTY){
								cout << "Recuperando do agente " << i << " no idx " << tambuf -1 << endl;
								bfTemp[i] = bfTempPending[i][tambuf-1]; //tambuf-1 corresponds to the index of where that last information was stored
								rcvArray(i)= _RECEIVED;
								rcvArrayBuffer(i)--;
								flagExitSearch = true;
								cout << "Info de buffer pendente para o agente " << i << " esvaziada." << endl;
							}
						}
					}
					if(flagExitSearch == false){
						cout << "Buffer principal ainda esperando ser esvaziado para adicionar qualquer coisa" << endl;
					}
					// }
				}
				cout << "Buffer Chegada = " << rcvArray.transpose() << endl;
				cout << "Buffer Pendente = " << rcvArrayBuffer.transpose() << endl;
			}
		}
		
		// if(rcvArray.minCoeff() == _DONE){ //DEBUG....................... _DONE
		//if(rcvArray.minCoeff() == _ESTIMATED){
		
		// Checks if it is ready to go timewise and based on starting condition
		if(flagTickStart == true){ //It means that tGlobalSendCont SHOULD have a meaningful computing value
			if(tGlobalSendCont - ros::Time::now().toSec() <= updateRate*coeffUpdRate){ //Computation time,send, receiving and implementing = updateRate*0.1 = user defined
				if(getToken() == false){
					setFlagReadyToSend(true);
				} else {
					setFlagReadyToSend(false);
				}
			} 
			else {
				if(tGlobalSendCont - ros::Time::now().toSec() < updateRate){
					setFlagReadyToSend(false);
					setToken(false);
				}
			}
		}
		//}		
	}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* 		Function: ComputeEstimation
*	  Created by: jrsbenevides
*  Last Modified: 
*
*  	 Description: EKF Estimator for Networked Control System
*		   Steps: 	DEVO COLOCAR AQUI UNS CONDICIONAIS PRA SABER SE AS MENSAGENS JÁ CHEGARAM E SE JA PODE PROCESSAR, SENAO SO BYPASSA*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		

	void Estimator::ComputeEstimation_identGlobal(void){
		
		// Declare and start local variables
		bool status = false, flagExitSearch;
		int agent = 0,tambuf;
		double tBar, deltaT,tempValue,minVal,maxVal;
		

		Vector2d sEst;
		VectorQuat uComp, uPre;
		Vector8d x;
		
		Matrix8d Ak;
		Matrix8x4 Bk;

		//ROTINA DE DEBUG

		// if(tGlobalSendCont>0){
		// 	double tNowDebug;
		// 	tNowDebug = ros::Time::now().toSec();	
		// 	// tNowDebug = ros::Time::now().toSec();			
		// 	if(tNowDebug < tGlobalSendCont){
		// 		tNowDebug = tGlobalSendCont - tNowDebug;
		// 		if(tNowDebug <= updateRate){
		// 			cout << "GOOD!" << endl;
		// 		}
		// 	} else {
		// 		// tNowDebug -= tGlobalSendCont;
		// 		tNowDebug -= tGlobalSendCont;
		// 		// for (int i =0;i<1000;i++)
		// 			cout << "OOPS!! T = " << tNowDebug << endl;
		// 	}
		// }

		if(rcvArray.maxCoeff() > _EMPTY){

			agent = nextAgentToCompute(); // Checks which agent should be handled next. -1 if waiting for computation. 
			
			if(agent>=0){
				
				bfTemp[agent].tGSendCont = tGlobalSendCont; //APRIMORAR AS CONDIÇÕES PARA ESSA DETERMINAÇÃO (msg adiantada, atrasada, etc...)
				
				cout  << "Tratando: Agente: " << agent << " e pkt: " << bfStruct[agent][0][0].index << endl; //DEBUG!!! 

				status = AddPkt2Buffer(bfTemp[agent],agent);
			
				if(status == true){ // 		If buffer received this new info:
					cout  << "Sucesso: Agente: " << agent << " e novo tamanho preenchido: " << bfStruct[agent][0][0].index << endl; //DEBUG!!! 
					if(bfStruct[agent][0][0].index >= thrCompEstimation){ //Buffer is ready to estimate
						if(bfStruct[agent][0][0].index == thrCompEstimation){ //%Initial guess for estimate
							// if(true){  //Initial guess will be skipped if there was already a prior estimation (was getReuseEstimate()==false)
							genParam[agent].t1     = getTimeShifted(bfStruct[agent][0][0].tsSensor);
							genParam[agent].tn     = getTimeShifted(bfStruct[agent][bfSize-1][0].tsArrival);
							genParam[agent].tnbar  = getTimeShifted(bfStruct[agent][bfSize-1][0].tsSensor);
							genParam[agent].sigmat = 1e-6;	
							// estParam.block<2,1>(0,agent) << 1.0,
															0.5*(genParam[agent].tn-genParam[agent].tnbar-genParam[agent].t1);
							for(int i=0;i<bfSize-1;i++){
								if(i==0){
									minVal = bfStruct[agent][i][0].tsArrival-bfStruct[agent][i][0].tsSensor;
									maxVal = bfStruct[agent][i][0].tsArrival-bfStruct[agent][i+1][0].tsSensor;
								} else{
									if(bfStruct[agent][i][0].tsArrival-bfStruct[agent][i][0].tsSensor < minVal){
										minVal = bfStruct[agent][i][0].tsArrival-bfStruct[agent][i][0].tsSensor;
									}
									if(bfStruct[agent][i][0].tsArrival-bfStruct[agent][i+1][0].tsSensor > maxVal){
										maxVal = bfStruct[agent][i][0].tsArrival-bfStruct[agent][i+1][0].tsSensor;
									}
								}
							}	

							estParam.block<2,1>(0,agent) << 1.0,
															0.5*(minVal + maxVal);														
							
							cout << "PRIMEIRISSIMO ESTIMADO!!: " <<  estParam.block<2,1>(0,agent) << endl;
							// }else{
							// 	cout << "EU IGNOREI O PRIMEIRO LOOP DE INICIALIZACAO" << endl;
							// }
						}
						updateEKF_2D(agent);
					}
					// else if(bfStruct[agent][0][0].index == 1){
					// 	bfStruct[agent][bfSize-1][0].upre << 0.0, 0.0, 0.0, 0.0;
					// }
					cout <<  "Montado o pacote " << bfStruct[agent][0][0].index <<  " do agente " << agent <<  endl;
				} else {
					//CASO NAO CONSIGA ADICIONAR AO PACOTE
					cout << "Pacote eh velho demais...esperar liberar o envio pra mandar de novo" << endl;
				}

				rcvArray(agent) = _ESTIMATED; // 7 == finished

				// 	% ####################################################
				// 	% ###########   STEP 1.3 Prediction Step  ############
				// 	% ####################################################
					
				// 	%Based on current estimate for s = [alpha,beta] for each agent, we
				// 	%predict the current state based on the last received sampled data  

				// cout << "Entrei aqui!" << endl;    
							
				if(bfStruct[agent][0][0].index > thrCompEstimation){ //Talvez aumentar o limiar para um valor maior que bfSize apresente um resultado melhor...quando estabilizar.
					sEst << estParam.block<2,1>(0,agent);

					cout << "alpha = " << sEst(0) << " beta = " << sEst(1) << endl;
					
					tBar = getTimeShifted(tGlobalSendCont);
					deltaT = (tBar - (sEst(0)*getTimeShifted(bfStruct[agent][bfSize-1][0].tsSensor) + sEst(1)))/stepT;
					Ak   = A; //DEBATE SE POR ACASO AQUI E EMBAIXO NAO DEVERIA SER sEst(0)*A e sEst(0)*B
					Bk   = B;
					x    = bfStruct[agent][bfSize-1][0].data;
					uPre = bfStruct[agent][bfSize-1][0].upre;

					cout << "tBar = " << tGlobalSendCont << endl;
					cout << "tsSensor = " << bfStruct[agent][bfSize-1][0].tsSensor << endl;
					cout << "uPre = " << uPre << endl;

					cout << "deltaT = " << deltaT << endl;
					cout << "Ultimo dado = " << x.transpose() << endl;

					if(tGlobalSendCont > ros::Time::now().toSec()){
						for(int j = 0; j<stepT; j++){
							x = (MatrixXd::Identity(8,8)+deltaT*Ak)*x + deltaT*Bk*uPre;	
						}
					} else{
						cout << "Tempo de envio já passou!" << endl;
					}


					cout << "Predicao = " << x.transpose() << endl;
				} else{
					x     = bfStruct[agent][bfSize-1][0].data;
					// uComp = bfStruct[agent][bfSize-1][0].upre;
				}	

				// Make it available for the controller
				setEstimatePose(x,agent);
							
			} else{
				//AGUARDANDO CHEGADA DE MENSAGEM OU FINALIZACAO DE CALCULO.
				// cout << "Buffer temporario já está vazio ou aguardando liberacao" << endl;

				if(rcvArrayBuffer.maxCoeff() > _EMPTY){ // There is something inside pending buffer!!
					// if(tGlobalSendCont - ros::Time::now().toSec() > availableTime){ //Is it worth treating now or not? based on available time (about to send?)
					flagExitSearch = false;
					
					cout << "Hora de recuperar a info de buffer pendente" << endl;

					for(int i=0;(i<nOfAgents)&&(!flagExitSearch);i++){
						tambuf = rcvArrayBuffer(i); //Amount of stored packages for a determined agent
						if(tambuf>0){
							if(rcvArray(i) == _EMPTY){
								cout << "Recuperando do agente " << i << " no idx " << tambuf -1 << endl;
								bfTemp[i] = bfTempPending[i][tambuf-1]; //tambuf-1 corresponds to the index of where that last information was stored
								rcvArray(i)= _RECEIVED;
								rcvArrayBuffer(i)--;
								flagExitSearch = true;
								cout << "Info de buffer pendente para o agente " << i << " esvaziada." << endl;
							}
						}
					}
					if(flagExitSearch == false){
						cout << "Buffer principal ainda esperando ser esvaziado para adicionar qualquer coisa" << endl;
					}
					// }
				}
				cout << "Buffer Chegada = " << rcvArray.transpose() << endl;
				cout << "Buffer Pendente = " << rcvArrayBuffer.transpose() << endl;
			}
		}
		
		// if(rcvArray.minCoeff() == _DONE){ //DEBUG....................... _DONE
		//if(rcvArray.minCoeff() == _ESTIMATED){
		
		// // Checks if it is ready to go timewise and based on starting condition
		// if(flagTickStart == true){ //It means that tGlobalSendCont SHOULD have a meaningful computing value
		// 	if(ros::Time::now().toSec() >= tGlobalSendCont - updateRate*coeffUpdRate){ //Computation time,send, receiving and implementing = updateRate*0.1 = user definedlegalgeasdasdasdasdadadasdasdasdadasdadadadadadadadadad
		// 		if(flagSentToken == false){
		// 			setFlagReadyToSend(true);
		// 		} else {
		// 			setFlagReadyToSend(false);
		// 		}
		// 	} 
		// 	else {
		// 		if(ros::Time::now().toSec() > tGlobalSendCont - updateRate){
		// 			setFlagReadyToSend(false);
		// 			flagSentToken = false;
		// 		}
		// 	}
		// }
		//}		
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: PresentDebug
	*	  Created by: jrsbenevides
	*  Last Modified: 
	*
	*  	 Description: Simple text debug
	*/
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Estimator::PresentDebug(void){
		for(int i=0;i<nOfAgents;i++){
			cout << "Agente " << i << " tam. Buffer = " << bfStruct[i][0][0].index << endl;
			for(int j=0;j<bfSize;j++){
				cout << "Agente " << i << ": " << "ts" << j << " = " << bfStruct[i][j][0].tsSensor << endl;
				cout << "x : " << bfStruct[i][j][0].data[4] << endl;
				cout << "y : " << bfStruct[i][j][0].data[5] << endl;
				cout << "z : " << bfStruct[i][j][0].data[6] << endl;
			}
			cout << " " << endl;
		}

		cout << "Mensagens não recebidas por estar ocupado: " << endl;

		cout << rcvArrayBuffer.transpose() << endl;

	}

	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	/* ##########################################             CALLBACKS                 ##########################################*/
	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: viconRcvCallback
	*	  Created by: jrsbenevides
	*  Last Modified: 
	*
	*  	 Description: Receives raw odometry data from multi-agent system (indexed).
	*    Remarks:     The stoi() function is implemented on C++11 compliant compilers only. Therefore we used atoi()
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

	void Estimator::viconRcvCallback(const geometry_msgs::TransformStamped::ConstPtr& viconRaw){
		
		static int countVicon = 0;
		static double timeOld,timeOldDebug;
		static bool flagFirstVicon = true;

		double yawOdom,yawThisFrame,timeNowStamp = ros::Time::now().toSec();
		VectorQuat poseRcv,orientation,velocity;
		Vector3axes rpy,positionNow;
		int agent;

		if(flagFirstVicon){
			flagFirstVicon = false;
			timeOld = timeNowStamp;
		}

		positionNow		<< 	viconRaw->transform.translation.x, 
							viconRaw->transform.translation.y, 
							viconRaw->transform.translation.z;
					
		orientation 	<< 	viconRaw->transform.rotation.w, 
							viconRaw->transform.rotation.x, 
							viconRaw->transform.rotation.y, 
							viconRaw->transform.rotation.z;


		agent = 0; //REFAZER A LOGICA PARA REALMENTE LER O AGENTE EM QUESTAO

		/*Reset frame location for this agent*/
		if (!getIsOdomStarted(agent)) {   
			cout << "######## Zeroing virtual coordinate frame" << endl;
			Conversion::quat2angleZYX(rpy,orientation);
			yawOdom = angles::normalize_angle(rpy(2));				//Added normalize...check if it works!!!
			poseRcv << positionNow,yawOdom;
			setPoseZero(poseRcv,agent);
			setIsOdomStarted(true,agent);
		}

		//Based on initial pose, calculate transformed pose			
		setPosition(positionNow,agent); //positionNow will be changed
		//yawThisFrame is gonna be the current yaw value given the zero
		setOrientationVicon(orientation,yawThisFrame,agent);

		poseRcv << 	positionNow,
					yawThisFrame;

		velocity = DvKalman(poseRcv,timeNowStamp,timeOld);

		setCurrentYaw(yawThisFrame,agent);

		timeOld = timeNowStamp; //Updates last stamped time with current one. Note that timeOld is a static variable

		if(countVicon % 5 == 0){
			
			countVicon = 1;
			double randNumber;

			countRcvMsg++;
			cout << "Recebi a mensagem " << countRcvMsg << "em " << getTimeShifted(timeNowStamp) <<endl;

			randNumber = ((double) rand() / (RAND_MAX));

			if((randNumber>dropProbability)||(lossCount>=nLossMax)){ 
				lossCount =0;
				//cout<< "Drop Prob: " << dropProbability << endl;
				//cout<< "rand() " << randNumber << endl;
				//cout<< "#################good" << endl;
				if(getIsFlagEnable()){
					string agent;
					int nAgent;
					Buffer incomingMsg;

					if(isCMHEenabled == 1){

						// Initialize buffer during first loop
						if(flagEnter){
							tGlobalSendCont  = 0;
							timeOldDebug = timeNowStamp; //DEBUG
							flagEnter		= false;
						}

						//Fill Header
						if(getFlagVicon()){
							nAgent =  0;
							incomingMsg.index = 0; 
							// cout << "ESTOU USANDO A VICON" << endl;
						} else{
							agent = viconRaw->header.frame_id; 
							nAgent =  atoi(agent.c_str());   
							incomingMsg.index = nAgent;
						}

						//DEBUG
						// cout << "\n-----------------------------------" << endl;
						// cout << "Msg: Ag " <<  nAgent << ": RECEBIDO" << endl;

						incomingMsg.tsArrival = timeNowStamp;
						incomingMsg.tsSensor  = viconRaw->header.stamp.toSec(); 
						// cout << "tempoArrival = " << incomingMsg.tsArrival << endl;
						// cout << "tempoSensor = " << incomingMsg.tsSensor << endl;
						// cout << "Tempo Decorrido: " << incomingMsg.tsArrival - incomingMsg.tsSensor << endl; //DEBUG
						incomingMsg.tGSendCont = 0;
						//Fill Data

						cout << "Received position: " << positionNow << endl;						
						
						incomingMsg.data << velocity,
											positionNow,
											yawThisFrame;

						cout << "Transformed Received Message: " << incomingMsg.data << endl;

						if(rcvArray(nAgent) == _EMPTY){
							
							// cout << "Status: RECEBIDO Buffer Principal" << endl; //DEBUG
							setBuffer(incomingMsg); //Save on main receive buffer
							rcvArray(nAgent) = _RECEIVED;

						} else {
							if(rcvArrayBuffer(nAgent) < _BUFMAXSIZE){
								// cout << "Status: RECEBIDO Buffer Pendente : Ag = " << nAgent << endl;
								setBufferNext(incomingMsg); 	//Save on pending receive buffer
								rcvArrayBuffer(nAgent)++; //This call HAS TO come after the set above
								// cout << "Pendente: " << rcvArrayBuffer.transpose() << endl;
							}
						}	

						cout << "Vicon Real UpdateRate: " << timeNowStamp - timeOldDebug << endl;

						// if(nAgent == 0){
						// 	cout << "Msg Recebida:" << odomRaw->pose.pose.position.x << " " << odomRaw->pose.pose.position.y << " " << odomRaw->pose.pose.position.z << endl;
						// }
						timeOldDebug = timeNowStamp; //DEBUG
					}
				}
			} else{
				// cout<< "Dropout at time:" << timeNowStamp << endl;
				cout << "Dropout " << countRcvMsg << " em " << getTimeShifted(timeNowStamp) <<endl;
				lossCount++;
			}
		} else{
			countVicon++;
		}
	}


	// void Estimator::viconRcvCallback(const geometry_msgs::TransformStamped::ConstPtr& viconRaw){
		
	// 	static int countVicon = 0;
	// 	double timeNowStamp = ros::Time::now().toSec();

	// 	if(countVicon % 5 == 0){
			
	// 		countVicon = 1;
	// 		double randNumber;

	// 		countRcvMsg++;
	// 		cout << "Recebi a mensagem " << countRcvMsg << "em " << getTimeShifted(timeNowStamp) <<endl;

	// 		randNumber = ((double) rand() / (RAND_MAX));

	// 		if((randNumber>dropProbability)||(lossCount>=nLossMax)){ 
	// 			lossCount =0;
	// 			//cout<< "Drop Prob: " << dropProbability << endl;
	// 			//cout<< "rand() " << randNumber << endl;
	// 			//cout<< "#################good" << endl;
	// 			if(getIsFlagEnable()){
	// 				string agent;
	// 				int nAgent;
	// 				Buffer incomingMsg;
	// 				VectorQuat poseRcv,orientation,velocity;
	// 				Vector3axes rpy,positionNow;
	// 				double yawOdom,yawThisFrame;

	// 				if(isCMHEenabled == 1){

	// 					// Initialize buffer during first loop
	// 					if(flagEnter){
	// 						tGlobalSendCont  = 0;
	// 						// for (int i = 0;i < nOfAgents ;i++){
	// 						// 	for (int j = 0;j < bfSize ;j++){
	// 						// 		for (int k = 0;k < 2 ;k++){
	// 						// 				cout << "inicio = " << bfStruct[i][j][k].index << endl;
	// 						// 		}
	// 						// 	}
	// 						// }
	// 						// counter++;
	// 						flagEnter		= false;
	// 					}

	// 					//Fill Header
	// 					if(getFlagVicon()){
	// 						nAgent =  0;
	// 						incomingMsg.index = 0; 
	// 						// cout << "ESTOU USANDO A VICON" << endl;
	// 					} else{
	// 						agent = viconRaw->header.frame_id; 
	// 						nAgent =  atoi(agent.c_str());   
	// 						incomingMsg.index = nAgent;
	// 					}

	// 					//DEBUG
	// 					// cout << "\n-----------------------------------" << endl;
	// 					// cout << "Msg: Ag " <<  nAgent << ": RECEBIDO" << endl;

	// 					incomingMsg.tsArrival = timeNowStamp;
	// 					incomingMsg.tsSensor  = viconRaw->header.stamp.toSec(); 
	// 					// cout << "tempoArrival = " << incomingMsg.tsArrival << endl;
	// 					// cout << "tempoSensor = " << incomingMsg.tsSensor << endl;
	// 					// cout << "Tempo Decorrido: " << incomingMsg.tsArrival - incomingMsg.tsSensor << endl; //DEBUG
	// 					incomingMsg.tGSendCont = 0;
	// 					//Fill Data

	// 					positionNow		<< 	viconRaw->transform.translation.x, 
	// 										viconRaw->transform.translation.y, 
	// 										viconRaw->transform.translation.z;
											
	// 					orientation 	<< 	viconRaw->transform.rotation.w, 
	// 										viconRaw->transform.rotation.x, 
	// 										viconRaw->transform.rotation.y, 
	// 										viconRaw->transform.rotation.z;


	// 					velocity		<< 0,0,0,0; //This will be replaced for real data after computation in dvKalman in the buffer section.

	// 					cout << "Received position: " << positionNow << endl;

	// 					/*Reset frame location for this agent*/
	// 					if (!getIsOdomStarted(nAgent)) {
	// 						cout << "######## Zeroing virtual coordinate frame" << endl;
	// 						Conversion::quat2angleZYX(rpy,orientation);
	// 						yawOdom = angles::normalize_angle(rpy(2));				//Added normalize...check if it works!!!
	// 						poseRcv << positionNow,yawOdom;
	// 						setPoseZero(poseRcv,nAgent);
	// 						setIsOdomStarted(true,nAgent);
	// 					}

	// 					//Based on initial pose, calculate transformed pose			
	// 					setPosition(positionNow,nAgent); //positionNow will be changed
	// 					// yawThisFrame = setOrientation(orientation,nAgent);
	// 					//Velocity is gonna be transformed from global to global
	// 					//yawThisFrame is gonna be the current yaw value given the zero
	// 					setOrientationVicon(orientation,yawThisFrame,nAgent);


	// 					// //To get velocity...now we will search in the current buffer
	// 					// setPosition(positionNow);
	// 					// positionNow		= drone.getPosition();
	// 					// linearVel 		= drone.DvKalman(positionNow,timeNow,timePast); //global terms instead of local (IMU)
						
	// 					// setOrientation(orientationVicon);
	// 					// orientationNow	= drone.getOrientation();
	// 					// angularVel 		= drone.DwKalman(orientationNow,timeNow,timePast);									

	// 					//When it comes from odometry, velocity is local, thus we need to transform it: dvg = Rot*dvb	
						
						
	// 					incomingMsg.data << velocity,
	// 										positionNow,
	// 										yawThisFrame;

	// 					cout << "Transformed Received Message: " << incomingMsg.data << endl;

	// 					if(rcvArray(nAgent) == _EMPTY){
							
	// 						// cout << "Status: RECEBIDO Buffer Principal" << endl; //DEBUG
	// 						setBuffer(incomingMsg); //Save on main receive buffer
	// 						rcvArray(nAgent) = _RECEIVED;

	// 					} else {
	// 						if(rcvArrayBuffer(nAgent) < _BUFMAXSIZE){
	// 							// cout << "Status: RECEBIDO Buffer Pendente : Ag = " << nAgent << endl;
	// 							setBufferNext(incomingMsg); 	//Save on pending receive buffer
	// 							rcvArrayBuffer(nAgent)++; //This call HAS TO come after the set above
	// 							// cout << "Pendente: " << rcvArrayBuffer.transpose() << endl;
	// 						}
	// 					}	

	// 					// if(nAgent == 0){
	// 					// 	cout << "Msg Recebida:" << odomRaw->pose.pose.position.x << " " << odomRaw->pose.pose.position.y << " " << odomRaw->pose.pose.position.z << endl;
	// 					// }

	// 				}
	// 			}
	// 		} else{
	// 			cout<< "################# DROPOUT!!!! " << endl;
	// 			lossCount++;
	// 		}
	// 	} else{
	// 		countVicon++;
	// 	}
	// }


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: odomRcvCallback
	*	  Created by: jrsbenevides
	*  Last Modified: 
	*
	*  	 Description: Receives raw odometry data from multi-agent system (indexed).
	*    Remarks:     The stoi() function is implemented on C++11 compliant compilers only. Therefore we used atoi()
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

	void Estimator::odomRcvCallback(const nav_msgs::Odometry::ConstPtr& odomRaw){		

		if(getIsFlagEnable()){
			string agent;
			int nAgent;
			Buffer incomingMsg;
			VectorQuat poseRcv,orientation,velocity;
			Vector3axes rpy,positionNow;
			double yawOdom,yawThisFrame;

			if(isCMHEenabled == 1){

				// Initialize buffer during first loop
				if(flagEnter){
					tGlobalSendCont  = 0;
					// for (int i = 0;i < nOfAgents ;i++){
					// 	for (int j = 0;j < bfSize ;j++){
					// 		for (int k = 0;k < 2 ;k++){
					// 				cout << "inicio = " << bfStruct[i][j][k].index << endl;
					// 		}
					// 	}
					// }
					// counter++;
					flagEnter		= false;
				}

				//Fill Header
				agent = odomRaw->header.frame_id;
				nAgent =  atoi(agent.c_str());   
				incomingMsg.index = nAgent;

				//DEBUG
				// cout << "\n-----------------------------------" << endl;
				cout << "Msg: Ag " <<  nAgent << ": RECEBIDO" << endl;

				incomingMsg.tsArrival = ros::Time::now().toSec();
				incomingMsg.tsSensor  = odomRaw->header.stamp.toSec();
				// cout << "tempoArrival = " << incomingMsg.tsArrival << endl;
				// cout << "tempoSensor = " << incomingMsg.tsSensor << endl;

				incomingMsg.tGSendCont = 0;
				//Fill Data

				orientation 	<< 	odomRaw->pose.pose.orientation.w, 
									odomRaw->pose.pose.orientation.x, 
									odomRaw->pose.pose.orientation.y, 
									odomRaw->pose.pose.orientation.z;	

				velocity 		<< 	odomRaw->twist.twist.linear.x,
									odomRaw->twist.twist.linear.y,
									odomRaw->twist.twist.linear.z,
									odomRaw->twist.twist.angular.z;													

				positionNow 	<< 	odomRaw->pose.pose.position.x,
									odomRaw->pose.pose.position.y,
									odomRaw->pose.pose.position.z;

				//When it comes from odometry, velocity is local, thus we need to transform it: dvg = Rot*dvb									

				/*Reset frame location*/
				if (!getIsOdomStarted(nAgent)) {
					Conversion::quat2angleZYX(rpy,orientation);
					yawOdom = angles::normalize_angle(rpy(2));				//Added normalize...check if it works!!!
					poseRcv << positionNow,yawOdom;
					setPoseZero(poseRcv,nAgent);
					setIsOdomStarted(true,nAgent);
				}

				//Based on initial pose, calculate transformed pose			
				setPosition(positionNow,nAgent);
				// yawThisFrame = setOrientation(orientation,nAgent);
				//Velocity is gonna be transformed from global to global
				//yawThisFrame is gonna be the current yaw value given the zero
				setOrientation(orientation,yawThisFrame,nAgent,velocity);
				
				
				incomingMsg.data << velocity,
									positionNow,
									yawThisFrame;

				if(rcvArray(nAgent) == _EMPTY){
					
					// cout << "Status: RECEBIDO Buffer Principal" << endl; //DEBUG
					setBuffer(incomingMsg); //Save on main receive buffer
					rcvArray(nAgent) = _RECEIVED;

				} else {
					if(rcvArrayBuffer(nAgent) < _BUFMAXSIZE){
						// cout << "Status: RECEBIDO Buffer Pendente : Ag = " << nAgent << endl;
						setBufferNext(incomingMsg); 	//Save on pending receive buffer
						rcvArrayBuffer(nAgent)++; //This call HAS TO come after the set above
						// cout << "Pendente: " << rcvArrayBuffer.transpose() << endl;
					}
				}	

				// if(nAgent == 0){
				// 	cout << "Msg Recebida:" << odomRaw->pose.pose.position.x << " " << odomRaw->pose.pose.position.y << " " << odomRaw->pose.pose.position.z << endl;
				// }

			}
		}
	}		
}


// int main(int argc, char **argv)
// {
 
// 	ros::init(argc, argv, "networkEstimator");
	
//  	try {

// 		DRONE::Estimator ncs_estimator;

// 		ros::Rate loop_rate(100);

// 		while (ros::ok())
// 	   	{
// 		    ncs_estimator.ComputeEstimation();
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


