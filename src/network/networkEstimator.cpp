/*
 * genTrajectory.cpp
 *
 *  Created on: Dec 28, 2020
 *      Author: João Benevides
 *      Modified:
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
		if(index<3){
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

	void Estimator::setCmdAgentDone(const int& agent){
		rcvArray(agent) = _DONE;
	}

	void Estimator::setToken(const bool& flag){
		flagSentToken = flag;
	}
	
	

	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	/* ########################################                 GETTERS                 ##########################################*/
	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/

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
		if(timeNow > nextTimeToSend){ //We passed already
			if(nextTimeToSend > 0){						//After every iteration
				nextTimeToSend += updateRate;
			} else if(nextTimeToSend == 0) { 			//Only on first iteration.
				nextTimeToSend = timeNow + updateRate;
			}
		}else{ //if we still didn't reach the next timeToSend
			if(flagSentToken == true){ //but if we already sent the message... 
				nextTimeToSend += updateRate; //...we should be working with timeToSend after this one
				setToken(false);  //flag of sent message disabled
			} 
		}
		if(nextTimeToSend>0){
			flagTickStart = true;
		}
		return nextTimeToSend;
	}

	Vector8d Estimator::getEstimatePose(const int agent){
		return estPose[agent];
	}
	
	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	/* #####################################            REGULAR FUNCTIONS                 ########################################*/
	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	
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


		Buffer msgDrone,rcvMsg;
		geometry_msgs::Pose p;

		rcvArray = rcvArray.Zero();
		rcvArrayBuffer = rcvArrayBuffer.Zero();

		cout << "Starting Estimator Node ...\n" << endl;

		
		counter = 0; //para testes apenas

		F = F.Identity();
		Q = 1*Q.Identity();			 
		R = 0.1*R.Identity();
		for(int i = 0;i<nOfAgents;i++){
			P[i] = MatrixXd::Identity(2,2);
		}

		_EMPTY 		= 0;
		_RECEIVED 	= 1;
		_ESTIMATED	= 7;
		_DONE 		= 15;


		flagEnter			= true;
		flagDebug 			= true;
		flagTickStart 		= false;
		flagEmergencyStop 	= false;
		setFlagReadyToSend(false);
		setFlagComputeControl(true);
		setToken(false);
		updateRate          = 0.1; //10Hz
		isCMHEenabled		= 0;
		nOfAgents			= 5;
		bfSize              = 5;
		PI 					= 3.141592653589793;
   		t 			  		= 0.0;
		stepT				= 1; //number of steps in integration
		nextTimeToSend      = -1;
		K 					<<  1.74199, 0.94016, 1.54413, 0.89628, 3.34885, 3.29467, 6.51209, 3.92187;
		Rotation 			= Rotation.Identity();

		loadTopics(n);
		loadSettings(n);

		//Update Model
		updateModel();
		
		// Initialize a void message in the Buffer;
		msgDrone.index 		= 0;
		msgDrone.tsSensor   = 0;
		msgDrone.tsArrival  = 0;
		msgDrone.tGSendCont = 0;
		msgDrone.upost      << 0,0,0,0;
		msgDrone.upre      	<< 0,0,0,0;
		msgDrone.data 		<< 0,0,0,0,0,0,0,0;

		// setBuffer(msgDrone);

		// rcvMsg = getBuffer(2);

		// estPose.header.stamp = ros::Time::now(); // timestamp of creation of the msg
		// estPose.header.frame_id = "map"; // frame id in which the array is published

		// for (int k = 0;k < nOfAgents ;k++){
		// 	p.position.x = 0.1 + (double) k;
		// 	// p.position = 0.1, 0.2, 0.3;
		// 	estPose.poses.push_back(p);
		// 	cout << "teste1 = " << estPose.poses[k].position.x << endl;
		// }
		
		for (int i = 0;i < nOfAgents ;i++){
			for (int j = 0;j < bfSize ;j++){
				for (int k = 0;k < 2 ;k++){
						bfStruct[i][j][k] = msgDrone;
				}
			}
		}
		
		for (int i = 0;i < bfSize ;i++){
			bfTemp[i] = msgDrone;
			for (int j = 0;j < 3 ;j++){
				bfTempPending[i][j] = msgDrone;
			}
		}

		cout << "Index = " << rcvMsg.index <<  endl;
		cout << "Tempo = " << rcvMsg.tsSensor <<  endl;
		cout << "SOMA = " << rcvArray.sum() << endl;

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

		joy_subscriber 	   	  = n.subscribe<sensor_msgs::Joy>("/drone/joy", 1, &Estimator::joyCallback, this);
		
		odomGlobal_publisher  = n.advertise<nav_msgs::Odometry>("/drone/odom_global", 1);
		odomRcv_subscriber    = n.subscribe<nav_msgs::Odometry>("odom_imu", 3, &Estimator::odomRcvCallback, this);
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

	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: updateModel
	*	  Created by: jrsbenevides
	*  Last Modified: jrsbenevides
	*
	*  	 Description: 1. Update Robot Model Based on the current orientation (TBD) - FALTA LER 
	*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Estimator::updateModel(void){

		Matrix4d Ac,Bc;

		double sinyaw,cosyaw,yaw;

		yaw = 0;

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
		A << Ac*Rotation.transpose(), MatrixXd::Zero(4,4),
			 MatrixXd::Identity(4,4), MatrixXd::Zero(4,4);
		B << Bc,
			 MatrixXd::Zero(4,4);
		
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
				cout << "eita" << endl;
				i--;
			}
		}

		if(terminalCondition == true){
			cout << "Sucesso >> ";
			if(pkt.tsSensor == bfStruct[agent][i][0].tsSensor)
				cout << "total" << endl;
			else
				cout << "parcial" << endl;

		}

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

	void Estimator::UpdateBuffer(const Buffer& pkt, const int agent,const int i){
		
		int curIndex;

		// Copies this row and saves it.
		for(int k=0; k < bfSize ; k++){
			bfStruct[agent][k][1] = bfStruct[agent][k][0];
		}

		// Stores current buffer index in order to increment at the end.
		curIndex = bfStruct[agent][0][0].index;

		// cout << "O agente eh " << agent << " e o pacote eh " << curIndex << " e i = " << i << endl; //DEBUG!!!

		// Shifts old packages to accomodate new package
		if(i > 0){
			for(int k = 0;k < i-1; k++){
				bfStruct[agent][k][0] = bfStruct[agent][k+1][0];
			}
		}
		
		// Stores new package
		bfStruct[agent][i][0] = pkt;
		
		// Updates buffer index
		bfStruct[agent][0][0].index = curIndex + 1;
		// cout << "Agora o agente eh " << agent << " e o pacote eh " << bfStruct[agent][0][0].index << endl; //DEBUG!!!

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
		if(isCMHEenabled == 1){
		  /*Enables "Automatic Control Mode" while pressing this button*/
		  if(joy->buttons[6]){
		    cout << "on" << endl;
		  }
		  else{
			cout << "off" << endl;
		  }
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: updateEKF
	*	  Created by: jrsbenevides
	*  Last Modified: 
	*
	*  	 Description: Performs the prediction and update steps on the EKF estimator
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
			isSinsideTrapezoid(s,sOld,agent);
			P[agent] = (MatrixXd::Identity(2,2) - K*Hk)*P[agent];
		}
		estParam.block<2,1>(0,agent) << s; 
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/* 		Function: isSinsideTrapezoid
	*	  Created by: jrsbenevides
	*  Last Modified: 
	*
	*  	 Description: Checks if the computed parameters lie inside the boundaries
	*/		   
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		

	void Estimator::isSinsideTrapezoid(Vector2d& s, const Vector2d& sOld, int agent){

		double alpha, beta;
		// Vector2d sNew;

		alpha = s(0);
		beta  = s(1);

		// sNew = sOld;

		if((alpha >= 1 - genParam[agent].sigmat) && (alpha <= 1 + genParam[agent].sigmat)){ // 		%Alpha lies inside boundaries for alpha
			if((beta >= -alpha*genParam[agent].t1) && (beta <= genParam[agent].tn - alpha*genParam[agent].tnbar)){ //Beta lies inside boundaries for beta
				s = s;
			} else {
				s = sOld;
			}
		} else {
			s = sOld;
		}
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
		double tBar, deltaT, tGlobalSendCont,tempValue;
		
		Vector2d sEst;
		VectorQuat uComp, uPre;
		Vector8d x;
		
		Matrix8d Ak;
		Matrix8x4 Bk;

		tGlobalSendCont = getThisTimeSend(); // TROCAR ESSE VALOR POR ALGO REAL! FAZER AS CONTAS CERTINHO!!!

		if(rcvArray.maxCoeff() > _EMPTY){

			agent = nextAgentToCompute(); // Checks which agent should be handled next. -1 if waiting for computation. 
			
			if(agent>=0){
				
				bfTemp[agent].tGSendCont = tGlobalSendCont; //APRIMORAR AS CONDIÇÕES PARA ESSA DETERMINAÇÃO (msg adiantada, atrasada, etc...)
				
				cout  << "Tratando: Agente: " << agent << " e pkt: " << bfStruct[agent][0][0].index << endl; //DEBUG!!! 

				status = AddPkt2Buffer(bfTemp[agent],agent);
			
				if(status == true){ // 		If buffer received this new info:
					cout  << "Sucesso: Agente: " << agent << " e novo tamanho preenchido: " << bfStruct[agent][0][0].index << endl; //DEBUG!!! 
					if(bfStruct[agent][0][0].index >= bfSize){ //Buffer is ready to estimate
						if(bfStruct[agent][0][0].index == bfSize){ //%Initial guess for estimate
							
							estParam.block<2,1>(0,agent) << 1,
															0.5*(bfStruct[agent][bfSize-1][0].tsArrival-bfStruct[agent][bfSize-1][0].tsSensor-bfStruct[agent][0][0].tsSensor);
							genParam[agent].t1     = bfStruct[agent][0][0].tsSensor;
							genParam[agent].tn     = bfStruct[agent][bfSize-1][0].tsArrival;
							genParam[agent].tnbar  = bfStruct[agent][bfSize-1][0].tsSensor;
							genParam[agent].sigmat = 0.5;									
							cout <<  "Built first estimate for agent " << agent <<  endl;

							PresentDebug(); //DEBUG
						}
						if(flagEmergencyStop == false) //DEBUG
							updateEKF(agent);
					} else {
						bfStruct[agent][bfSize-1][0].upre << 0.01, 0.01, 0.01, 0.01; //CORRIGIR PARA O COMANDO DE ENTRADA CORRETO NO LUGAR DE 0.01!!!
					}
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
							
				if(bfStruct[agent][0][0].index > bfSize){ //Talvez aumentar o limiar para um valor maior que bfSize apresente um resultado melhor...quando estabilizar.
					
					sEst << estParam.block<2,1>(0,agent);
					
					tBar = (1/sEst(0))*tGlobalSendCont - (sEst(1)/sEst(0));
					deltaT = (tBar - bfStruct[agent][bfSize-1][0].tsSensor)/stepT;
					Ak   = sEst(0)*A;
					Bk   = sEst(0)*B;
					x    = bfStruct[agent][bfSize-1][0].data;
					uPre = bfStruct[agent][bfSize-1][0].upre;

					for(int j = 0; j<stepT; j++){
						x = (MatrixXd::Identity(8,8)+deltaT*Ak)*x + deltaT*Bk*uPre;
						// x = (eye(dataSize)+deltaT*Ak)*x + deltaT*Bk*uComp;
					}
				} else{
					x     = bfStruct[agent][bfSize-1][0].data;
					uComp = bfStruct[agent][bfSize-1][0].upre;
				}	

				// Make it available for the controller
				setEstimatePose(x,agent);
							
			} else{
		
				if(flagDebug){
					//AGUARDANDO CHEGADA DE MENSAGEM OU FINALIZACAO DE CALCULO.
					// cout << "Buffer temporario já está vazio ou aguardando liberacao" << endl;

					if(rcvArrayBuffer.minCoeff() > 0){ // There is something inside pending buffer!!
						// if(tGlobalSendCont - ros::Time::now().toSec() > availableTime){ //Is it worth treating now or not? based on available time (about to send?)
						flagExitSearch = false;
						
						for(int i=0;(i<nOfAgents)&&(!flagExitSearch);i++){
							tambuf = rcvArrayBuffer(i);
							if(tambuf>0){
								if(rcvArray(i) == _EMPTY){
									bfTemp[i] = bfTempPending[i][tambuf];
									rcvArray(i)= _RECEIVED;
									rcvArrayBuffer(i)--;
									flagExitSearch = true;
									cout << "Info de buffer pendente para o agente " << i << " esvaziada." << endl;
								}
							}
						}
						// }
					}
					cout << "Buffer chegada = " << rcvArray.transpose() << endl;
					flagDebug = true;
				}
			}
		}
		
		
		
		// if(rcvArray.minCoeff() == _DONE){ //DEBUG....................... _DONE
		//if(rcvArray.minCoeff() == _ESTIMATED){
		
		// Checks if it is ready to go timewise and based on starting condition
		if(flagTickStart == true){ //It means that tGlobalSendCont SHOULD have a meaningful computing value
			if(tGlobalSendCont - ros::Time::now().toSec() < updateRate*0.05){ //Computation time,send, receiving and implementing = updateRate*0.1 = user definedlegalgeasdasdasdasdadadasdasdasdadasdadadadadadadadadad
				if(flagSentToken == false){
					setFlagReadyToSend(true);
				} else {
					setFlagReadyToSend(false);
				}
				
			} 
			else {
				if(tGlobalSendCont - ros::Time::now().toSec() < updateRate){
					setFlagReadyToSend(false);
					flagSentToken = false;
				}
			}
		}
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
			}
			cout << " " << endl;
		}

		cout << "Mensagens não recebidas por estar ocupado: " << endl;

		cout << rcvArrayBuffer.transpose() << endl;

		flagEmergencyStop = true;

	}

	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/
	/* ##########################################             CALLBACKS                 ##########################################*/
	/* ###########################################################################################################################*/
	/* ###########################################################################################################################*/

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
		string agent;
		int nAgent;
		Buffer incomingMsg;
		if(isCMHEenabled == 1){

			// Initialize buffer during first loop
			if(flagEnter){
				nextTimeToSend  = 0;
				// for (int i = 0;i < nOfAgents ;i++){
				// 	for (int j = 0;j < bfSize ;j++){
				// 		for (int k = 0;k < 2 ;k++){
				// 				cout << "inicio = " << bfStruct[i][j][k].index << endl;
				// 		}
				// 	}
				// }
				flagEnter		= false;
			}

			//Fill Header
			agent = odomRaw->header.frame_id;
			nAgent =  atoi(agent.c_str());   
			incomingMsg.index = nAgent;

			//DEBUG
			cout << "Msg: Ag " <<  nAgent << ": RECEBIDO" << endl;

			if(rcvArray(nAgent) == _EMPTY){
				rcvArray(nAgent) = _RECEIVED;

				//DEBUG
				cout << "Status: RECEBIDO " << endl;
				
				incomingMsg.tsArrival = ros::Time::now().toSec();
				incomingMsg.tsSensor  = odomRaw->header.stamp.toSec();
				incomingMsg.tGSendCont = 0;
				//Fill Data
				incomingMsg.data << odomRaw->pose.pose.position.x,
									odomRaw->pose.pose.position.y,
									odomRaw->pose.pose.position.z,
									odomRaw->pose.pose.orientation.z, //CORRIGIR ISSO AQUI PARA A CONVERSÃO DE QUATERNIO
									odomRaw->twist.twist.linear.x,
									odomRaw->twist.twist.linear.y,
									odomRaw->twist.twist.linear.z,
									odomRaw->twist.twist.angular.z;
				
				//Save

				setBuffer(incomingMsg);
			} else {
				if(rcvArrayBuffer(nAgent) < 3){
					cout << "Add info de buffer pendente para o agente " << nAgent << endl;
					setBufferNext(incomingMsg);
					rcvArrayBuffer(nAgent)++; //This call HAS TO come after the set above
					cout << "Pendente: " << rcvArrayBuffer.transpose() << endl;
				}
				// cout << "####### acho que aqui mora o problema" << endl;
				// if(rcvArrayBuffer(nAgent) < 1000){
				// 	rcvArrayBuffer(nAgent)++;
				// } else {
				// 	rcvArrayBuffer(nAgent) = 1000;
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


