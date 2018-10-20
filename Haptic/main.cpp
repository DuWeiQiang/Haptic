//==============================================================================
/*
Software License Agreement (BSD License)
Copyright (c) 2003-2016, CHAI3D.
(www.chai3d.org)

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above
copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided
with the distribution.

* Neither the name of CHAI3D nor the names of its contributors may
be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

\author    <http://www.chai3d.org>
\author    Francois Conti
\version   3.2.0 $Rev: 1869 $
*/
//==============================================================================
#include "commTool.cpp"
#include "config.h"
#include "HapticCommLib.h"
//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
using namespace chai3d;
//using namespace std; //std namespace contains bind function which conflicts with socket bind function
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
C_STEREO_DISABLED:            Stereo is disabled
C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// a flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

WORD sockVersion;
WSADATA data;
Sender<hapticMessageM2S> *sender;
Receiver<hapticMessageS2M> *receiver;
threadsafe_queue<hapticMessageM2S> *commandQ;
threadsafe_queue<hapticMessageS2M> *forceQ;
SOCKET sclient;
LARGE_INTEGER cpuFreq;

//------------------------------------------------------------------------------
// Read Parameters from configuration file
//------------------------------------------------------------------------------
ConfigFile cfg("cfg/config.cfg"); // get the configuration file
double ForceDeadbandParameter = cfg.getValueOfKey<double>("ForceDeadbandParameter"); //deadband parameter for force data reduction, 0.1 is the default value
double VelocityDeadbandParameter = cfg.getValueOfKey<double>("VelocityDeadbandParameter"); //deadband parameter for velcity data reduction, 0.1 is the default value
double PositionDeadbandParameter = cfg.getValueOfKey<double>("PositionDeadbandParameter"); //deadband parameter for position data reduction, 0.1 is the default value

int ForceDelay = cfg.getValueOfKey<int>("ForceDelay"); // ms: constant network delay on Force feedback
int CommandDelay = cfg.getValueOfKey<int>("CommandDelay"); // ms: constant network delay on Commanding channel 
														   //int RecordSwitch = cfg.getValueOfKey<int>("RecordSignals"); // 0: Turn off recording, 1: Turn on recording
int RecordSwitch = 1;
int ControlMode = cfg.getValueOfKey<int>("ControlMode"); // 0: position control, 1:velocity control
int FlagVelocityKalmanFilter = cfg.getValueOfKey<int>("FlagVelocityKalmanFilter"); // 0: Kalman filter disabled 1: Kalman filter enabled on velocity signal

DeadbandDataReduction* DBForce; // data reduction class for force samples
DeadbandDataReduction* DBVelocity; // data reduction class for velocity samples
DeadbandDataReduction* DBPosition; // data reduction class for position samples

bool ForceTransmitFlag = false; // true: deadband triger false: keep last recently transmitted sample (ZoH)
bool VelocityTransmitFlag = false; // true: deadband triger false: keep last recently transmitted sample (ZoH)
bool PositionTransmitFlag = false; // true: deadband triger false: keep last recently transmitted sample (ZoH)

KalmanFilter VelocityKalmanFilter; // applies 3 DoF kalman filtering to remove noise from velocity signal																				   

int ForcePacketNum = 0;
int VelocityPacketNum = 0;
int PositionPacketNum = 0;
//------------------------------------------------------------------------------
// TDPA variable and function realted code
//------------------------------------------------------------------------------
//----------TDPA---------------------

void ComputeEnergy(double &Ein, double &Eout, double vel[3], double force[3]);
void initEnergy();
double sample_interval = 0.001;   //1kHz

double SlaveControlForce[3] = { 0.0,0.0,0.0 };  // current 3 DoF slave control force sample
double Em_in = 0, Em_out = 0, Es_in = 0, Es_out = 0;
double Em_in_last = 0, Es_in_last = 0;   // last transmitted master/slave input energy
double E_trans_m = 0, E_trans_s = 0, E_recv_m = 0, E_recv_s = 0;   // transmitted and received input energy at the master/slave side
double alpha_m = 0, beta_s = 0;
bool TDPAon = false;
double lastMasterForce[3] = { 0.0, 0.0, 0.0 };   // use dot_f and tau to filter the master force
bool use_tauFilter = false;

double MasterForce[3] = { 0.0, 0.0, 0.0 };
double MasterVelocity[3] = { 0.0, 0.0, 0.0 }; // update 3 DoF master velocity sample (holds the signal before deadband)
double MasterPosition[3] = { 0.0, 0.0, 0.0 }; // update 3 DoF master position sample (holds the signal before deadband)

double UpdatedForceSample[3] = { 0.0,0.0,0.0 };  // updated 3 DoF force sample (holds the signal after deadband)
double UpdatedVelocitySample[3] = { 0.0,0.0,0.0 }; // update 3 DoF velocity sample (holds the signal after deadband)
double UpdatedPositionSample[3] = { 0.0, 0.0, 0.0 }; // update 3 DoF position sample (holds the signal after deadband)


KalmanFilter ForceKalmanFilter; // applies 3 DoF kalman filtering to remove noise from force signal
//-----------TDPA compute energy--------------
void ComputeEnergy(double &Ein, double &Eout, double vel[3], double force[3])
{
	// only for z direction
	double power = vel[2] * (-1 * force[2]);
	if (power >= 0)
	{
		Ein = Ein + sample_interval*power;
	}
	else
	{
		Eout = Eout - sample_interval*power;
	}
}
void initEnergy()
{
	SlaveControlForce[0] = SlaveControlForce[0] = SlaveControlForce[0] = 0.0;
	Em_in = Em_out = Es_in = Es_out = 0;
	Em_in_last = Es_in_last = 0;
	E_trans_m = E_trans_s = E_recv_m = E_recv_s = 0;
	alpha_m = beta_s = 0;
}


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);


//==============================================================================
/*
DEMO:   01-mydevice.cpp

This application illustrates how to program forces, torques and gripper
forces to your haptic device.

In this example the application opens an OpenGL window and displays a
3D cursor for the device connected to your computer. If the user presses
onto the user button (if available on your haptic device), the color of
the cursor changes from blue to green.

In the main haptics loop function  "updateHaptics()" , the position,
orientation and user switch status are read at each haptic cycle.
Force and torque vectors are computed and sent back to the haptic device.
*/
//==============================================================================

int main(int argc, char* argv[])
{
	//--------------------------------------------------------------------------
	// INITIALIZATION
	//--------------------------------------------------------------------------
	//--------------------------------------------------------------------------
	// INITIALIZATION
	//--------------------------------------------------------------------------
	printf("-----------------------------------\n");

	printf("(C) TU Munich 2017 Chair of Media Technology\n");
	printf("Author: Burak Cizmeci\n");
	printf("Contributors: Xiao Xu\n");

	printf("Haptic Communication Demo Application\n");
	printf("Demo:  Cube manipulation\n");
	printf("Copyright 2017\n");
	printf("-----------------------------------\n");
	printf("\n");
	printf("Keyboard Options:\n\n");

	printf("******* Control mode keys *********************************\n");
	printf("[p] - switch to position control\n");
	printf("[v] - switch to velocity control\n");
	printf("[k] - enable/disable Kalman filter on velocity\n");
	printf("*********************************************************\n\n");

	printf("******* Deadband increase/decrease keys *******************\n");
	printf("[q] - increase force deadband (+0.01) \n");
	printf("[a] - decrease force deadband (-0.01)\n");
	printf("[w] - increase velocity/position deadband (+0.01)\n");
	printf("[s] - decrease velocity/position deadband (-0.01)\n");
	printf("**********************************************************\n\n");
	printf("******* Virtual Environment Control keys *****************\n");
	printf("[r] - move the cube to its initial position (reset button)\n");
	printf("[x] - Exit application\n");
	printf("\n\n");

	//-----------------------------------------------------------------------
	// Haptic communication initialization
	//-----------------------------------------------------------------------

	printf("Current force deadband parameter = %f \n", ForceDeadbandParameter);
	printf("Current velocity deadband parameter = %f \n", VelocityDeadbandParameter);
	printf("Current position deadband parameter = %f \n", PositionDeadbandParameter);
	printf("Delay from Teleoperator to Operator is %d ms\n", ForceDelay);
	printf("Delay from Operator to Teleoperator is %d ms\n\n", CommandDelay);
	printf("Control Mode: Perceptual Deadband only \n");

	// initialized deadband classes for force and velocity
	DBForce = new DeadbandDataReduction(ForceDeadbandParameter);
	DBVelocity = new DeadbandDataReduction(VelocityDeadbandParameter);
	DBPosition = new DeadbandDataReduction(PositionDeadbandParameter);
	//--------------------------------------------------------------------------
	// socket communication setup
	//--------------------------------------------------------------------------
	QueryPerformanceFrequency(&cpuFreq);
	sockVersion = MAKEWORD(2, 2);
	sender = new Sender<hapticMessageM2S>();
	receiver = new Receiver<hapticMessageS2M>();
	commandQ = new threadsafe_queue<hapticMessageM2S>();
	forceQ = new threadsafe_queue<hapticMessageS2M>();


	if (WSAStartup(sockVersion, &data) != 0)
	{
		return 0;
	}

	sclient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sclient == INVALID_SOCKET)
	{
		printf("invalid socket!");
		return 0;
	}
	//绑定IP和端口  
	sockaddr_in sin;
	sin.sin_family = AF_INET;
	sin.sin_port = htons(8887);
	sin.sin_addr.S_un.S_addr = INADDR_ANY;
	if (bind(sclient, (LPSOCKADDR)&sin, sizeof(sin)) == SOCKET_ERROR)
	{
		printf("bind error !");
	}

	sockaddr_in serAddr;
	serAddr.sin_family = AF_INET;
	serAddr.sin_port = htons(8888);
	serAddr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	if (connect(sclient, (sockaddr *)&serAddr, sizeof(serAddr)) == SOCKET_ERROR)
	{  //连接失败 
		printf("connect error !");
		closesocket(sclient);
		//return 0;
	}


	sender->s = sclient;
	sender->Q = commandQ;
	receiver->s = sclient;
	receiver->Q = forceQ;
	unsigned  uiThread1ID;
	HANDLE hth1 = (HANDLE)_beginthreadex(NULL, // security
		0,             // stack size
		ThreadX::ThreadStaticEntryPoint,// entry-point-function
		sender,           // arg list holding the "this" pointer
		CREATE_SUSPENDED, // so we can later call ResumeThread()
		&uiThread1ID);
	ResumeThread(hth1);
	// From here on there are two separate threads executing
	// our one program.
	unsigned  uiThread2ID;
	HANDLE hth2 = (HANDLE)_beginthreadex(NULL, // security
		0,             // stack size
		ThreadX::ThreadStaticEntryPoint,// entry-point-function
		receiver,           // arg list holding the "this" pointer
		CREATE_SUSPENDED, // so we can later call ResumeThread()
		&uiThread2ID);
	ResumeThread(hth2);

	//--------------------------------------------------------------------------
	// HAPTIC DEVICE
	//--------------------------------------------------------------------------

	// create a haptic device handler
	handler = new cHapticDeviceHandler();

	// get a handle to the first haptic device
	handler->getDevice(hapticDevice, 0);

	// create a tool (cursor) and insert into the world
	tool = new cToolCursor(nullptr);

	// connect the haptic device to the tool
	tool->setHapticDevice(hapticDevice);

	// map the physical workspace of the haptic device to a larger virtual workspace.
	tool->setWorkspaceRadius(1.0);

	// define the radius of the tool (sphere)
	double toolRadius = 0.05;

	// define a radius for the tool
	tool->setRadius(toolRadius);

	// hide the device sphere. only show proxy.
	tool->setShowContactPoints(true, true);

	// enable if objects in the scene are going to rotate of translate
	// or possibly collide against the tool. If the environment
	// is entirely static, you can set this parameter to "false"
	tool->enableDynamicObjects(true);

	// haptic forces are enabled only if small forces are first sent to the device;
	// this mode avoids the force spike that occurs when the application starts when 
	// the tool is located inside an object for instance. 
	tool->setWaitForSmallForce(true);

	// start the haptic tool
	tool->start();

	//--------------------------------------------------------------------------
	// START SIMULATION
	//--------------------------------------------------------------------------

	// create a thread which starts the main haptics rendering loop
	hapticsThread = new cThread();
	hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

	// setup callback when application exits
	atexit(close);


	//--------------------------------------------------------------------------
	// MAIN GRAPHIC LOOP
	//--------------------------------------------------------------------------
	char a = 'a';
	// main graphic loop
	while (a != 'q')
	{
		std::cin >> a;
	}
	// exit
	return 0;
}

//------------------------------------------------------------------------------

void close(void)
{
	// stop the simulation
	simulationRunning = false;

	// wait for graphics and haptics loops to terminate
	while (!simulationFinished) { cSleepMs(100); }

	// close haptic device
	//hapticDevice->close();
	tool->stop();

	// delete resources
	delete hapticsThread;
	delete handler;
}

//------------------------------------------------------------------------------
int counter = 0;
void updateHaptics(void)
{
	// simulation in nowTimes running
	simulationRunning = true;
	simulationFinished = false;
	__int64 curtime;
	// main haptic simulation loop



	while (simulationRunning)
	{

#pragma region READ HAPTIC DATA FROM DEVICE
		/////////////////////////////////////////////////////////////////////
		// READ HAPTIC DEVICE
		/////////////////////////////////////////////////////////////////////

		// update position and orientation of tool
		tool->updateFromDevice();

		// read position 
		cVector3d position = tool->getDeviceLocalPos();
		//hapticDevice->getPosition(position);

		// read orientation 
		cMatrix3d rotation = tool->getDeviceLocalRot();
		//hapticDevice->getRotation(rotation);

		// read gripper position
		double gripperAngle = tool->getGripperAngleRad();
		//hapticDevice->getGripperAngleRad(gripperAngle);

		// read linear velocity 
		cVector3d linearVelocity = tool->getDeviceLocalLinVel();
		//hapticDevice->getLinearVelocity(linearVelocity);

		// read angular velocity
		cVector3d angularVelocity = tool->getDeviceLocalAngVel();
		//hapticDevice->getAngularVelocity(angularVelocity);

		// read gripper angular velocity
		double gripperAngularVelocity = tool->getGripperAngVel();
		//hapticDevice->getGripperAngularVelocity(gripperAngularVelocity);

		unsigned int allSwitches = tool->getUserSwitches();
		//hapticDevice->getUserSwitches(allSwitches);

		// read user-switch status (button 0)
		bool button0, button1, button2, button3;
		button0 = false;
		button1 = false;
		button2 = false;
		button3 = false;

		button0 = tool->getUserSwitch(0);
		button1 = tool->getUserSwitch(1);
		button2 = tool->getUserSwitch(2);
		button3 = tool->getUserSwitch(3);
#pragma endregion

#pragma region save TDPA-realsted data into variable
		//used to calculate Eout and damping
		for (int i = 0; i < 3; i++) {
			MasterVelocity[i] = linearVelocity(i);
			MasterPosition[i] = position(i);
		}

		if (FlagVelocityKalmanFilter == 1) {
			// Apply Kalman filtering to remove the noise on velocity signal
			VelocityKalmanFilter.ApplyKalmanFilter(MasterVelocity);
			MasterVelocity[0] = VelocityKalmanFilter.CurrentEstimation[0];
			MasterVelocity[1] = VelocityKalmanFilter.CurrentEstimation[1];
			MasterVelocity[2] = VelocityKalmanFilter.CurrentEstimation[2];
		}
#pragma endregion

#pragma region check receive queues and read Slave2Master message from it.
		/////////////////////////////////////////////////////////////////////
		// check receive queues.
		/////////////////////////////////////////////////////////////////////
		hapticMessageS2M msgS2M;

		if (!forceQ->empty()) {
			if (forceQ->try_pop(msgS2M)) {
				//todo  we reveive a message from slave side.
				//cVector3d force(msgS2M.force[0], msgS2M.force[1], msgS2M.force[2]);
				//cVector3d torque(msgS2M.torque[0], msgS2M.torque[1], msgS2M.torque[2]);
				//double gripperForce = msgS2M.gripperForce;
				// send computed force, torque, and gripper force to haptic device	
				//hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);
				//tool->setDeviceLocalForce(force);
				//tool->setDeviceLocalTorque(torque);
				//tool->setGripperForce(gripperForce);

				QueryPerformanceCounter((LARGE_INTEGER *)&curtime);
				//std::cout << freqCounterHaptics.getFrequency() << "master" << ((double)(curtime - msgS2M.time) / (double)cpuFreq.QuadPart) * 1000 ;
				//tool->applyToDevice();
		#pragma region TDPA
				/////////////////////////////////////////////////////////////////////
				// TDPA algorithm related code
				/////////////////////////////////////////////////////////////////////

				//get force and energy from Slave2Master message
				memcpy(MasterForce, msgS2M.force, 3 * sizeof(double));
				E_recv_m = msgS2M.energy;

				// 2. compute Emout and damping
				ComputeEnergy(Em_in, Em_out, MasterVelocity, MasterForce);
				if (Em_out > E_recv_m && abs(MasterVelocity[2]) > 0.001)
				{
					alpha_m = (Em_out - E_recv_m) / (sample_interval*MasterVelocity[2] * MasterVelocity[2]);
					Em_out = E_recv_m;
				}
				else
					alpha_m = 0;

				// 3. revise force and apply the revised force
				if (TDPAon)
					MasterForce[2] = MasterForce[2] - alpha_m*MasterVelocity[2];

				// 3.5 force filter (use dot(f) and tau)
				if (use_tauFilter)
				{
					ForceKalmanFilter.ApplyKalmanFilter(MasterForce);
					MasterForce[2] = ForceKalmanFilter.CurrentEstimation[2];
				}
		#pragma endregion

		#pragma region Apply Force
				// apply the master force to the device
				// here we minus MasterVelocity[2] * 0.15 
				// because there are still some tremble after filter 
				// and inherent characteristic of device will cause tremble
				cVector3d force(MasterForce[0], MasterForce[1], MasterForce[2]);
				tool->setDeviceLocalForce(force);
				tool->applyToDevice();
				lastMasterForce[2] = MasterForce[2];
		#pragma endregion
			}
		}
		else {
			memset(&msgS2M, 0, sizeof(hapticMessageS2M));
		}
#pragma endregion



#pragma region calculate velocity and Ein used to create Master2Slave message
		// 4. send master velocity and Ein
		// Apply deadband on position



		DBPosition->GetCurrentSample(MasterPosition);
		DBPosition->ApplyZOHDeadband(UpdatedPositionSample, &PositionTransmitFlag);

		if (PositionTransmitFlag == true) {

			PositionPacketNum++;

		}

		// Apply deadband on velocity
		DBVelocity->GetCurrentSample(MasterVelocity);
		DBVelocity->ApplyZOHDeadband(UpdatedVelocitySample, &VelocityTransmitFlag);

		if (VelocityTransmitFlag == true) {

			VelocityPacketNum++;
			// for TDPA
			E_trans_m = Em_in;
			Em_in_last = Em_in;
		}
		else
		{
			//for TDPA
			E_trans_m = Em_in_last;
			//---passive deadband-----
		}
#pragma endregion

#pragma region create message and send
		/////////////////////////////////////////////////////////////////////
		// create message to send
		/////////////////////////////////////////////////////////////////////



		hapticMessageM2S msgM2S;
		for (int i = 0; i < 3; i++) {
			msgM2S.position[i] = position(i);//modified by TDPA 
			msgM2S.linearVelocity[i] = linearVelocity(i);//modified by TDPA 
			msgM2S.angularVelocity[i] = angularVelocity(i);
			msgM2S.rotation[i] = rotation.getCol0()(i);
			msgM2S.rotation[i + 3] = rotation.getCol1()(i);
			msgM2S.rotation[i + 6] = rotation.getCol2()(i);
		}
		msgM2S.gripperAngle = gripperAngle;
		msgM2S.gripperAngularVelocity = gripperAngularVelocity;
		msgM2S.button0 = button0;
		msgM2S.button1 = button1;
		msgM2S.button2 = button2;
		msgM2S.button3 = button3;
		msgM2S.userSwitches = allSwitches;
		msgM2S.energy = E_trans_m;//modified by TDPA 
		QueryPerformanceCounter((LARGE_INTEGER *)&curtime);
		msgM2S.time = curtime;

		/////////////////////////////////////////////////////////////////////
		// push into sender queues and prepare to send by sender thread.
		/////////////////////////////////////////////////////////////////////
		//counter++;
		//if (counter == 1000) {
		//	counter = 0;
		std::cout<< "hello" << msgM2S.position[0] << " " << msgM2S.position[1] << " " << msgM2S.position[2] << " " << std::endl;
		commandQ->push(msgM2S);
		//}

#pragma endregion

		// signal frequency counter
		freqCounterHaptics.signal(1);
	}

	// exit haptics thread
	simulationFinished = true;
}

//------------------------------------------------------------------------------
