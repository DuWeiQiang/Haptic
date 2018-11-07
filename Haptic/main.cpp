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
#include <iomanip>
//------------------------------------------------------------------------------
using namespace chai3d;
//using namespace std; //std namespace contains bind function which conflicts with socket bind function
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// Read Parameters from configuration file
// GENERAL SETTINGS
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
bool use_tauFilter = true;
double MasterForce[3] = { 0.0, 0.0, 0.0 };
double MasterVelocity[3] = { 0.0, 0.0, 0.0 }; // update 3 DoF master velocity sample (holds the signal before deadband)
double MasterPosition[3] = { 0.0, 0.0, 0.0 }; // update 3 DoF master position sample (holds the signal before deadband)
double UpdatedForceSample[3] = { 0.0,0.0,0.0 };  // updated 3 DoF force sample (holds the signal after deadband)
double UpdatedVelocitySample[3] = { 0.0,0.0,0.0 }; // update 3 DoF velocity sample (holds the signal after deadband)
double UpdatedPositionSample[3] = { 0.0, 0.0, 0.0 }; // update 3 DoF position sample (holds the signal after deadband)
KalmanFilter ForceKalmanFilter; // applies 3 DoF kalman filtering to remove noise from force signal

//----------ISS------------------------
inline void ISSVelocityRevise(double* vel, double d_force, double mu, double factor);
inline void ISSForceRevise(double * force, double d_force, double tau);
double mu_max = 10;
float stiff_factor = 0.5;
double d_force = 0.0;
double tau = 0.005;
bool ISS_enabled = true;
bool PassDB_enable = 0;
bool ISS_delay = 0;
double last_force = 0.0;
float mu_factor = 1.7;
double last_vel_slave = 0.0, last_vel_master = 0.0;
double last_force_master = 0.0, last_force_slave = 0.0;
double u_init = 0.01;
double um_max = u_init, um_max_neg = u_init;
double us_max = u_init, us_max_neg = u_init;
bool isContact = 0;
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

SOCKET sServer;
LARGE_INTEGER cpuFreq;
double delay = 0;
std::queue<hapticMessageS2M> forceQ;
//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// a flag for using damping (ON/OFF)
bool useDamping = false;

// a flag for using force field (ON/OFF)
bool useForceField = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;




// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

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
	std::cout << sizeof(hapticMessageM2S) << "" << sizeof(hapticMessageS2M);
	std::cout << std::endl; 
	std::cout << "-----------------------------------" << std::endl;
	std::cout << "CHAI3D" << std::endl;
	std::cout << "Demo: 01-mydevice" << std::endl;
	std::cout << "Copyright 2003-2016" << std::endl;
	std::cout << "-----------------------------------" << std::endl << std::endl << std::endl;
	std::cout << "Keyboard Options:" << std::endl << std::endl;
	std::cout << "[1] - Enable/Disable potential field" << std::endl;
	std::cout << "[2] - Enable/Disable damping" << std::endl;
	std::cout << "[f] - Enable/Disable full screen mode" << std::endl;
	std::cout << "[m] - Enable/Disable vertical mirroring" << std::endl;
	std::cout << "[q] - Exit application" << std::endl;
	std::cout << std::endl << std::endl;



	// initialized deadband classes for force and velocity
	DBForce = new DeadbandDataReduction(ForceDeadbandParameter);
	DBVelocity = new DeadbandDataReduction(VelocityDeadbandParameter);
	DBPosition = new DeadbandDataReduction(PositionDeadbandParameter);


	//--------------------------------------------------------------------------
	// socket communication setup
	//--------------------------------------------------------------------------
	QueryPerformanceFrequency(&cpuFreq);
	sockVersion = MAKEWORD(2, 2);

	if (WSAStartup(sockVersion, &data) != 0)
	{
		return 0;
	}

	sServer = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sServer == INVALID_SOCKET)
	{
		printf("invalid socket!");
		return 0;
	}
	//绑定IP和端口  
	sockaddr_in sin;
	sin.sin_family = AF_INET;
	sin.sin_port = htons(8887);
	sin.sin_addr.S_un.S_addr = INADDR_ANY;
	if (bind(sServer, (LPSOCKADDR)&sin, sizeof(sin)) == SOCKET_ERROR)
	{
		printf("bind error !");
	}

	sockaddr_in serAddr;
	serAddr.sin_family = AF_INET;
	serAddr.sin_port = htons(4242);
	serAddr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	if (connect(sServer, (sockaddr *)&serAddr, sizeof(serAddr)) == SOCKET_ERROR)
	{  //连接失败 
		printf("connect error !");
		closesocket(sServer);
		//return 0;
	}
	unsigned long on_windows = 1;
	if (ioctlsocket(sServer, FIONBIO, &on_windows) == SOCKET_ERROR) {
		printf("non-block error");
	}
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
	
	//120 is maxStiffness
	mu_max = 46;
	//--------------------------------------------------------------------------
	// START SIMULATION
	//--------------------------------------------------------------------------

	// create a thread which starts the main haptics rendering loop
	hapticsThread = new cThread();
	hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

	// setup callback when application exits
	atexit(close);

	//--------------------------------------------------------------------------
	// OPENGL - WINDOW DISPLAY
	//--------------------------------------------------------------------------

	// initialize GLFW library
	if (!glfwInit())
	{
		std::cout << "failed initialization" << std::endl;
		cSleepMs(1000);
		return 1;
	}

	// set error callback
	glfwSetErrorCallback(errorCallback);

	// compute desired size of window
	const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
	int w = 0.8 * mode->height;
	int h = 0.5 * mode->height;
	int x = 0.5 * (mode->width - w);
	int y = 0.5 * (mode->height - h);

	// set OpenGL version
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

	// set active stereo mode
	if (stereoMode == C_STEREO_ACTIVE)
	{
		glfwWindowHint(GLFW_STEREO, GL_TRUE);
	}
	else
	{
		glfwWindowHint(GLFW_STEREO, GL_FALSE);
	}

	// create display context
	window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
	if (!window)
	{
		std::cout << "failed to create window" << std::endl;
		cSleepMs(1000);
		glfwTerminate();
		return 1;
	}

	// get width and height of window
	glfwGetWindowSize(window, &width, &height);

	// set position of window
	glfwSetWindowPos(window, x, y);

	// set key callback
	glfwSetKeyCallback(window, keyCallback);

	// set resize callback
	glfwSetWindowSizeCallback(window, windowSizeCallback);

	// set current display context
	glfwMakeContextCurrent(window);

	// sets the swap interval for the current display context
	glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
	// initialize GLEW library
	if (glewInit() != GLEW_OK)
	{
		std::cout << "failed to initialize GLEW library" << std::endl;
		glfwTerminate();
		return 1;
	}
#endif


	//--------------------------------------------------------------------------
	// WORLD - CAMERA - LIGHTING
	//--------------------------------------------------------------------------

	// create a new world.
	world = new cWorld();

	// set the background color of the environment
	world->m_backgroundColor.setBlack();

	// create a camera and insert it into the virtual world
	camera = new cCamera(world);
	world->addChild(camera);

	// position and orient the camera
	camera->set(cVector3d(0.5, 0.0, 0.0),    // camera position (eye)
		cVector3d(0.0, 0.0, 0.0),    // look at position (target)
		cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

									 // set the near and far clipping planes of the camera
	camera->setClippingPlanes(0.01, 10.0);

	// set stereo mode
	camera->setStereoMode(stereoMode);

	// set stereo eye separation and focal length (applies only if stereo is enabled)
	camera->setStereoEyeSeparation(0.01);
	camera->setStereoFocalLength(0.5);

	// set vertical mirrored display mode
	camera->setMirrorVertical(mirroredDisplay);

	//--------------------------------------------------------------------------
	// WIDGETS
	//--------------------------------------------------------------------------

	// create a font
	font = NEW_CFONTCALIBRI20();

	// create a label to display the haptic and graphic rate of the simulation
	labelRates = new cLabel(font);
	camera->m_frontLayer->addChild(labelRates);
	//--------------------------------------------------------------------------
	// MAIN GRAPHIC LOOP
	//--------------------------------------------------------------------------

	// call window size callback at initialization
	windowSizeCallback(window, width, height);

	// main graphic loop
	while (!glfwWindowShouldClose(window))
	{
		// get width and height of window
		glfwGetWindowSize(window, &width, &height);

		// render graphics
		updateGraphics();

		// swap buffers
		glfwSwapBuffers(window);

		// process events
		glfwPollEvents();

		// signal frequency counter
		freqCounterGraphics.signal(1);
	}

	// close window
	glfwDestroyWindow(window);

	// terminate GLFW library
	glfwTerminate();
	// exit
	return 0;
}

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
	// update window size
	width = a_width;
	height = a_height;

}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
	std::cout << "Error: " << a_description << std::endl;
}

//------------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
	// filter calls that only include a key press
	if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
	{
		return;
	}

	// option - exit
	else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
	{
		glfwSetWindowShouldClose(a_window, GLFW_TRUE);
	}

	// option - enable/disable force field
	else if (a_key == GLFW_KEY_1)
	{
		useForceField = !useForceField;
		if (useForceField)
			std::cout << "> Enable force field     \r";
		else
			std::cout << "> Disable force field    \r";
	}

	// option - enable/disable damping
	else if (a_key == GLFW_KEY_2)
	{
		useDamping = !useDamping;
		if (useDamping)
			std::cout << "> Enable damping         \r";
		else
			std::cout << "> Disable damping        \r";
	}

	// option - toggle fullscreen
	else if (a_key == GLFW_KEY_F)
	{
		// toggle state variable
		fullscreen = !fullscreen;

		// get handle to monitor
		GLFWmonitor* monitor = glfwGetPrimaryMonitor();

		// get information about monitor
		const GLFWvidmode* mode = glfwGetVideoMode(monitor);

		// set fullscreen or window mode
		if (fullscreen)
		{
			glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
		else
		{
			int w = 0.8 * mode->height;
			int h = 0.5 * mode->height;
			int x = 0.5 * (mode->width - w);
			int y = 0.5 * (mode->height - h);
			glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
	}

	// option - toggle vertical mirroring
	else if (a_key == GLFW_KEY_M)
	{
		mirroredDisplay = !mirroredDisplay;
		camera->setMirrorVertical(mirroredDisplay);
	}
}

//------------------------------------------------------------------------------

void close(void)
{
	// stop the simulation
	simulationRunning = false;

	// wait for graphics and haptics loops to terminate
	while (!simulationFinished) { cSleepMs(100); }

	// close haptic device
	
	tool->stop();
	// delete resources
	delete hapticsThread;
	delete world;
	delete handler;
}



//------------------------------------------------------------------------------


//------------------------------------------------------------------------------


void updateHaptics(void)
{
	// simulation in nowTimes running
	simulationRunning = true;
	simulationFinished = false;

	char recData[1000];
	unsigned int unprocessedPtr = 0;
	
	// main haptic simulation loop
	__int64 beginTime;
	QueryPerformanceCounter((LARGE_INTEGER *)&beginTime);

	while (simulationRunning)
	{
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



#pragma region calculate velocity and Ein used to create M2S message
		for (int i = 0; i < 3; i++) {
			MasterVelocity[i] = linearVelocity(i);
			MasterPosition[i] = position(i);
		}
#pragma region ISS
		//----------------------------ISS velocity revising------------------------------------			
		if (ISS_enabled)
		{
			ISSVelocityRevise(MasterVelocity, d_force, mu_max, mu_factor);
		}
#pragma endregion
		if (FlagVelocityKalmanFilter == 1) {
			// Apply Kalman filtering to remove the noise on velocity signal
			VelocityKalmanFilter.ApplyKalmanFilter(MasterVelocity);
			MasterVelocity[0] = VelocityKalmanFilter.CurrentEstimation[0];
			MasterVelocity[1] = VelocityKalmanFilter.CurrentEstimation[1];
			MasterVelocity[2] = VelocityKalmanFilter.CurrentEstimation[2];
		}

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




#pragma region create message and send it
		/////////////////////////////////////////////////////////////////////
// create message to send
/////////////////////////////////////////////////////////////////////

		hapticMessageM2S msgM2S;
		for (int i = 0; i < 3; i++) {
			msgM2S.position[i] = UpdatedPositionSample[i];//modified by TDPA 
			msgM2S.linearVelocity[i] = UpdatedVelocitySample[i];//modified by TDPA 
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
		__int64 curtime;
		QueryPerformanceCounter((LARGE_INTEGER *)&curtime);
		msgM2S.time = curtime;


		/////////////////////////////////////////////////////////////////////
		// push into send queues and prepare to send by sender thread.
		/////////////////////////////////////////////////////////////////////
		beginTime = curtime;
		send(sServer, (char *)&msgM2S, sizeof(hapticMessageM2S), 0);
		freqCounterHaptics.signal(1);

#pragma endregion

		
#pragma region check receive queues and apply force
		/////////////////////////////////////////////////////////////////////
// check receive queues.
/////////////////////////////////////////////////////////////////////
		hapticMessageS2M msgS2M;


		int ret = recv(sServer, recData + unprocessedPtr, sizeof(recData) - unprocessedPtr, 0);

		

		if (ret > 0) {
			// we receive some char data and transform it to hapticMessageM2S.
			unprocessedPtr += ret;

			unsigned int hapticMsgL = sizeof(hapticMessageS2M);
			unsigned int i = 0;
			for (; i < unprocessedPtr / hapticMsgL - 1; i++) {
				forceQ.push(*(hapticMessageS2M*)(recData + i* hapticMsgL));
			}
			std::queue<hapticMessageS2M> empty;
			forceQ.swap(empty);
			forceQ.push(*(hapticMessageS2M*)(recData + i* hapticMsgL));
			unsigned int processedPtr = (unprocessedPtr / hapticMsgL) * hapticMsgL;
			unprocessedPtr %= hapticMsgL;

			for (unsigned int i = 0; i < unprocessedPtr; i++) {
				recData[i] = recData[processedPtr + i];
			}
		}
		
		if (forceQ.size()) {
			msgS2M = forceQ.front();
			forceQ.pop();
			
			// send computed force, torque, and gripper force to haptic device	


			QueryPerformanceCounter((LARGE_INTEGER *)&curtime);
			delay = ((double)(curtime - msgS2M.time) / (double)cpuFreq.QuadPart) * 1000;


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

#pragma region ISS
			//----------ISS force revising------------------------------------------
			if (ISS_enabled)
			{
				d_force = (MasterForce[2] - last_force) / 0.001;  // get derivation of force respect to time 
				last_force = MasterForce[2];
				ISSForceRevise(MasterForce, d_force, tau);
			}
#pragma endregion


			cVector3d force(MasterForce[0], MasterForce[1], MasterForce[2] - MasterVelocity[2] * 0.15);
			cVector3d torque(msgS2M.torque[0], msgS2M.torque[1], msgS2M.torque[2]);
			double gripperForce = msgS2M.gripperForce;
			tool->setDeviceLocalForce(force);
			tool->setDeviceLocalTorque(torque);
			tool->setGripperForce(gripperForce);
			
		}tool->applyToDevice();

#pragma endregion


	}

	// exit haptics thread
	simulationFinished = true;
}

//------------------------------------------------------------------------------
void updateGraphics(void)
{
	/////////////////////////////////////////////////////////////////////
	// UPDATE WIDGETS
	/////////////////////////////////////////////////////////////////////


	// update haptic and graphic rate data
	labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
		cStr(freqCounterHaptics.getFrequency(), 0) + " Hz    S2M delay" + cStr(delay, 0) + " ");

	// update position of label
	labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);


	/////////////////////////////////////////////////////////////////////
	// RENDER SCENE
	/////////////////////////////////////////////////////////////////////

	// update shadow maps (if any)
	world->updateShadowMaps(false, mirroredDisplay);

	// render world
	camera->renderView(width, height);

	// wait until all OpenGL commands are completed
	glFinish();

	// check for any OpenGL errors
	GLenum err;
	err = glGetError();
	if (err != GL_NO_ERROR) std::cout << "Error:  %s\n" << gluErrorString(err);
}


//-----------TDPA compute energy--------------
void ComputeEnergy(double &Ein, double &Eout, double vel[3], double force[3])
{
	// only for z direction
	double power = vel[2] * (-1 * force[2]);
	if (power >= 0)	{
		Ein = Ein + sample_interval*power;
	}
	else{
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

//-----------------for ISS control---------------------
inline void ISSVelocityRevise(double* vel, double d_force, double mu, double factor)
{
	vel[2] = vel[2] + d_force / (mu*factor);
}

inline void ISSForceRevise(double * force, double d_force, double tau)
{
	force[2] = force[2] + d_force*tau;  // use "+" because MasterForce direction is opposite to f_e in the paper
}