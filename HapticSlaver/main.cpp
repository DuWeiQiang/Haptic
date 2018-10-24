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
//------------------------------------------------------------------------------
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
bool TDPAon = true;
double lastMasterForce[3] = { 0.0, 0.0, 0.0 };   // use dot_f and tau to filter the master force
bool use_tauFilter = false;

double MasterVelocity[3] = { 0.0, 0.0, 0.0 }; // update 3 DoF master velocity sample (holds the signal before deadband)
double MasterPosition[3] = { 0.0, 0.0, 0.0 }; // update 3 DoF master position sample (holds the signal before deadband)

double CurrentForceSample[3] = { 0.0,0.0,0.0 };  // current 3 DoF force sample
double UpdatedForceSample[3] = { 0.0,0.0,0.0 };  // updated 3 DoF force sample (holds the signal after deadband)
double UpdatedVelocitySample[3] = { 0.0,0.0,0.0 }; // update 3 DoF velocity sample (holds the signal after deadband)
double UpdatedPositionSample[3] = { 0.0, 0.0, 0.0 }; // update 3 DoF position sample (holds the signal after deadband)

double ModifiedSlaveVel[3] = { 0.0, 0.0, 0.0 };
double ModifiedSlavePos[3] = { 0.0, 0.0, 0.0 };

double MasterForce[3] = { 0.0, 0.0, 0.0 };
double SlaveForce[3] = { 0.0, 0.0, 0.0 };


double PreviousPosition[3] = { 0.0,0.0,0.0 }; // buffer to keep previous position for next iteration
double PreviousSlaveEnergyIn[3] = { 0.0,0.0,0.0 }; // buffer to keep previous Slave Energy In for next iteration
double PreviousMasterEnergyIn[3] = { 0.0,0.0,0.0 }; // buffer to keep previous Master Energy In for next iteration

KalmanFilter ForceKalmanFilter; // applies 3 DoF kalman filtering to remove noise from force signal
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

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a flag for using damping (ON/OFF)
bool useDamping = false;

// a flag for using force field (ON/OFF)
bool useForceField = true;

// a flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// a flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

WORD sockVersion;
WSADATA wsaData;
std::queue<hapticMessageM2S> commandQ;
SOCKET sClient;
SOCKET slisten;
LARGE_INTEGER cpuFreq;
double delay;// M2S delay
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

inline int socketServerInit() {
	//--------------------------------------------------------------------------
	// socket communication setup
	//--------------------------------------------------------------------------
	QueryPerformanceFrequency(&cpuFreq);
	//初始化WSA  
	sockVersion = MAKEWORD(2, 2);

	if (WSAStartup(sockVersion, &wsaData) != 0)
	{

		std::cout << "helloworld" << std::endl;
		return 0;
	}

	//创建套接字  
	slisten = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (slisten == INVALID_SOCKET)
	{
		printf("socket error !");
		return 0;
	}

	//绑定IP和端口  
	sockaddr_in sin;
	sin.sin_family = AF_INET;
	sin.sin_port = htons(8888);
	sin.sin_addr.S_un.S_addr = INADDR_ANY;
	if (bind(slisten, (LPSOCKADDR)&sin, sizeof(sin)) == SOCKET_ERROR)
	{
		printf("bind error !");
	}

	//开始监听  
	if (listen(slisten, 5) == SOCKET_ERROR)
	{
		printf("listen error !");
		return 0;
	}

	//循环接收数据  
	sockaddr_in remoteAddr;
	int nAddrlen = sizeof(remoteAddr);

	printf("等待连接...\n");
	sClient = accept(slisten, (SOCKADDR *)&remoteAddr, &nAddrlen);
	if (sClient == INVALID_SOCKET)
	{
		printf("accept error !");
	}
	printf("接受到一个连接：%s:%d \r\n", inet_ntoa(remoteAddr.sin_addr), ntohs(remoteAddr.sin_port));

	unsigned long on_windows = 1;
	if (ioctlsocket(sClient, FIONBIO, &on_windows) == SOCKET_ERROR) {
		printf("non-block error");
	}
}

inline int socketClientInit() {
	//--------------------------------------------------------------------------
	// socket communication setup
	//--------------------------------------------------------------------------
	QueryPerformanceFrequency(&cpuFreq);
	sockVersion = MAKEWORD(2, 2);

	if (WSAStartup(sockVersion, &wsaData) != 0)
	{
		return 0;
	}

	sClient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sClient == INVALID_SOCKET)
	{
		printf("invalid socket!");
		return 0;
	}
	//绑定IP和端口  
	sockaddr_in sin;
	sin.sin_family = AF_INET;
	sin.sin_port = htons(8889);
	sin.sin_addr.S_un.S_addr = INADDR_ANY;
	if (bind(sClient, (LPSOCKADDR)&sin, sizeof(sin)) == SOCKET_ERROR)
	{
		printf("bind error !");
	}

	sockaddr_in serAddr;
	serAddr.sin_family = AF_INET;
	serAddr.sin_port = htons(4242);
	serAddr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	if (connect(sClient, (sockaddr *)&serAddr, sizeof(serAddr)) == SOCKET_ERROR)
	{  //连接失败 
		printf("connect error !");
		closesocket(sClient);
		//return 0;
	}

	unsigned long on_windows = 1;
	if (ioctlsocket(sClient, FIONBIO, &on_windows) == SOCKET_ERROR) {
		printf("non-block error");
	}
}
//==============================================================================
/*
DEMO:   03-analytics.cpp

This application illustrates how to program forces, torques and gripper
forces to your haptic device.

In this example the application opens an OpenGL window and displays a
3D cursor for the device connected to your computer. Several widgets are
used to demonstrate several possible approaches for to display signal data.
*/
//==============================================================================

int main(int argc, char* argv[])
{
	//--------------------------------------------------------------------------
	// INITIALIZATION
	//--------------------------------------------------------------------------
	std::cout << sizeof(DWORD) << "" << sizeof(hapticMessageS2M);
	std::cout << std::endl;
	std::cout << "-----------------------------------" << std::endl;
	std::cout << "CHAI3D" << std::endl;
	std::cout << "Demo: 03-analytics" << std::endl;
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

	socketClientInit();
	//--------------------------------------------------------------------------
	// OPEN GL - WINDOW DISPLAY
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
	camera->set(cVector3d(0.9, 0.0, 0.6),    // camera position (eye)
				cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
				cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

	// set the near and far clipping planes of the camera
	// anything in front or behind these clipping planes will not be rendered
	camera->setClippingPlanes(0.01, 10.0);

	// set stereo mode
	camera->setStereoMode(stereoMode);

	// set stereo eye separation and focal length (applies only if stereo is enabled)
	camera->setStereoEyeSeparation(0.03);
	camera->setStereoFocalLength(1.8);

	// set vertical mirrored display mode
	camera->setMirrorVertical(mirroredDisplay);

	// create a directional light source
	light = new cDirectionalLight(world);

	// insert light source inside world
	world->addChild(light);

	// enable light source
	light->setEnabled(true);

	// define direction of light beam
	light->setDir(-1.0, 0.0, 0.0);

	////////////////////////////////////////////////////////////////////////////
	//// retrieve information about the current haptic device
	//cHapticDeviceInfo info = hapticDevice->getSpecifications();-------------------operated by master

	// create a tool (cursor) and insert into the world
	tool = new cToolCursor(world);
	world->addChild(tool);
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
	// tool->setWaitForSmallForce(true); ------------------ - operated by master

	// start the haptic tool
	//tool->start();-------------------operated by master

	tool->m_hapticPoint->initialize();

	//todo  transmit haptic device at first
	////////////////////////////////////////////////////////////////////////////

	// read the scale factor between the physical workspace of the haptic
	// device and the virtual workspace defined for the tool
	double workspaceScaleFactor = 25;//tool->getWorkspaceScaleFactor();
	// hapticDeviceInfo.m_workspaceRadius----->0.04
	// stiffness properties
	double maxStiffness = 120;// hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

	/////////////////////////////////////////////////////////////////////////
	// BASE
	/////////////////////////////////////////////////////////////////////////

	// create a mesh
	cMesh* base = new cMesh();

	// add object to world
	world->addChild(base);

	// build mesh using a cylinder primitive
	cCreateCylinder(base,
		0.01,
		0.5,
		36,
		1,
		10,
		true,
		true,
		cVector3d(0.0, 0.0, -0.01),
		cMatrix3d(cDegToRad(0), cDegToRad(0), cDegToRad(0), C_EULER_ORDER_XYZ)
	);

	// set material properties
	base->m_material->setGrayGainsboro();
	base->m_material->setStiffness(0.5 * maxStiffness);

	// build collision detection tree
	base->createAABBCollisionDetector(toolRadius);

	// use display list to optimize graphic rendering performance
	base->setUseDisplayList(true);
	//--------------------------------------------------------------------------
	// WIDGETS
	//--------------------------------------------------------------------------

	// create a font
	font = NEW_CFONTCALIBRI20();

	// create a label to display the haptic and graphic rate of the simulation
	labelRates = new cLabel(font);
	camera->m_frontLayer->addChild(labelRates);
	
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

	// call window size callback at initialization
	windowSizeCallback(window, width, height);
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
	return (0);
}

//------------------------------------------------------------------------------

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
	if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
	{
		glfwSetWindowShouldClose(a_window, GLFW_TRUE);
	}

	// option - enable/disable force field
	if (a_key == GLFW_KEY_1)
	{
		useForceField = !useForceField;
		if (useForceField)
			std::cout << "> Enable force field     \r";
		else
			std::cout << "> Disable force field    \r";
	}

	// option - enable/disable damping
	if (a_key == GLFW_KEY_2)
	{
		useDamping = !useDamping;
		if (useDamping)
			std::cout << "> Enable damping         \r";
		else
			std::cout << "> Disable damping        \r";
	}

	// option - toggle fullscreen
	if (a_key == GLFW_KEY_F)
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
	if (a_key == GLFW_KEY_M)
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

	// delete resources
	delete hapticsThread;
	delete world;
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
	// Record the initial position of the haptic device
	PreviousPosition[0] = tool->getGlobalPos().x();
	PreviousPosition[1] = tool->getGlobalPos().y();
	PreviousPosition[2] = tool->getGlobalPos().z();
	// main haptic simulation loop
	while (simulationRunning)
	{
		// compute global reference frames for each object
		world->computeGlobalPositions(true);
		/////////////////////////////////////////////////////////////////////
		// READ HAPTIC DEVICE
		/////////////////////////////////////////////////////////////////////
		hapticMessageM2S msgM2S;

		int ret = recv(sClient, recData + unprocessedPtr, sizeof(recData) - unprocessedPtr, 0);
		if (ret>0) {
			
			// we receive some char data and transform it to hapticMessageM2S.
			unprocessedPtr += ret;

			unsigned int hapticMsgL = sizeof(hapticMessageM2S);
			for (unsigned int i = 0; i < unprocessedPtr / hapticMsgL; i++) {
				commandQ.push(*(hapticMessageM2S*)(recData + i* hapticMsgL));
			}
			unsigned int processedPtr = (unprocessedPtr / hapticMsgL) * hapticMsgL;
			unprocessedPtr %= hapticMsgL;

			for (unsigned int i = 0; i < unprocessedPtr; i++) {
				recData[i] = recData[processedPtr + i];
			}
		}
		if (commandQ.size()) {
			msgM2S = commandQ.front();
			commandQ.pop();

			//calculate M2S delay
			__int64 curtime;
			QueryPerformanceCounter((LARGE_INTEGER *)&curtime);			
			delay = ((double)(curtime - msgM2S.time) / (double)cpuFreq.QuadPart) * 1000;

			// read position 
			cVector3d position(msgM2S.position[0], msgM2S.position[1], msgM2S.position[2]);

			// read orientation 
			cVector3d col0(msgM2S.rotation[0], msgM2S.rotation[1], msgM2S.rotation[2]);
			cVector3d col1(msgM2S.rotation[3], msgM2S.rotation[4], msgM2S.rotation[5]);
			cVector3d col2(msgM2S.rotation[6], msgM2S.rotation[7], msgM2S.rotation[8]);
			cMatrix3d rotation(col0, col1, col2);

			// read gripper position
			double gripperAngle = msgM2S.gripperAngle;

			// read linear velocity 
			cVector3d linearVelocity(msgM2S.linearVelocity[0], msgM2S.linearVelocity[1], msgM2S.linearVelocity[2]);

			// read angular velocity
			cVector3d angularVelocity(msgM2S.angularVelocity[0], msgM2S.angularVelocity[1], msgM2S.angularVelocity[2]);

			// read gripper angular velocity
			double gripperAngularVelocity = msgM2S.gripperAngularVelocity;

			unsigned int allSwitches = msgM2S.userSwitches;

			// read user-switch status (button 0)
			bool button0, button1, button2, button3;
			button0 = msgM2S.button0;
			button1 = msgM2S.button1;
			button2 = msgM2S.button2;
			button3 = msgM2S.button3;

			E_recv_s = msgM2S.energy;
			memcpy(ModifiedSlavePos, msgM2S.position, 3 * sizeof(double));
			memcpy(ModifiedSlaveVel, msgM2S.linearVelocity, 3 * sizeof(double));


#pragma region TDPA
			// 2. compute Esout and damping
			ComputeEnergy(Es_in, Es_out, ModifiedSlaveVel, SlaveControlForce);
			if (Es_out > E_recv_s && abs(SlaveControlForce[2]) > 0.001)
			{
				beta_s = (Es_out - E_recv_s) / (sample_interval*SlaveControlForce[2] * SlaveControlForce[2]);
				Es_out = E_recv_s;
			}
			else
				beta_s = 0;

			// 3. revise slave vel 
			if (TDPAon)
				ModifiedSlaveVel[2] = ModifiedSlaveVel[2] - beta_s*SlaveControlForce[2];
#pragma endregion

			if (ControlMode == 1) { // if velocity control mode is selected
									// Compute tool position using delayed velocity signal
				if (fabs(ModifiedSlaveVel[0]) < 10 && fabs(ModifiedSlaveVel[1]) < 10 && fabs(ModifiedSlaveVel[2]) < 10) {

					position.x(PreviousPosition[0] + 0.001*ModifiedSlaveVel[0]);
					position.y(PreviousPosition[1] + 0.001*ModifiedSlaveVel[1]);
					position.z(PreviousPosition[2] + 0.001*ModifiedSlaveVel[2]);

				}
				else {

					position.x(PreviousPosition[0]);
					position.y(PreviousPosition[1]);
					position.z(PreviousPosition[2]);
				}

			}
			else
			{

				position.x(ModifiedSlavePos[0]);
				position.y(ModifiedSlavePos[1]);
				position.z(ModifiedSlavePos[2]);
			}

			// set the data into toolCursor
			tool->setDeviceLocalPos(position);
			tool->setDeviceLocalRot(rotation);
			tool->setDeviceLocalAngVel(angularVelocity);
			//tool->setDeviceLocalLinVel(linearVelocity);
			tool->setUserSwitches(allSwitches);
			tool->setGripperAngleRad(gripperAngle);
			tool->setGripperAngVel(gripperAngularVelocity);
			tool->setUserSwitch(0, button0);
			tool->setUserSwitch(1, button1);
			tool->setUserSwitch(2, button2);
			tool->setUserSwitch(3, button3);
			PreviousPosition[0] = position.x();
			PreviousPosition[1] = position.y();
			PreviousPosition[2] = position.z();
			/////////////////////////////////////////////////////////////////////
			// COMPUTE AND APPLY FORCES
			/////////////////////////////////////////////////////////////////////

			// compute interaction forces
			tool->computeInteractionForces();
			



			cVector3d force = tool->getDeviceLocalForce();
			cVector3d torque = tool->getDeviceLocalTorque();
			double gripperForce = tool->getGripperForce();

			CurrentForceSample[0] = force.x();
			CurrentForceSample[1] = force.y();
			CurrentForceSample[2] = force.z();

			SlaveControlForce[0] = -1 * force.x();
			SlaveControlForce[1] = -1 * force.y();
			SlaveControlForce[2] = -1 * force.z();

			// Slave side: Perceptual deadband data reduction is applied
			DBForce->GetCurrentSample(CurrentForceSample); // pass the current sample for DB data reduction
			DBForce->ApplyZOHDeadband(UpdatedForceSample, &ForceTransmitFlag); // apply DB data reduction

			if (ForceTransmitFlag == true) {
				ForcePacketNum++;
				E_trans_s = Es_in;
				Es_in_last = Es_in;
			}
			else
			{
				E_trans_s = Es_in_last;
			}

			/////////////////////////////////////////////////////////////////////
			// Send Forces
			/////////////////////////////////////////////////////////////////////
			hapticMessageS2M msgS2M;
			for (int i = 0; i < 3; i++) {
				msgS2M.force[i] = UpdatedForceSample[i];// modified by TDPA
				msgS2M.torque[i] = torque(i);
			}
			msgS2M.gripperForce = gripperForce;
			msgS2M.energy = E_trans_s;
			QueryPerformanceCounter((LARGE_INTEGER *)&curtime);
			msgS2M.time = curtime;
			send(sClient, (char *)&msgS2M, sizeof(hapticMessageS2M), 0); 
			freqCounterHaptics.signal(1);
		}
		// update frequency counter
		
		
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
		cStr(freqCounterHaptics.getFrequency(), 0) + " Hz " + "M2S delay:" + cStr(delay, 3));

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