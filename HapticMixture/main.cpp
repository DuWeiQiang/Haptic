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
#include "hapticAlgorithm.h"
//------------------------------------------------------------------------------
#include "chai3d.h"
#include "CBullet.h"
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
namespace Master {
	double VelocityDeadbandParameter = cfg.getValueOfKey<double>("VelocityDeadbandParameter"); //deadband parameter for velcity data reduction, 0.1 is the default value
	double PositionDeadbandParameter = cfg.getValueOfKey<double>("PositionDeadbandParameter"); //deadband parameter for position data reduction, 0.1 is the default value

	int FlagVelocityKalmanFilter = cfg.getValueOfKey<int>("FlagVelocityKalmanFilter"); // 0: Kalman filter disabled 1: Kalman filter enabled on velocity signal
	KalmanFilter VelocityKalmanFilter; // applies 3 DoF kalman filtering to remove noise from velocity signal																				   
	bool FlagForceKalmanFilter = true;
	KalmanFilter ForceKalmanFilter; // applies 3 DoF kalman filtering to remove noise from force signal

	DeadbandDataReduction* DBVelocity; // data reduction class for velocity samples
	DeadbandDataReduction* DBPosition; // data reduction class for position samples

	bool VelocityTransmitFlag = false; // true: deadband triger false: keep last recently transmitted sample (ZoH)
	bool PositionTransmitFlag = false; // true: deadband triger false: keep last recently transmitted sample (ZoH)

	double MasterForce[3] = { 0.0, 0.0, 0.0 };
	double MasterVelocity[3] = { 0.0, 0.0, 0.0 }; // update 3 DoF master velocity sample (holds the signal before deadband)
	double MasterPosition[3] = { 0.0, 0.0, 0.0 }; // update 3 DoF master position sample (holds the signal before deadband)

	AlgorithmType ATypeChange = AlgorithmType::AT_None;

	TDPA_Algorithm TDPA;
	ISS_Algorithm ISS;

	SOCKET sServer;
	double delay = 0;
	std::queue<hapticMessageS2M> forceQ;
	char recData[1000];
	unsigned int unprocessedPtr = 0;
	// a virtual tool representing the haptic device in the scene
	cGenericTool* tool;
}

namespace Slave {
	double ForceDeadbandParameter = cfg.getValueOfKey<double>("ForceDeadbandParameter"); //deadband parameter for force data reduction, 0.1 is the default value

	int ControlMode = cfg.getValueOfKey<int>("ControlMode"); // 0: position control, 1:velocity control

	DeadbandDataReduction* DBForce; // data reduction class for force samples
	bool ForceTransmitFlag = false; // true: deadband triger false: keep last recently transmitted sample (ZoH)

	//------------------------------------------------------------------------------
	// TDPA variable and function realted code
	//------------------------------------------------------------------------------
	//----------TDPA---------------------
	double SlaveForce[3] = { 0.0,0.0,0.0 };  // current 3 DoF slave control force sample

	double MasterVelocity[3] = { 0.0, 0.0, 0.0 }; // update 3 DoF master velocity sample (holds the signal before deadband)
	double MasterPosition[3] = { 0.0, 0.0, 0.0 }; // update 3 DoF master position sample (holds the signal before deadband)
	double MasterForce[3] = { 0.0,0.0,0.0 };  // current 3 DoF force sample
	TDPA_Algorithm TDPA;

	SOCKET sServer;
	double delay = 0;
	std::queue<hapticMessageM2S> commandQ;
	char recData[1000];
	unsigned int unprocessedPtr = 0;

	// reset clock
	cPrecisionClock clock;
	// a virtual tool representing the haptic device in the scene
	cGenericTool* tool;
}
//
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




LARGE_INTEGER cpuFreq;
//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cBulletWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight *light;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;



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

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;


cBulletBox* bulletBox0;
cBulletBox* bulletBox1;

// bullet static walls and ground
cBulletStaticPlane* bulletInvisibleWall1;
cBulletStaticPlane* bulletInvisibleWall2;
cBulletStaticPlane* bulletInvisibleWall3;
cBulletStaticPlane* bulletInvisibleWall4;
cBulletStaticPlane* bulletInvisibleWall5;

WORD sockVersion;
WSADATA wsaData;

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

inline int MasterSocketInit() {
	//--------------------------------------------------------------------------
	// socket communication setup
	//--------------------------------------------------------------------------
	


	Master::sServer = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (Master::sServer == INVALID_SOCKET)
	{
		printf("invalid socket!");
		return 0;
	}
	//绑定IP和端口  
	sockaddr_in sin;
	sin.sin_family = AF_INET;
	sin.sin_port = htons(8887);
	sin.sin_addr.S_un.S_addr = INADDR_ANY;
	if (bind(Master::sServer, (LPSOCKADDR)&sin, sizeof(sin)) == SOCKET_ERROR)
	{
		printf("bind error !");
	}

	sockaddr_in serAddr;
	serAddr.sin_family = AF_INET;
	serAddr.sin_port = htons(4242);
	serAddr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	if (connect(Master::sServer, (sockaddr *)&serAddr, sizeof(serAddr)) == SOCKET_ERROR)
	{  //连接失败 
		printf("connect error !");
		closesocket(Master::sServer);
		//return 0;
	}
	unsigned long on_windows = 1;
	if (ioctlsocket(Master::sServer, FIONBIO, &on_windows) == SOCKET_ERROR) {
		printf("non-block error");
	}
}

inline int SlaveSocketInit() {
	//--------------------------------------------------------------------------
	// socket communication setup
	//--------------------------------------------------------------------------
	


	Slave::sServer = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (Slave::sServer == INVALID_SOCKET)
	{
		printf("invalid socket!");
		return 0;
	}
	//绑定IP和端口  
	sockaddr_in sin;
	sin.sin_family = AF_INET;
	sin.sin_port = htons(8889);
	sin.sin_addr.S_un.S_addr = INADDR_ANY;
	if (bind(Slave::sServer, (LPSOCKADDR)&sin, sizeof(sin)) == SOCKET_ERROR)
	{
		printf("bind error !");
	}

	sockaddr_in serAddr;
	serAddr.sin_family = AF_INET;
	serAddr.sin_port = htons(4242);
	serAddr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	if (connect(Slave::sServer, (sockaddr *)&serAddr, sizeof(serAddr)) == SOCKET_ERROR)
	{  //连接失败 
		printf("connect error !");
		closesocket(Slave::sServer);
		//return 0;
	}
	unsigned long on_windows = 1;
	if (ioctlsocket(Slave::sServer, FIONBIO, &on_windows) == SOCKET_ERROR) {
		printf("non-block error");
	}
}

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
	Master::DBVelocity = new DeadbandDataReduction(Master::VelocityDeadbandParameter);
	Master::DBPosition = new DeadbandDataReduction(Master::PositionDeadbandParameter);
	Slave::DBForce = new DeadbandDataReduction(Slave::ForceDeadbandParameter);

	QueryPerformanceFrequency(&cpuFreq);
	sockVersion = MAKEWORD(2, 2);
	if (WSAStartup(sockVersion, &wsaData) != 0)
	{
		return 0;
	}
	MasterSocketInit();
	SlaveSocketInit();
	//-----------------------------------------------------------------------
	// WORLD - CAMERA - LIGHTING
	//-----------------------------------------------------------------------

	// create a new world.
	world = new cBulletWorld();

	// set the background color of the environment
	world->m_backgroundColor.setWhite();

	// create a camera and insert it into the virtual world
	camera = new cCamera(world);
	world->addChild(camera);

	// position and orient the camera
	camera->set(cVector3d(2.5, 0.0, 1.3),    // camera position (eye)
		cVector3d(0.0, 0.0, 0.5),    // lookat position (target)
		cVector3d(0.0, 0.0, 1.0));   // direction of the "up" vector

									 // set the near and far clipping planes of the camera
	camera->setClippingPlanes(0.01, 10.0);

	// set stereo mode
	camera->setStereoMode(stereoMode);

	// set stereo eye separation and focal length (applies only if stereo is enabled)
	camera->setStereoEyeSeparation(0.02);
	camera->setStereoFocalLength(2.0);

	// set vertical mirrored display mode
	camera->setMirrorVertical(mirroredDisplay);

	// create a light source
	light = new cSpotLight(world);

	// attach light to camera
	world->addChild(light);

	// enable light source
	light->setEnabled(true);

	// position the light source
	light->setLocalPos(0.0, 0.0, 2.2);

	// define the direction of the light beam
	light->setDir(0.0, 0.0, -1.0);

	// set uniform concentration level of light 
	light->setSpotExponent(0.0);

	// enable this light source to generate shadows
	light->setShadowMapEnabled(true);

	// set the resolution of the shadow map
	light->m_shadowMap->setQualityLow();
	//light->m_shadowMap->setQualityMedium();

	// set light cone half angle
	light->setCutOffAngleDeg(45);
	//--------------------------------------------------------------------------
	// HAPTIC DEVICE
	//--------------------------------------------------------------------------

	// create a haptic device handler
	handler = new cHapticDeviceHandler();

	// get a handle to the first haptic device
	handler->getDevice(hapticDevice, 0);

	// retrieve information about the current haptic device
	cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

	// create a tool (gripper or pointer)
	if (hapticDeviceInfo.m_actuatedGripper)
	{
		Master::tool = new cToolGripper(world);
		Slave::tool = new cToolGripper(world);
	}
	else
	{
		Master::tool = new cToolCursor(world);
		Slave::tool = new cToolCursor(world);
	}

	// insert tool into world
	world->addChild(Slave::tool);

	// connect the haptic device to the tool
	Master::tool->setHapticDevice(hapticDevice);
	Slave::tool->setHapticDevice(hapticDevice);
	// map the physical workspace of the haptic device to a larger virtual workspace.
	Master::tool->setWorkspaceRadius(1.3);
	Slave::tool->setWorkspaceRadius(1.3);
	// define the radius of the tool (sphere)
	double toolRadius = 0.05;

	// define a radius for the tool
	Master::tool->setRadius(toolRadius);
	Slave::tool->setRadius(toolRadius);
	// hide the device sphere. only show proxy.
	Master::tool->setShowContactPoints(true, true);
	Slave::tool->setShowContactPoints(true, true);

	// enable if objects in the scene are going to rotate of translate
	// or possibly collide against the tool. If the environment
	// is entirely static, you can set this parameter to "false"
	Master::tool->enableDynamicObjects(true);
	Slave::tool->enableDynamicObjects(true);

	// haptic forces are enabled only if small forces are first sent to the device;
	// this mode avoids the force spike that occurs when the application starts when 
	// the tool is located inside an object for instance. 
	Master::tool->setWaitForSmallForce(true);
	Slave::tool->setWaitForSmallForce(true);

	// start the haptic tool
	// only start master haptic device.
	Master::tool->start();

	cVector3d position = Master::tool->getDeviceLocalPos();
	Slave::MasterPosition[0] = position.x();
	Slave::MasterPosition[1] = position.y();
	Slave::MasterPosition[2] = position.z();

	// read the scale factor between the physical workspace of the haptic
	// device and the virtual workspace defined for the tool
	double workspaceScaleFactor = Master::tool->getWorkspaceScaleFactor();
	// hapticDeviceInfo.m_workspaceRadius----->0.04
	// stiffness properties
	double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

	//120 is maxStiffness
	Master::ISS.mu_max = maxStiffness * Master::ISS.stiff_factor;

	world->setGravity(0.0, 0.0, -9.8);

	//////////////////////////////////////////////////////////////////////////
	// 3 BULLET BLOCKS
	//////////////////////////////////////////////////////////////////////////
	double size = 0.1;

	// create three objects that are added to the world
	bulletBox0 = new cBulletBox(world, 5 * size, size, 5 * size);
	world->addChild(bulletBox0);

	bulletBox1 = new cBulletBox(world, 5 * size, size, 5 * size);
	world->addChild(bulletBox1);

	// define some material properties for each cube
	cMaterial mat0, mat1;
	mat0.setRedIndian();
	mat0.setStiffness(0.3 * maxStiffness);
	mat0.setDynamicFriction(0.6);
	mat0.setStaticFriction(0.6);
	bulletBox0->setMaterial(mat0);

	mat1.setBlueRoyal();
	mat1.setStiffness(0.3 * maxStiffness);
	mat1.setDynamicFriction(0.6);
	mat1.setStaticFriction(0.6);
	bulletBox1->setMaterial(mat1);

	// define some mass properties for each cube
	bulletBox0->setMass(0.5);
	bulletBox1->setMass(0.5);


	// estimate their inertia properties
	bulletBox0->estimateInertia();
	bulletBox1->estimateInertia();


	// create dynamic models
	//bulletBox0->buildDynamicModel();
	bulletBox1->buildDynamicModel();


	// create collision detector for haptic interaction
	bulletBox0->createAABBCollisionDetector(toolRadius);
	bulletBox1->createAABBCollisionDetector(toolRadius);


	// set friction values
	bulletBox0->setSurfaceFriction(0.4);
	bulletBox1->setSurfaceFriction(0.4);


	// set position of each cube
	bulletBox0->setLocalPos(0.0, -0.0, 0.25);
	bulletBox1->setLocalPos(0.0, 0.6, 0.5);

	bulletBox0->setGhostEnabled(true);
	bulletBox1->m_bulletRigidBody->setLinearFactor(btVector3(0, 1, 1));
	bulletBox1->m_bulletRigidBody->setAngularFactor(btVector3(0, 0, 0));
	//////////////////////////////////////////////////////////////////////////
	// INVISIBLE WALLS
	//////////////////////////////////////////////////////////////////////////

	// we create 5 static walls to contain the dynamic objects within a limited workspace
	double planeWidth = 1.0;
	bulletInvisibleWall1 = new cBulletStaticPlane(world, cVector3d(0.0, 0.0, -1.0), -2.0 * planeWidth);
	bulletInvisibleWall2 = new cBulletStaticPlane(world, cVector3d(0.0, -1.0, 0.0), -planeWidth);
	bulletInvisibleWall3 = new cBulletStaticPlane(world, cVector3d(0.0, 1.0, 0.0), -planeWidth);
	bulletInvisibleWall4 = new cBulletStaticPlane(world, cVector3d(-1.0, 0.0, 0.0), -planeWidth);
	bulletInvisibleWall5 = new cBulletStaticPlane(world, cVector3d(1.0, 0.0, 0.0), -0.8 * planeWidth);

	//////////////////////////////////////////////////////////////////////////
	// GROUND
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	// GROUND
	//////////////////////////////////////////////////////////////////////////

	// create ground plane
	cBulletStaticPlane *ground = new cBulletStaticPlane(world, cVector3d(0.0, 0.0, 1.0), 0);

	// add plane to world as we will want to make it visibe
	world->addChild(ground);

	// create a mesh plane where the static plane is located
	cCreatePlane(ground, 3.0, 3.0, cVector3d(0, 0, 0));

	// define some material properties and apply to mesh
	cMaterial matGround;
	matGround.setStiffness(0.5 * maxStiffness);
	matGround.setDynamicFriction(0.2);
	matGround.setStaticFriction(0.0);
	matGround.setWhite();
	matGround.m_emission.setGrayLevel(0.3);
	ground->setMaterial(matGround);

	// setup collision detector for haptic interaction
	ground->createAABBCollisionDetector(toolRadius);

	// set friction values
	ground->setSurfaceFriction(0.4);
	// set material properties
	bool fileload;
	ground->m_texture = cTexture2d::create();
	fileload = ground->m_texture->loadFromFile("resources/wood.jpg");
	if (!fileload)
	{
		std::cout << "Error - Texture image failed to load correctly." << std::endl;
		close();
		return (-1);
	}

	// enable texture mapping
	ground->setUseTexture(true);
	ground->m_material->setWhite();

	// create normal map from texture data
	cNormalMapPtr normalMap0 = cNormalMap::create();
	normalMap0->createMap(ground->m_texture);
	ground->m_normalMap = normalMap0;
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

	else if (a_key == GLFW_KEY_I) {
		std::cout << "ISS enabled" << std::endl;
		Master::ISS.ISS_enabled = true;
		Master::ISS.Initialize();
		Master::ATypeChange = AlgorithmType::AT_ISS;

		Master::TDPA.TDPAon = false;
	}
	else if (a_key == GLFW_KEY_T) {
		std::cout << "TDPA enabled" << std::endl;
		Master::TDPA.TDPAon = true;
		Master::TDPA.Initialize();
		Master::ATypeChange = AlgorithmType::AT_TDPA;

		Master::ISS.ISS_enabled = false;
	}
	else if (a_key == GLFW_KEY_N) {
		std::cout << "None enabled" << std::endl;
		Master::TDPA.TDPAon = false;
		Master::ISS.ISS_enabled = false;
	}
	else if (a_key = GLFW_KEY_R) {
		world->setHapticEnabled(!world->getHapticEnabled());
		if (!world->getHapticEnabled()) {
			std::cout << "Postion has been modified; Press R to contunue";
			cVector3d position = Master::tool->getDeviceLocalPos();
			Slave::MasterPosition[0] = position.x();
			Slave::MasterPosition[1] = position.y();
			Slave::MasterPosition[2] = position.z();
		}		
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
	
	Master::tool->stop();
	// delete resources
	delete hapticsThread;
	delete world;
	delete handler;
}



//------------------------------------------------------------------------------


//------------------------------------------------------------------------------

inline void MasterUpdate() {
	/////////////////////////////////////////////////////////////////////
	// READ HAPTIC DEVICE
	/////////////////////////////////////////////////////////////////////

	// update position and orientation of tool
	Master::tool->updateFromDevice();

	// read position 
	cVector3d position = Master::tool->getDeviceLocalPos();
	//hapticDevice->getPosition(position);

	// read orientation 
	cMatrix3d rotation = Master::tool->getDeviceLocalRot();
	//hapticDevice->getRotation(rotation);

	// read gripper position
	double gripperAngle = Master::tool->getGripperAngleRad();
	//hapticDevice->getGripperAngleRad(gripperAngle);

	// read linear velocity 
	cVector3d linearVelocity = Master::tool->getDeviceLocalLinVel();
	//hapticDevice->getLinearVelocity(linearVelocity);

	// read angular velocity
	cVector3d angularVelocity = Master::tool->getDeviceLocalAngVel();
	//hapticDevice->getAngularVelocity(angularVelocity);

	// read gripper angular velocity
	double gripperAngularVelocity = Master::tool->getGripperAngVel();
	//hapticDevice->getGripperAngularVelocity(gripperAngularVelocity);

	unsigned int allSwitches = Master::tool->getUserSwitches();
	//hapticDevice->getUserSwitches(allSwitches);

	// read user-switch status (button 0)
	bool button0, button1, button2, button3;
	button0 = false;
	button1 = false;
	button2 = false;
	button3 = false;

	button0 = Master::tool->getUserSwitch(0);
	button1 = Master::tool->getUserSwitch(1);
	button2 = Master::tool->getUserSwitch(2);
	button3 = Master::tool->getUserSwitch(3);

	for (int i = 0; i < 3; i++) {
		Master::MasterVelocity[i] = linearVelocity(i);
		Master::MasterPosition[i] = position(i);
	}



	if (Master::FlagVelocityKalmanFilter == 1) {
		// Apply Kalman filtering to remove the noise on velocity signal
		Master::VelocityKalmanFilter.ApplyKalmanFilter(Master::MasterVelocity);
		Master::MasterVelocity[0] = Master::VelocityKalmanFilter.CurrentEstimation[0];
		Master::MasterVelocity[1] = Master::VelocityKalmanFilter.CurrentEstimation[1];
		Master::MasterVelocity[2] = Master::VelocityKalmanFilter.CurrentEstimation[2];
	}

	// Apply deadband on position
	Master::DBPosition->GetCurrentSample(Master::MasterPosition);
	Master::DBPosition->ApplyZOHDeadband(Master::MasterPosition, &Master::PositionTransmitFlag);

	if (Master::PositionTransmitFlag == true) {
	}

	// Apply deadband on velocity
	Master::DBVelocity->GetCurrentSample(Master::MasterVelocity);
	Master::DBVelocity->ApplyZOHDeadband(Master::MasterVelocity, &Master::VelocityTransmitFlag);

	Master::ISS.VelocityRevise(Master::MasterVelocity);

	if (Master::VelocityTransmitFlag == true) {
		memcpy(Master::TDPA.E_trans, Master::TDPA.E_in, 3*sizeof(double));
		memcpy(Master::TDPA.E_in_last, Master::TDPA.E_in, 3 * sizeof(double));
	}
	else
	{
		memcpy(Master::TDPA.E_trans, Master::TDPA.E_in_last, 3 * sizeof(double));
	}

#pragma region create message and send it
	/////////////////////////////////////////////////////////////////////
	// create message to send
	/////////////////////////////////////////////////////////////////////

	hapticMessageM2S msgM2S;
	for (int i = 0; i < 3; i++) {
		msgM2S.position[i] = Master::MasterVelocity[i];//modified by TDPA 
		msgM2S.linearVelocity[i] = Master::MasterVelocity[i];//modified by TDPA 
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
	memcpy(msgM2S.energy, Master::TDPA.E_trans, 3 * sizeof(double));//modified by TDPA 
	__int64 curtime;
	QueryPerformanceCounter((LARGE_INTEGER *)&curtime);
	msgM2S.time = curtime;
	msgM2S.ATypeChange = Master::ATypeChange;
	Master::ATypeChange = AlgorithmType::AT_KEEP;


	/////////////////////////////////////////////////////////////////////
	// push into send queues and prepare to send by sender thread.
	/////////////////////////////////////////////////////////////////////
	send(Master::sServer, (char *)&msgM2S, sizeof(hapticMessageM2S), 0);
	freqCounterHaptics.signal(1);

#pragma endregion


#pragma region check receive queues and apply force
	/////////////////////////////////////////////////////////////////////
	// check receive queues.
	/////////////////////////////////////////////////////////////////////
	hapticMessageS2M msgS2M;


	int ret = recv(Master::sServer, Master::recData + Master::unprocessedPtr, sizeof(Master::recData) - Master::unprocessedPtr, 0);
	if (ret > 0) {
		// we receive some char data and transform it to hapticMessageS2M.
		// if receive more than one hapticMessageS2M, only save the last one.
		Master::unprocessedPtr += ret;

		unsigned int hapticMsgL = sizeof(hapticMessageS2M);
		unsigned int i = 0;
		for (; i < Master::unprocessedPtr / hapticMsgL - 1; i++) {
			Master::forceQ.push(*(hapticMessageS2M*)(Master::recData + i* hapticMsgL));
		}
		std::queue<hapticMessageS2M> empty;
		Master::forceQ.swap(empty);
		Master::forceQ.push(*(hapticMessageS2M*)(Master::recData + i* hapticMsgL));
		unsigned int processedPtr = (Master::unprocessedPtr / hapticMsgL) * hapticMsgL;
		Master::unprocessedPtr %= hapticMsgL;

		for (unsigned int i = 0; i < Master::unprocessedPtr; i++) {
			Master::recData[i] = Master::recData[processedPtr + i];
		}
	}

	if (Master::forceQ.size()) {
		msgS2M = Master::forceQ.front();
		Master::forceQ.pop();

		QueryPerformanceCounter((LARGE_INTEGER *)&curtime);
		Master::delay = ((double)(curtime - msgS2M.time) / (double)cpuFreq.QuadPart) * 1000;

		//get force and energy from Slave2Master message
		memcpy(Master::MasterForce, msgS2M.force, 3 * sizeof(double));

		memcpy(Master::TDPA.E_recv, msgS2M.energy, 3 * sizeof(double));
		Master::TDPA.ForceRevise(Master::MasterVelocity, Master::MasterForce);

		//orce filter (use dot(f) and tau)
		if (Master::FlagForceKalmanFilter)
		{
			Master::ForceKalmanFilter.ApplyKalmanFilter(Master::MasterForce);
			Master::MasterForce[2] = Master::ForceKalmanFilter.CurrentEstimation[2];
		}

		Master::ISS.ForceRevise(Master::MasterForce);

		cVector3d force(Master::MasterForce[0], Master::MasterForce[1], Master::MasterForce[2] - Master::MasterVelocity[2] * 0.15);
		cVector3d torque(msgS2M.torque[0], msgS2M.torque[1], msgS2M.torque[2]);
		double gripperForce = msgS2M.gripperForce;
		Master::tool->setDeviceLocalForce(force);
		Master::tool->setDeviceLocalTorque(torque);
		Master::tool->setGripperForce(gripperForce);

	}
	Master::tool->applyToDevice();

#pragma endregion
}

inline void SlaveUpdate() {
	// compute global reference frames for each object
	world->computeGlobalPositions(true);
	/////////////////////////////////////////////////////////////////////
	// READ HAPTIC DEVICE
	/////////////////////////////////////////////////////////////////////
	hapticMessageM2S msgM2S;

	int ret = recv(Slave::sServer, Slave::recData + Slave::unprocessedPtr, sizeof(Slave::recData) - Slave::unprocessedPtr, 0);
	if (ret>0) {

		// we receive some char data and transform it to hapticMessageM2S.
		Slave::unprocessedPtr += ret;

		unsigned int hapticMsgL = sizeof(hapticMessageM2S);
		unsigned int i = 0;
		for (; i < Slave::unprocessedPtr / hapticMsgL - 1; i++) {
			Slave::commandQ.push(*(hapticMessageM2S*)(Slave::recData + i* hapticMsgL));
		}
		std::queue<hapticMessageM2S> empty;
		Slave::commandQ.swap(empty);
		Slave::commandQ.push(*(hapticMessageM2S*)(Slave::recData + i* hapticMsgL));
		unsigned int processedPtr = (Slave::unprocessedPtr / hapticMsgL) * hapticMsgL;
		Slave::unprocessedPtr %= hapticMsgL;

		for (unsigned int i = 0; i < Slave::unprocessedPtr; i++) {
			Slave::recData[i] = Slave::recData[processedPtr + i];
		}
	}
	if (Slave::commandQ.size()) {


		msgM2S = Slave::commandQ.front();
		Slave::commandQ.pop();

		//calculate M2S delay
		__int64 curtime;
		QueryPerformanceCounter((LARGE_INTEGER *)&curtime);
		Slave::delay = ((double)(curtime - msgM2S.time) / (double)cpuFreq.QuadPart) * 1000;

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

		switch (msgM2S.ATypeChange) {
		case AlgorithmType::AT_None:
			Slave::TDPA.TDPAon = false;
			break;
		case AlgorithmType::AT_TDPA:
			Slave::TDPA.TDPAon = true;
			Slave::TDPA.Initialize();
			break;
		case AlgorithmType::AT_ISS:
			Slave::TDPA.TDPAon = false;
			break;
		case AlgorithmType::AT_MMT:
			Slave::TDPA.TDPAon = false;
			break;
		case AlgorithmType::AT_KEEP:
			break;
		}



		memcpy(Slave::MasterVelocity, msgM2S.linearVelocity, 3 * sizeof(double));
		memcpy(Slave::TDPA.E_recv, msgM2S.energy, 3 * sizeof(double));
		Slave::TDPA.VelocityRevise(Slave::MasterVelocity, Slave::SlaveForce);

		if (Slave::ControlMode == 1) { // if velocity control mode is selected
								// Compute tool position using delayed velocity signal
			if (fabs(Slave::MasterVelocity[0]) < 10 && fabs(Slave::MasterVelocity[1]) < 10 && fabs(Slave::MasterVelocity[2]) < 10) {

				Slave::MasterPosition[0] = Slave::MasterPosition[0] + 0.001*Slave::MasterVelocity[0];
				Slave::MasterPosition[1] = Slave::MasterPosition[1] + 0.001*Slave::MasterVelocity[1];
				Slave::MasterPosition[2] = Slave::MasterPosition[2] + 0.001*Slave::MasterVelocity[2];
			}
		}
		else {
			memcpy(Slave::MasterPosition, msgM2S.position, 3 * sizeof(double));
		}
		position.x(Slave::MasterPosition[0]);
		position.y(Slave::MasterPosition[1]);
		position.z(Slave::MasterPosition[2]);
		// set the data into toolCursor
		Slave::tool->setDeviceLocalPos(position);
		Slave::tool->setDeviceLocalRot(rotation);
		Slave::tool->setDeviceLocalAngVel(angularVelocity);
		//tool->setDeviceLocalLinVel(linearVelocity);
		Slave::tool->setUserSwitches(allSwitches);
		Slave::tool->setGripperAngleRad(gripperAngle);
		Slave::tool->setGripperAngVel(gripperAngularVelocity);
		Slave::tool->setUserSwitch(0, button0);
		Slave::tool->setUserSwitch(1, button1);
		Slave::tool->setUserSwitch(2, button2);
		Slave::tool->setUserSwitch(3, button3);
		/////////////////////////////////////////////////////////////////////
		// COMPUTE AND APPLY FORCES
		/////////////////////////////////////////////////////////////////////

		// compute interaction forces
		Slave::tool->computeInteractionForces();




		cVector3d force = Slave::tool->getDeviceLocalForce();
		cVector3d torque = Slave::tool->getDeviceLocalTorque();
		double gripperForce = Slave::tool->getGripperForce();

		Slave::MasterForce[0] = force.x();
		Slave::MasterForce[1] = force.y();
		Slave::MasterForce[2] = force.z();

		Slave::SlaveForce[0] = -1 * force.x();
		Slave::SlaveForce[1] = -1 * force.y();
		Slave::SlaveForce[2] = -1 * force.z();

		// Slave side: Perceptual deadband data reduction is applied
		Slave::DBForce->GetCurrentSample(Slave::MasterForce); // pass the current sample for DB data reduction
		Slave::DBForce->ApplyZOHDeadband(Slave::MasterForce, &Slave::ForceTransmitFlag); // apply DB data reduction

		if (Slave::ForceTransmitFlag == true) {
			memcpy(Slave::TDPA.E_trans, Slave::TDPA.E_in, 3 * sizeof(double));
			memcpy(Slave::TDPA.E_in_last, Slave::TDPA.E_in, 3 * sizeof(double));
		}
		else
		{
			memcpy(Slave::TDPA.E_trans, Slave::TDPA.E_in_last, 3 * sizeof(double));
		}

		/////////////////////////////////////////////////////////////////////
		// Send Forces
		/////////////////////////////////////////////////////////////////////
		hapticMessageS2M msgS2M;
		for (int i = 0; i < 3; i++) {
			msgS2M.force[i] = Slave::MasterForce[i];// modified by TDPA
			msgS2M.torque[i] = torque(i);
		}
		msgS2M.gripperForce = gripperForce;
		memcpy(msgS2M.energy, Slave::TDPA.E_trans, 3 * sizeof(double));
		QueryPerformanceCounter((LARGE_INTEGER *)&curtime);
		msgS2M.time = curtime;
		send(Slave::sServer, (char *)&msgS2M, sizeof(hapticMessageS2M), 0);
	}

	/////////////////////////////////////////////////////////////////////
	// SIMULATION TIME    
	/////////////////////////////////////////////////////////////////////

	// stop the simulation clock
	Slave::clock.stop();

	// read the time increment in seconds
	double timeInterval = cClamp(Slave::clock.getcurrentTimeSeconds(), 0.0001, 0.001);

	// restart the simulation clock
	Slave::clock.reset();
	Slave::clock.start();

	/////////////////////////////////////////////////////////////////////
	// DYNAMIC SIMULATION
	/////////////////////////////////////////////////////////////////////

	// for each interaction point of the tool we look for any contact events
	// with the environment and apply forces accordingly
	int numInteractionPoints = Slave::tool->getNumHapticPoints();
	for (int i = 0; i<numInteractionPoints; i++)
	{
		// get pointer to next interaction point of tool
		cHapticPoint* interactionPoint = Slave::tool->getHapticPoint(i);

		// check all contact points
		int numContacts = interactionPoint->getNumCollisionEvents();
		for (int i = 0; i<numContacts; i++)
		{
			cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(i);

			// given the mesh object we may be touching, we search for its owner which
			// could be the mesh itself or a multi-mesh object. Once the owner found, we
			// look for the parent that will point to the Bullet object itself.
			cGenericObject* object = collisionEvent->m_object->getOwner()->getOwner();

			// cast to Bullet object
			cBulletGenericObject* bulletobject = dynamic_cast<cBulletGenericObject*>(object);

			// if Bullet object, we apply interaction forces
			if (bulletobject != NULL)
			{
				bulletobject->addExternalForceAtPoint(-interactionPoint->getLastComputedForce(),
					collisionEvent->m_globalPos - object->getLocalPos());
			}
		}
	}

	// update simulation
	world->updateDynamics(timeInterval);
}
void updateHaptics(void)
{
	// simulation in nowTimes running
	simulationRunning = true;
	simulationFinished = false;

	Slave::clock.reset();
	int i = 0;
	// main haptic simulation loop
	while (simulationRunning)
	{
		cVector3d pos = bulletBox0->getLocalPos();
		pos.y(1.5*sin(i++*0.001));
		bulletBox0->setLocalPos(pos);

		MasterUpdate();

		SlaveUpdate();


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
		cStr(freqCounterHaptics.getFrequency(), 0) + " Hz    S2M delay" + cStr(Slave::delay, 3) + " ");

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
