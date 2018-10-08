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
//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
using namespace chai3d;
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

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a label to display the haptic device model
cLabel* labelHapticDeviceModel;

// a label to display the position [m] of the haptic device
cLabel* labelHapticDevicePosition;

// a global variable to store the position [m] of the haptic device
cVector3d hapticDevicePosition;

// a global variable to store the velocity [m/s] of the haptic device
cVector3d hapticDeviceVelocity;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

//// a small sphere (cursor) representing the haptic device 
//cShapeSphere* cursor;

// a line representing the velocity of the haptic device
cShapeLine* velocity;

// a scope to monitor position values of haptic device
cScope* scope;

// a level widget to display velocity
cLevel* levelVelocity;

// three dials to display position information
cDial* dialPosX;
cDial* dialPosY;
cDial* dialPosZ;

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
Sender<hapticMessageS2M> *sender;
Receiver<hapticMessageM2S> *receiver;
threadsafe_queue<hapticMessageM2S> *forceQ;
threadsafe_queue<hapticMessageS2M> *commandQ;
SOCKET sClient;
SOCKET slisten;
LARGE_INTEGER cpuFreq;
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

	forceQ = new threadsafe_queue<hapticMessageM2S>();
	commandQ = new threadsafe_queue<hapticMessageS2M>();


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

	sender = new Sender<hapticMessageS2M>();
	receiver = new Receiver<hapticMessageM2S>();
	sender->s = sClient;
	sender->Q = commandQ;
	receiver->s = sClient;
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
}

inline int socketClientInit() {
	//--------------------------------------------------------------------------
	// socket communication setup
	//--------------------------------------------------------------------------
	QueryPerformanceFrequency(&cpuFreq);
	sockVersion = MAKEWORD(2, 2);
	sender = new Sender<hapticMessageS2M>();
	receiver = new Receiver<hapticMessageM2S>();
	forceQ = new threadsafe_queue<hapticMessageM2S>();
	commandQ = new threadsafe_queue<hapticMessageS2M>();


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


	sender->s = sClient;
	sender->Q = commandQ;
	receiver->s = sClient;
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

	//// create a sphere (cursor) to represent the haptic device
	//cursor = new cShapeSphere(0.01);

	//// insert cursor inside world
	//world->addChild(cursor);

	// create small line to illustrate the velocity of the haptic device
	velocity = new cShapeLine(cVector3d(0, 0, 0), cVector3d(0, 0, 0));

	// insert line inside world
	world->addChild(velocity);
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

	hapticMessageM2S msgM2S = *forceQ->wait_and_pop();
	std::cout << "helloworld";
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

	tool->setDeviceGlobalPos(position);
	tool->setDeviceLocalRot(rotation);
	tool->setDeviceLocalAngVel(angularVelocity);
	tool->setDeviceLocalLinVel(linearVelocity);
	tool->setUserSwitches(allSwitches);
	tool->setGripperAngleRad(gripperAngle);
	tool->setGripperAngVel(gripperAngularVelocity);
	tool->setUserSwitch(0, button0);
	tool->setUserSwitch(1, button1);
	tool->setUserSwitch(2, button2);
	tool->setUserSwitch(3, button3);
	tool->m_hapticPoint->initialize();

	//todo  transmit haptic device at first
	////////////////////////////////////////////////////////////////////////////

	// read the scale factor between the physical workspace of the haptic
	// device and the virtual workspace defined for the tool
	double workspaceScaleFactor = 25;//tool->getWorkspaceScaleFactor();

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

	// create a label to display the haptic device model
	labelHapticDeviceModel = new cLabel(font);
	camera->m_frontLayer->addChild(labelHapticDeviceModel);
	labelHapticDeviceModel->setText("falcon");

	// create a label to display the position of haptic device
	labelHapticDevicePosition = new cLabel(font);
	camera->m_frontLayer->addChild(labelHapticDevicePosition);

	// create a label to display the haptic and graphic rate of the simulation
	labelRates = new cLabel(font);
	camera->m_frontLayer->addChild(labelRates);


	

	////////////////////////////////////////////////////////////////////////////
	// In the following lines we set up several widgets to display position
	// and velocity data coming from the haptic device. For each widget we
	// define a range of values to expect from the haptic device. In this
	// example the units are meters (as we are tracking a position signal!) and 
	// have a set a default range between -0.1 to 0.1 meters. If you are using 
	// devices with a small or larger workspace, you may want to adjust these 
	// values accordingly. The other settings will modify the visual appearance
	// of the widgets. Have fun playing with these values!
	////////////////////////////////////////////////////////////////////////////

	// create a scope to plot haptic device position data
	scope = new cScope();
	camera->m_frontLayer->addChild(scope);
	scope->setLocalPos(100, 60);
	scope->setRange(-2.5, 2.5);
	scope->setSignalEnabled(true, true, true, false);
	scope->setTransparencyLevel(0.7);

	// create a level to display velocity data
	levelVelocity = new cLevel();
	camera->m_frontLayer->addChild(levelVelocity);
	levelVelocity->setLocalPos(20, 60);
	levelVelocity->setRange(0.0, 25.0);
	levelVelocity->setWidth(40);
	levelVelocity->setNumIncrements(46);
	levelVelocity->setSingleIncrementDisplay(false);
	levelVelocity->setTransparencyLevel(0.5);

	// three dials to display position data
	dialPosX = new cDial();
	camera->m_frontLayer->addChild(dialPosX);
	dialPosX->setLocalPos(750, 200);
	dialPosX->setRange(-2.5, 2.5);
	dialPosX->setSize(40);
	dialPosX->setSingleIncrementDisplay(true);

	dialPosY = new cDial();
	camera->m_frontLayer->addChild(dialPosY);
	dialPosY->setLocalPos(750, 140);
	dialPosY->setRange(-2.5, 2.5);
	dialPosY->setSize(40);
	dialPosY->setSingleIncrementDisplay(true);

	dialPosZ = new cDial();
	camera->m_frontLayer->addChild(dialPosZ);
	dialPosZ->setLocalPos(750, 80);
	dialPosZ->setRange(-2.5, 2.5);
	dialPosZ->setSize(40);
	dialPosZ->setSingleIncrementDisplay(true);

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

	// update position of label
	labelHapticDevicePosition->setLocalPos(20, width - 60, 0);

	// update position of label
	labelHapticDeviceModel->setLocalPos(20, height - 40, 0);

	// update position of scope
	scope->setSize(width - 200, 180);

	// update position of dials
	dialPosX->setLocalPos(width - 50, 210);
	dialPosY->setLocalPos(width - 50, 150);
	dialPosZ->setLocalPos(width - 50, 90);
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

void updateGraphics(void)
{
	/////////////////////////////////////////////////////////////////////
	// UPDATE WIDGETS
	/////////////////////////////////////////////////////////////////////

	// update position data
	labelHapticDevicePosition->setText(hapticDevicePosition.str(3));

	// update haptic and graphic rate data
	labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
		cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

	// update position of label
	labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);

	// update information to scope
	scope->setSignalValues(hapticDevicePosition.x(),
		hapticDevicePosition.y(),
		hapticDevicePosition.z());

	// update information to dials
	dialPosX->setValue(hapticDevicePosition.x());
	dialPosY->setValue(hapticDevicePosition.y());
	dialPosZ->setValue(hapticDevicePosition.z());

	// update velocity information to level
	levelVelocity->setValue(hapticDeviceVelocity.length());


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

//------------------------------------------------------------------------------

void updateHaptics(void)
{
	// simulation in nowTimes running
	simulationRunning = true;
	simulationFinished = false;

	// main haptic simulation loop
	while (simulationRunning)
	{
		// compute global reference frames for each object
		world->computeGlobalPositions(true);
		/////////////////////////////////////////////////////////////////////
		// READ HAPTIC DEVICE
		/////////////////////////////////////////////////////////////////////
		hapticMessageM2S msgM2S;
		if (!forceQ->empty()) {
			__int64 curtime;
			QueryPerformanceCounter((LARGE_INTEGER *)&curtime);
			forceQ->try_pop(msgM2S);
			std::cout << ((double)(curtime - msgM2S.time) / (double)cpuFreq.QuadPart) * 1000 << std::endl;
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

			tool->setDeviceGlobalPos(position);
			tool->setDeviceLocalRot(rotation);
			tool->setDeviceLocalAngVel(angularVelocity);
			tool->setDeviceLocalLinVel(linearVelocity);
			tool->setUserSwitches(allSwitches);
			tool->setGripperAngleRad(gripperAngle);
			tool->setGripperAngVel(gripperAngularVelocity);
			tool->setUserSwitch(0, button0);
			tool->setUserSwitch(1, button1);
			tool->setUserSwitch(2, button2);
			tool->setUserSwitch(3, button3);


			/////////////////////////////////////////////////////////////////////
			// UPDATE 3D CURSOR MODEL
			/////////////////////////////////////////////////////////////////////

			// update arrow
			velocity->m_pointA = position;
			velocity->m_pointB = cAdd(position, linearVelocity);

			// update position and orientation of cursor
			//cursor->setLocalPos(position);
			//cursor->setLocalRot(rotation);

			// adjust the  color of the cursor according to the status of
			// the user-switch (ON = TRUE / OFF = FALSE)
			if (button0)
			{
				tool->m_material->setGreenMediumAquamarine();
				//cursor->m_material->setGreenMediumAquamarine();
			}
			else if (button1)
			{
				tool->m_material->setYellowGold();
				//cursor->m_material->setYellowGold();
			}
			else if (button2)
			{
				tool->m_material->setOrangeCoral();
				//cursor->m_material->setOrangeCoral();
			}
			else if (button3)
			{
				tool->m_material->setPurpleLavender();
				//cursor->m_material->setPurpleLavender();
			}
			else
			{
				tool->m_material->setBlueRoyal();
				//cursor->m_material->setBlueRoyal();
			}

			// update global variable for graphic display update
			hapticDevicePosition = position;
			hapticDeviceVelocity = linearVelocity;


			/////////////////////////////////////////////////////////////////////
			// COMPUTE AND APPLY FORCES
			/////////////////////////////////////////////////////////////////////

			// compute interaction forces
			tool->computeInteractionForces();

			cVector3d force = tool->getDeviceLocalForce();
			cVector3d torque = tool->getDeviceLocalTorque();
			double gripperForce = tool->getGripperForce();

			//// apply force field
			//if (useForceField)
			//{
			//	// compute linear force
			//	double Kp = 25; // [N/m]
			//	cVector3d forceField = -Kp * position;
			//	force.add(forceField);

			//	// compute angular torque
			//	double Kr = 0.05; // [N/m.rad]
			//	cVector3d axis;
			//	double angle;
			//	rotation.toAxisAngle(axis, angle);
			//	torque = (-Kr * angle) * axis;
			//}

			hapticMessageS2M msgS2M;
			for (int i = 0; i < 3; i++) {
				msgS2M.force[i] = force(i);
				msgS2M.torque[i] = torque(i);
			}
			msgS2M.gripperForce = gripperForce;
			
			QueryPerformanceCounter((LARGE_INTEGER *)&curtime);
			msgS2M.time = curtime;
			commandQ->push(msgS2M);
		}
		// update frequency counter
		freqCounterHaptics.signal(1);
	}

	// exit haptics thread
	simulationFinished = true;
}

//------------------------------------------------------------------------------
