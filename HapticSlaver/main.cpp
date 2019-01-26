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
#include "commTool.h"
#include "config.h"
#include "HapticCommLib.h"
//------------------------------------------------------------------------------
#include "chai3d.h"
#include "CBullet.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
#include <iomanip>
#include<fstream>
//------------------------------------------------------------------------------
using namespace chai3d;
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Read Parameters from configuration file
//------------------------------------------------------------------------------
ConfigFile cfg("cfg/config.cfg"); // get the configuration file
double ForceDeadbandParameter = cfg.getValueOfKey<double>("ForceDeadbandParameter"); //deadband parameter for force data reduction, 0.1 is the default value

int ControlMode = cfg.getValueOfKey<int>("ControlMode"); // 0: position control, 1:velocity control
std::string IP_master = cfg.getValueOfKey<std::string>("MasterIP");
std::string IP_slave = cfg.getValueOfKey<std::string>("SlaveIP");
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
cBulletWorld* world;

// a camera to render the world in the window display
cCamera* camera, *cameraMain;

// Four view panels
cViewPanel* viewPanel1;
cViewPanel* viewPanel2;

// Four framebuffer
cFrameBufferPtr frameBuffer1;
cFrameBufferPtr frameBuffer2;

// a light source to illuminate the objects in the world
cSpotLight *light;

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
sockaddr_in RecvAddr;
sockaddr_in SendAddr;
int AddrSize;
SOCKET sClient_Image;
sockaddr_in RecvAddr_Image;
sockaddr_in SendAddr_Image;

LARGE_INTEGER cpuFreq;
double delay;// M2S delay
Sender<hapticMessageS2M> *sender;
threadsafe_queue<hapticMessageS2M> backwardQ;

cBulletBox* bulletBox0;
cBulletBox* bulletBox1;


// bullet static walls and ground
cBulletStaticPlane* bulletInvisibleWall1;
cBulletStaticPlane* bulletInvisibleWall2;
cBulletStaticPlane* bulletInvisibleWall3;
cBulletStaticPlane* bulletInvisibleWall4;
cBulletStaticPlane* bulletInvisibleWall5;

cHapticDeviceInfo Falcon = {
	C_HAPTIC_DEVICE_FALCON,
	"Novint Technologies",
	"Falcon",
	8.0,      // [N]
	0.0,      // [N*m]
	0.0,      // [N]
	3000.0,   // [N/m]
	0.0,      // [N*m/Rad]
	0.0,      // [N*m/Rad]
	20.0,     // [N/(m/s)]
	0.0,      // [N*m/(Rad/s)]
	0.0,      // [N*m/(Rad/s)]
	0.04,     // [m]
	cDegToRad(0.0),
	true,
	false,
	false,
	true,
	false,
	false,
	true,
	true
};

cHapticDeviceInfo Touch = {
	C_HAPTIC_DEVICE_PHANTOM_TOUCH,
	"Touch",
	"Sensable Technologies",
	3.3,     // [N]
	0.0,     // [N*m]
	0.0,     // [N]
	400.0,   // [N/m]
	0.0,     // [N*m/Rad]
	0.0,     // [N/m]
	4.0,     // [N/(m/s)]
	0.0,     // [N*m/(Rad/s)]
	0.0,     // [N*m/(Rad/s)]
	0.075,    // [m];
	cDegToRad(0.0),
	true,
	true,
	false,
	true,
	false,
	false,
	true,
	true
};

class Game {
public:
	enum State { Stop, Running, episodeEnd };
	State state = State::Stop;
	unsigned int EpisodeIndex;
	double score;
	unsigned int slot;
	std::ofstream OutFile;
	std::string userName;
	cVector3d initialPos;
	Game() {
		OutFile.open("Test.txt");
		initialPos = cVector3d(0, 0, 0);
	}

	bool isEpisodeEnd() {
		if (bulletBox0->getLocalPos().y() > 0.9)
			return true;
		return false;
	};
	// press key s to start.
	void StartGame() {
		state = State::Running;
		EpisodeIndex++;
		Reset();
	};

	void Reset() {
		score = 0;
		slot = 0;
		bulletBox0->setLocalPos(initialPos);
	}
	// press key x to stop and start a new user.
	void StopGame() {
		state = State::Stop;
		userName = "hello";
		EpisodeIndex = 0;
		Reset();
	};

	void OutputScore() {
		// one record = UTC TimeStamp + EpisodeIndex + Score
		time_t now;
		time(&now);
		OutFile << userName << "," << now << "," << EpisodeIndex << "," << score << std::endl;
	};

	void EnvironmentUpdate() {
		if (state == State::Stop)return;
		cVector3d pos = bulletBox0->getLocalPos();
		pos.y(1.5*sin(slot++*0.001));
		bulletBox0->setLocalPos(pos);
		ScoreUpdate();
		if (isEpisodeEnd()) {
			state = State::Stop;
			OutputScore();
		}
	};

	void ScoreUpdate() {
		score += bulletBox0->getLocalPos().distance(bulletBox1->getLocalPos());
	};

	~Game() {
		OutFile.close();
	}
}game;
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

inline int socketServerInit(u_short listenPort, SOCKET &sClient) {
	SOCKET slisten;
	//--------------------------------------------------------------------------
	// socket communication setup
	//--------------------------------------------------------------------------
	QueryPerformanceFrequency(&cpuFreq);
	//initialize WSA  
	sockVersion = MAKEWORD(2, 2);

	if (WSAStartup(sockVersion, &wsaData) != 0)
	{

		std::cout << "helloworld" << std::endl;
		return 0;
	}

	//create socket 
	slisten = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (slisten == INVALID_SOCKET)
	{
		printf("socket error !");
		return 0;
	}

	//bind ip and port
	sockaddr_in sin;
	sin.sin_family = AF_INET;
	sin.sin_port = htons(listenPort);
	sin.sin_addr.S_un.S_addr = INADDR_ANY;
	if (bind(slisten, (LPSOCKADDR)&sin, sizeof(sin)) == SOCKET_ERROR)
	{
		printf("bind error !");
	}

	//start listening
	if (listen(slisten, 5) == SOCKET_ERROR)
	{
		printf("listen error !");
		return 0;
	}

	
	sockaddr_in remoteAddr;
	int nAddrlen = sizeof(remoteAddr);

	printf("wait to connection...\n");
	sClient = accept(slisten, (SOCKADDR *)&remoteAddr, &nAddrlen);
	if (sClient == INVALID_SOCKET)
	{
		printf("accept error !");
	}
	printf("accept a connection£º%s:%d \r\n", inet_ntoa(remoteAddr.sin_addr), ntohs(remoteAddr.sin_port));

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
	//bind ip and port
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
	{  //connection failed
		printf("connect error !");
		closesocket(sClient);
		//return 0;
	}

	unsigned long on_windows = 1;
	if (ioctlsocket(sClient, FIONBIO, &on_windows) == SOCKET_ERROR) {
		printf("non-block error");
	}
}
//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())
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
	//std::cout << sizeof(hapticMessageM2S) << " " << sizeof(hapticMessageS2M);
	std::cout << std::endl;
	std::cout << "-----------------------------------" << std::endl;
	std::cout << "Teleoperation" << std::endl;
	std::cout << "-----------------------------------" << std::endl << std::endl << std::endl;
	std::cout << "Keyboard Options:" << std::endl << std::endl;
	std::cout << "[X] - Stop Game" << std::endl;
	std::cout << "[S] - Start a new episode" << std::endl;
	std::cout << "-----------------------------------" << std::endl << std::endl << std::endl;
	std::cout << "Game score will store in a TXT document." << std::endl << std::endl << std::endl;
	std::cout << "-----------------------------------" << std::endl << std::endl << std::endl;
	std::cout << std::endl << std::endl;

	// initialized deadband classes for force and velocity
	DBForce = new DeadbandDataReduction(ForceDeadbandParameter);

	/*socketServerInit(888, sClient);
	socketServerInit(889, sClient_Image);*/
	QueryPerformanceFrequency(&cpuFreq);
	std::cout << cpuFreq.QuadPart << std::endl;
	sockVersion = MAKEWORD(2, 2);

	WSADATA wsaData;
	SOCKET SendSocket;
	WSAStartup(MAKEWORD(2, 2), &wsaData);

	sClient = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sClient == INVALID_SOCKET)
	{
		printf("invalid socket!");
		return 0;
	}

	RecvAddr.sin_family = AF_INET;
	RecvAddr.sin_port = htons(888);
	RecvAddr.sin_addr.s_addr = INADDR_ANY;
	if (bind(sClient, (SOCKADDR *)&RecvAddr, sizeof(RecvAddr))) {
		printf("bind error !");
	}
	SendAddr.sin_family = AF_INET;
	SendAddr.sin_port = htons(887);
	SendAddr.sin_addr.s_addr = inet_addr(IP_master.data());

	sClient_Image = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sClient_Image == INVALID_SOCKET)
	{
		printf("invalid socket!");
		return 0;
	}

	RecvAddr_Image.sin_family = AF_INET;
	RecvAddr_Image.sin_port = htons(889);
	RecvAddr_Image.sin_addr.s_addr = INADDR_ANY;
	bind(sClient_Image, (SOCKADDR *)&RecvAddr_Image, sizeof(RecvAddr_Image));

	SendAddr_Image.sin_family = AF_INET;
	SendAddr_Image.sin_port = htons(886);
	SendAddr_Image.sin_addr.s_addr = inet_addr(IP_master.data());

	unsigned long on_windows = 1;
	if (ioctlsocket(sClient, FIONBIO, &on_windows) == SOCKET_ERROR) {
		printf("non-block error");
	}
	if (ioctlsocket(sClient_Image, FIONBIO, &on_windows) == SOCKET_ERROR) {
		printf("non-block error");
	}

	AddrSize = sizeof(sockaddr_in);
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
	glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
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

	////////////////////////////////////////////////////////////////////////////
	//// retrieve information about the current haptic device
	//cHapticDeviceInfo info = hapticDevice->getSpecifications();-------------------operated by master

	// create a tool (cursor) and insert into the world
	tool = new cToolCursor(world);
	world->addChild(tool);
	cGenericHapticDevicePtr hapticDevice = cGenericHapticDevice::create();
	hapticDevice->m_specifications = Falcon;

	// connect the haptic device to the tool
	tool->setHapticDevice(hapticDevice);

	// map the physical workspace of the haptic device to a larger virtual workspace.
	tool->setWorkspaceRadius(1.3);

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

	tool->m_material->m_ambient.set(1.0, 1.0, 1.0);
	tool->m_material->m_diffuse.set(1.0, 1.0, 1.0);
	tool->m_material->m_specular.set(1.0, 1.0, 1.0);
	tool->setTransparencyLevel(1);

	//todo  transmit haptic device at first
	////////////////////////////////////////////////////////////////////////////

	// read the scale factor between the physical workspace of the haptic
	// device and the virtual workspace defined for the tool
	double workspaceScaleFactor = tool->getWorkspaceScaleFactor();//tool->getWorkspaceRadius() / Falcon.m_workspaceRadius;
																  // hapticDeviceInfo.m_workspaceRadius----->0.04
																  // stiffness properties
																  // retrieve information about the current haptic device
	cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();
	double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;//Falcon.m_maxLinearStiffness / workspaceScaleFactor;
	std::cout << "space factor and max stiffness: " << workspaceScaleFactor << ", " << maxStiffness << std::endl;
	world->setGravity(0.0, 0.0, -9.8);
	//////////////////////////////////////////////////////////////////////////
	// 3 BULLET BLOCKS
	//////////////////////////////////////////////////////////////////////////
	double size = 0.4;

	// create three objects that are added to the world
	bulletBox0 = new cBulletBox(world, size, size, size);
	//world->addChild(bulletBox0);

	bulletBox1 = new cBulletBox(world, size, size, size);
	world->addChild(bulletBox1);

	// define some material properties for each cube
	cMaterial mat0, mat1;
	mat0.setRedIndian();
	mat0.setStiffness(0.06 * maxStiffness);
	mat0.setDynamicFriction(0.5);
	mat0.setStaticFriction(0.5);
	bulletBox0->setMaterial(mat0);

	mat1.setBlueRoyal();
	mat1.setStiffness(0.06 * maxStiffness);
	mat1.setDynamicFriction(0.5);
	mat1.setStaticFriction(0.5);
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
	bulletBox0->setLocalPos(0.0, -0.6, 0.5);
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
	matGround.setStiffness(0.15 * maxStiffness);
	matGround.setDynamicFriction(0.3);
	matGround.setStaticFriction(0.35);
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




	////////////////////////////////////////////////////////////////////////////
	//// retrieve information about the current haptic device
	//cHapticDeviceInfo info = hapticDevice->getSpecifications();-------------------operated by master


	//////////////////////////////////////////////////////////////////////////
	// 3 BULLET BLOCKS
	//////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////
	// GROUND
	//////////////////////////////////////////////////////////////////////////


	if (!fileload)
	{
		std::cout << "Error - Texture image failed to load correctly." << std::endl;
		close();
		return (-1);
	}

	//--------------------------------------------------------------------------
	// WIDGETS
	//--------------------------------------------------------------------------

	// create a font

	//--------------------------------------------------------------------------
	// FRAMEBUFFERS
	//--------------------------------------------------------------------------

	// create framebuffer for view 1
	frameBuffer1 = cFrameBuffer::create();
	frameBuffer1->setup(camera);

	// create framebuffer for view 2
	frameBuffer2 = cFrameBuffer::create();


	//--------------------------------------------------------------------------
	// VIEW PANELS
	//--------------------------------------------------------------------------
	cameraMain = new cCamera(NULL);
	// create and setup view panel 1
	viewPanel1 = new cViewPanel(frameBuffer1);
	cameraMain->m_frontLayer->addChild(viewPanel1);

	// create and setup view panel 2
	viewPanel2 = new cViewPanel(frameBuffer2);
	cameraMain->m_frontLayer->addChild(viewPanel2);

	//--------------------------------------------------------------------------
	// START SIMULATION
	//--------------------------------------------------------------------------

	game.StopGame();

	// create a thread which starts the main haptics rendering loop
	hapticsThread = new cThread();
	hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

	// setup callback when application exits
	atexit(close);

	sender = new Sender<hapticMessageS2M>();
	sender->Q = &backwardQ;
	sender->s = sClient;
	sender->AddrSize = AddrSize;
	sender->SendAddr = SendAddr;
	unsigned  uiThread1ID;
	HANDLE hth1 = (HANDLE)_beginthreadex(NULL, // security
		0,             // stack size
		ThreadX::ThreadStaticEntryPoint,// entry-point-function
		sender,           // arg list holding the "this" pointer
		0, // so we can later call ResumeThread()
		&uiThread1ID);
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

	int halfW = width / 2;
	int halfH = height / 2;
	int offset = 1;

	// update display panel sizes and positions
	viewPanel1->setLocalPos(0.0, 0.0);
	viewPanel1->setSize(width, halfH);

	viewPanel2->setLocalPos(0.0, halfH + offset);
	viewPanel2->setSize(width, halfH);

	// update frame buffer sizes
	frameBuffer1->setSize(width, halfH);
	frameBuffer2->setSize(width, halfH);
	//std::cout << width << "hello" << halfH << std::endl;
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
	if (a_key == GLFW_KEY_D) {
		std::cout << "Dynamic Delay :" << !sender->dynamicDelay << std::endl;

		sender->dynamicDelay = !sender->dynamicDelay;
	}

	if (a_key == GLFW_KEY_S) {
		std::cout << "Game start" << std::endl;
		game.StartGame();
	}

	if (a_key == GLFW_KEY_X) {
		std::cout << "Game stop" << std::endl;
		game.StopGame();
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
	// reset clock
	cPrecisionClock clock;
	clock.reset();
	__int64 lastCounter = 0;
	QueryPerformanceCounter((LARGE_INTEGER *)&lastCounter);

	// main haptic simulation loop
	while (simulationRunning)
	{

		// compute global reference frames for each object
		world->computeGlobalPositions(true);
		/////////////////////////////////////////////////////////////////////
		// READ HAPTIC DEVICE
		/////////////////////////////////////////////////////////////////////
		hapticMessageM2S msgM2S;

		int ret = recvfrom(sClient, recData + unprocessedPtr, sizeof(recData) - unprocessedPtr, 0,(SOCKADDR *)&RecvAddr, &AddrSize);
		
		if (ret>0) {
			// we receive some char data and transform it to hapticMessageM2S.
			unprocessedPtr += ret;

			unsigned int hapticMsgL = sizeof(hapticMessageM2S);
			unsigned int i = 0;
			for (; i < unprocessedPtr / hapticMsgL; i++) {
				commandQ.push(*(hapticMessageM2S*)(recData + i* hapticMsgL));
			}
			//std::queue<hapticMessageM2S> empty;
			//commandQ.swap(empty);
			//commandQ.push(*(hapticMessageM2S*)(recData + i* hapticMsgL));
			unsigned int processedPtr = (unprocessedPtr / hapticMsgL) * hapticMsgL;
			unprocessedPtr %= hapticMsgL;

			for (unsigned int i = 0; i < unprocessedPtr; i++) {
				recData[i] = recData[processedPtr + i];
			}
		}
		while (commandQ.size()) {
			msgM2S = commandQ.front();


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

			/////////////////////////////////////////////////////////////////////
			// COMPUTE AND APPLY FORCES
			/////////////////////////////////////////////////////////////////////


			// update simulation

			if (commandQ.size() > 1) {
				commandQ.pop();
			}
			if (commandQ.size() == 1) {
				break;
			}

		}
		if (commandQ.size()) {


			msgM2S = commandQ.front();
			commandQ.pop();

			//calculate M2S delay
			__int64 curtime;
			QueryPerformanceCounter((LARGE_INTEGER *)&curtime);
			delay = ((double)(curtime - msgM2S.timestamp) / (double)cpuFreq.QuadPart) * 1000;

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

			memcpy(MasterVelocity, msgM2S.linearVelocity, 3 * sizeof(double));

			switch (msgM2S.ATypeChange) {
			case AlgorithmType::AT_None:
				ControlMode = 1;
				break;
			case AlgorithmType::AT_KEEP:
				break;
			}

			if (ControlMode == 1) { // if velocity control mode is selected
									// Compute tool position using delayed velocity signal
				if (fabs(MasterVelocity[0]) < 10 && fabs(MasterVelocity[1]) < 10 && fabs(MasterVelocity[2]) < 10) {

					MasterPosition[0] = MasterPosition[0] + 0.001*MasterVelocity[0];
					MasterPosition[1] = MasterPosition[1] + 0.001*MasterVelocity[1];
					MasterPosition[2] = MasterPosition[2] + 0.001*MasterVelocity[2];
				}
			}
			else {
				memcpy(MasterPosition, msgM2S.position, 3 * sizeof(double));
			}
			position.x(MasterPosition[0]);
			position.y(MasterPosition[1]);
			position.z(MasterPosition[2]);
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
			/////////////////////////////////////////////////////////////////////
			// COMPUTE AND APPLY FORCES
			/////////////////////////////////////////////////////////////////////

			// compute interaction forces
			tool->computeInteractionForces();


			cVector3d force = tool->getDeviceLocalForce();
			cVector3d torque = tool->getDeviceLocalTorque();
			double gripperForce = tool->getGripperForce();

			MasterForce[0] = force.x();
			MasterForce[1] = force.y();
			MasterForce[2] = force.z();

			// Slave side: Perceptual deadband data reduction is applied
			DBForce->GetCurrentSample(MasterForce); // pass the current sample for DB data reduction
			DBForce->ApplyZOHDeadband(MasterForce, &ForceTransmitFlag); // apply DB data reduction

			SlaveForce[0] = -1 * force.x();
			SlaveForce[1] = -1 * force.y();
			SlaveForce[2] = -1 * force.z();

			/////////////////////////////////////////////////////////////////////
			// Send Forces
			/////////////////////////////////////////////////////////////////////
			hapticMessageS2M msgS2M;
			for (int i = 0; i < 3; i++) {
				msgS2M.force[i] = MasterForce[i];// modified by TDPA
				msgS2M.torque[i] = torque(i);
			}

			msgS2M.gripperForce = gripperForce;
			QueryPerformanceCounter((LARGE_INTEGER *)&curtime);
			msgS2M.timestamp = curtime;
			//send(sClient, (char *)&msgS2M, sizeof(hapticMessageS2M), 0); 
			backwardQ.push(msgS2M);
			freqCounterHaptics.signal(1);
		}
		__int64 currentCounter;
		QueryPerformanceCounter((LARGE_INTEGER *)&currentCounter);
		if (currentCounter - lastCounter < 3609)continue;
		lastCounter = currentCounter;
		/////////////////////////////////////////////////////////////////////
		// SIMULATION TIME    
		/////////////////////////////////////////////////////////////////////
		game.EnvironmentUpdate();
		// stop the simulation clock
		clock.stop();

		// read the time increment in seconds
		double timeInterval = cClamp(clock.getcurrentTimeSeconds(), 0.0001, 0.001);

		// restart the simulation clock
		clock.reset();
		clock.start();

		/////////////////////////////////////////////////////////////////////
		// DYNAMIC SIMULATION
		/////////////////////////////////////////////////////////////////////

		// for each interaction point of the tool we look for any contact events
		// with the environment and apply forces accordingly
		int numInteractionPoints = tool->getNumHapticPoints();
		for (int i = 0; i<numInteractionPoints; i++)
		{
			// get pointer to next interaction point of tool
			cHapticPoint* interactionPoint = tool->getHapticPoint(i);

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
		cStr(freqCounterHaptics.getFrequency(), 0) + " Hz " + "S2M delay:" + cStr(delay, 3)
		+ " " + cStr(MasterVelocity[0], 3) + " " + cStr(MasterVelocity[1], 3) + " " + cStr(MasterVelocity[2], 3));

	// update position of label
	labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);

	/////////////////////////////////////////////////////////////////////
	// RENDER SCENE
	/////////////////////////////////////////////////////////////////////

	// update shadow maps (if any)
	world->updateShadowMaps(false, mirroredDisplay);

	// render all framebuffers
	frameBuffer1->renderView();
	frameBuffer2->renderView();
	cImagePtr ImgPtr = cImage::create();
	frameBuffer1->copyImageBuffer(ImgPtr);
	//send(sClient_Image, (const char *)ImgPtr->getData(), ImgPtr->getSizeInBytes(),0);
	//sendto(sClient_Image, (const char *)ImgPtr->getData(), ImgPtr->getSizeInBytes(), 0, (SOCKADDR *)&SendAddr_Image, sizeof(SendAddr));
	// render world
	cameraMain->renderView(width, height);
	ImgPtr->clear();
	// wait until all OpenGL commands are completed
	glFinish();

	// check for any OpenGL errors
	GLenum err;
	err = glGetError();
	if (err != GL_NO_ERROR) std::cout << "Error:  %s\n" << gluErrorString(err);
}


