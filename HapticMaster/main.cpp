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
//------------------------------------------------------------------------------
using namespace chai3d;
//using namespace std; //std namespace contains bind function which conflicts with socket bind function
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Read Parameters from configuration file
// GENERAL SETTINGS
//------------------------------------------------------------------------------
ConfigFile cfg("cfg/config.cfg"); // get the configuration file
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

double MasterTorque[3] = { 0.0, 0.0, 0.0 };
double MasterGripperForce = 0.0;
AlgorithmType ATypeChange = AlgorithmType::AT_None;

Sender<hapticMessageM2S> *sender;
threadsafe_queue<hapticMessageM2S> forwardQ;

class MMT_ALGORITHM {
public:
	bool enable = false;
	/*
	enviroment variable:
	1. box position;
	3. K used to calculate the force
	*/
	static struct envPar
	{
		cVector3d parPosition;
		cVector3d parStiffness;
		bool Flag;//whether this parameters is used to update the master's enviroment
	};
	envPar MasterPar = { cVector3d(0, 0, 0) ,cVector3d(0, 0, 0) ,false };
	envPar SlavePar = { cVector3d(0, 0, 0) ,cVector3d(0, 0, 0) ,true };
	cVector3d oldStiffness;
	int length = 100;
	int index = 0;
	cVector3d parStiffness[100];//store parameter K used to calculate the average K value.

	void ForceRevise(double* force, cVector3d goalPos, cVector3d proxyPos) {
		if (!enable)return;
		cVector3d Temp = MasterPar.parStiffness;
		Temp.mulElement(proxyPos - goalPos);
		force[0] = Temp(0);
		force[1] = Temp(1);
		force[2] = Temp(2);
	}

	void Initialize() {
		MasterPar = { cVector3d(0, 0, 0) ,cVector3d(0, 0, 0) ,false };
		SlavePar = { cVector3d(0, 0, 0) ,cVector3d(0, 0, 0) ,true };
		index = 0;
		oldStiffness = cVector3d(0, 0, 0);
		for (int i = 0; i < 100; i++) {
			parStiffness[i] = cVector3d(0, 0, 0);
		}
	}

	cVector3d Average(cVector3d temp) {
		if (index < 100) {
			parStiffness[index] = temp;
			index += 1;
		}
		else {
			parStiffness[index % 100] = temp;
			index = index % 100;
		}
		cVector3d avg;
		for (int i = 0; i < 100; i++) {
			avg += parStiffness[i];
		}
		avg /= 100;
		return avg;
	}

	bool isTransmit() {
		for (int i = 0; i < 3; i++) {
			// The threshold of  change rate is 10 percentage
			if (abs(1 - SlavePar.parStiffness(i) / MasterPar.parStiffness(i)) > 0.1)
				return true;
			// The threshold of  position offset is 0.1 meters
			if (abs(MasterPar.parPosition(i) - SlavePar.parPosition(i)) > 0.1)
				return true;
		}
		return false;
	}

	/*
	dead band parameter
	*/
	double db;
}MMT;

class WAVE_ALGORITHM {
public:
	// b=1.2 for Touch
	double b = 8;	//damping factor
	bool waveOn = false;
	double scaleFactor = 1;
	KalmanFilter *KF = new KalmanFilter();
	// WAVE algorithm variables
	struct WaveV
	{

		cVector3d ul;	//sent signal OP
		cVector3d vl;	//received signal OP
		cVector3d ur;	//sent signal TOP
		cVector3d vr;	//received signal OP
		cVector3d F;	//force at OP
	};

	WaveV WV = { cVector3d(0,0,0),cVector3d(0,0,0), cVector3d(0,0,0), cVector3d(0,0,0), cVector3d(0,0,0) };

	void VelocityRevise(double* vel, WaveV* wave, double* force) {
		if (waveOn) {
			cVector3d temp = getVel_r(b, wave, cVector3d(force[0], force[1], force[2]));
			vel[0] = temp.x() * scaleFactor;
			vel[1] = temp.y() * scaleFactor;
			vel[2] = temp.z() * scaleFactor;
		}
	}

	void ForceRevise(double* vel, WaveV* wave, double* force) {
		if (waveOn) {
			cVector3d temp = getForce_l(b, wave, cVector3d(vel[0], vel[1], vel[2]) / scaleFactor);
			force[0] = -1 * temp.x();
			force[1] = -1 * temp.y();
			force[2] = -1 * temp.z();
		}
	}

	void getWave_l(double b, WaveV* wave, cVector3d vel)
	{
		if (waveOn) {
			wave->ul = sqrt(2 * b)*vel / scaleFactor + wave->vl;
			double ttemp[3] = { WV.ul.x(),WV.ul.y(),WV.ul.z() };
			KF->ApplyKalmanFilter(ttemp);
			WV.ul.x(KF->CurrentEstimation[0]);
			WV.ul.y(KF->CurrentEstimation[1]);
			WV.ul.z(KF->CurrentEstimation[2]);
		}
	}

	void getWave_r(double b, WaveV* wave, cVector3d f)
	{
		wave->ur = (sqrt(2 / b)*f - wave->vr);
	}

	cVector3d getVel_r(double b, WaveV* wave, cVector3d f)
	{
		return -1 / b*(f - sqrt(2 * b)*wave->vr);
	}

	cVector3d getForce_l(double b, WaveV* wave, cVector3d vel)
	{
		return 1 * b*vel / scaleFactor + sqrt(2 * b)*wave->vl;
	}

	void Initialize() {
		WV = { cVector3d(0,0,0),cVector3d(0,0,0), cVector3d(0,0,0), cVector3d(0,0,0), cVector3d(0,0,0) };
		delete KF;
		KF = new KalmanFilter();
	}
}WAVE;

class TDPA_Algorithm {
public:
	double sample_interval = 0.001;   //1kHz
	double E_in[3] = { 0,0,0 }, E_out[3] = { 0,0,0 };
	double E_in_last[3] = { 0,0,0 };
	double E_trans[3] = { 0,0,0 }, E_recv[3] = { 0,0,0 };
	double alpha[3] = { 0,0,0 }, beta[3] = { 0,0,0 };
	bool TDPAon = false;
	void ComputeEnergy(double vel[3], double force[3])
	{
		for (int i = 0; i < 3; i++) {
			// only for z direction
			double power = vel[i] * (-1 * force[i]);
			if (power >= 0) {
				E_in[i] = E_in[i] + sample_interval*power;
			}
			else {
				E_out[i] = E_out[i] - sample_interval*power;
			}
		}

	};
	// only used by master
	void ForceRevise(double* Vel, double* force) {
		ComputeEnergy(Vel, force);
		for (int i = 0; i < 3; i++) {
			if (E_out[i] > E_recv[i] && abs(Vel[i]) > 0.001)
			{
				alpha[i] = (E_out[i] - E_recv[i]) / (sample_interval*Vel[i] * Vel[i]);
				E_out[i] = E_recv[i];
			}
			else
				alpha[i] = 0;

			// 3. revise force and apply the revised force
			if (TDPAon)
				force[i] = force[i] - alpha[i] * Vel[i];
		}
	};
	// only used by slavor
	void VelocityRevise(double* Vel, double* force) {
		ComputeEnergy(Vel, force);
		for (int i = 0; i < 3; i++) {
			if (E_out[i] > E_recv[i] && abs(force[i]) > 0.001)
			{
				beta[i] = (E_out[i] - E_recv[i]) / (sample_interval*force[i] * force[i]);
				E_out[i] = E_recv[i];
			}
			else
				beta[i] = 0;

			// 3. revise slave vel 
			if (TDPAon)
				Vel[i] = Vel[i] - beta[i] * force[i];
		}
	};

	void Initialize()
	{
		memset(E_in, 0, 3 * sizeof(double));
		memset(E_out, 0, 3 * sizeof(double));
		memset(E_in_last, 0, 3 * sizeof(double));
		memset(E_trans, 0, 3 * sizeof(double));
		memset(E_recv, 0, 3 * sizeof(double));
		memset(alpha, 0, 3 * sizeof(double));
		memset(beta, 0, 3 * sizeof(double));
	};
}TDPA;

class ISS_Algorithm {
public:
	double mu_max = 10;
	float stiff_factor = 0.5;
	double d_force[3] = { 0,0,0 };
	double tau = 0.005;
	bool ISS_enabled = false;
	double last_force[3] = { 0,0,0 };
	float mu_factor = 1.7;

	void VelocityRevise(double* vel) {
		if (ISS_enabled) {
			for (int i = 0; i < 3; i++) {
				vel[i] = vel[i] + d_force[i] / (mu_max*mu_factor);
			}

		}
	};

	void ForceRevise(double* force) {
		if (ISS_enabled) {
			for (int i = 0; i < 3; i++) {
				d_force[i] = (force[i] - last_force[i]) / 0.001;  // get derivation of force respect to time 
				last_force[i] = force[i];
				force[i] = force[i] + d_force[i] * tau;  // use "+" because MasterForce direction is opposite to f_e in the paper
			}
		}
	};

	void Initialize()
	{
		memset(last_force, 0, 3 * sizeof(double));
	};
}ISS;

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
SOCKET sServer_Image;
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


inline int socketClientInit(const char* addr, u_short remoteport, u_short myPort,SOCKET &sServer) {
	//--------------------------------------------------------------------------
	// socket communication setup
	//--------------------------------------------------------------------------
	QueryPerformanceFrequency(&cpuFreq);
	std::cout << cpuFreq.QuadPart << std::endl;
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
	sin.sin_port = htons(myPort);
	sin.sin_addr.S_un.S_addr = INADDR_ANY;
	if (bind(sServer, (LPSOCKADDR)&sin, sizeof(sin)) == SOCKET_ERROR)
	{
		printf("bind error !");
	}

	sockaddr_in serAddr;
	serAddr.sin_family = AF_INET;
	serAddr.sin_port = htons(remoteport);
	serAddr.sin_addr.S_un.S_addr = inet_addr(addr);
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
}

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
cBitmap* bitmap;
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
	DBVelocity = new DeadbandDataReduction(VelocityDeadbandParameter);
	DBPosition = new DeadbandDataReduction(PositionDeadbandParameter);


	socketClientInit("127.0.0.1", 888, 887, sServer);
	socketClientInit("127.0.0.1", 889, 886, sServer_Image);

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


	//--------------------------------------------------------------------------
	// WORLD - CAMERA - LIGHTING
	//--------------------------------------------------------------------------

	// create a new world.
	world = new cBulletWorld();
	world->setEnabled(false, true);
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
	// WIDGETS
	//--------------------------------------------------------------------------

	// create a font
	font = NEW_CFONTCALIBRI20();

	// create a label to display the haptic and graphic rate of the simulation
	labelRates = new cLabel(font);
	camera->m_frontLayer->addChild(labelRates);


	//--------------------------------------------------------------------------
	// HAPTIC DEVICE
	//--------------------------------------------------------------------------

	// create a haptic device handler
	handler = new cHapticDeviceHandler();

	// get a handle to the first haptic device
	handler->getDevice(hapticDevice, 0);

	// create a tool (cursor) and insert into the world
	tool = new cToolCursor(world);
	world->addChild(tool);
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
	tool->setWaitForSmallForce(true);

	// start the haptic tool
	tool->start();
	// read the scale factor between the physical workspace of the haptic
	// device and the virtual workspace defined for the tool
	double workspaceScaleFactor = tool->getWorkspaceScaleFactor();//tool->getWorkspaceRadius() / Falcon.m_workspaceRadius;
	// hapticDeviceInfo.m_workspaceRadius----->0.04
	// stiffness properties
	// retrieve information about the current haptic device
	cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();
	double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;//Falcon.m_maxLinearStiffness / workspaceScaleFactor;
	std::cout << workspaceScaleFactor << " "<< hapticDeviceInfo.m_workspaceRadius << std::endl;
	world->setGravity(0.0, 0.0, -9.8);
	//120 is maxStiffness
	ISS.mu_max = maxStiffness * ISS.stiff_factor;
	WAVE.scaleFactor = workspaceScaleFactor;
	

	//////////////////////////////////////////////////////////////////////////
	// 3 BULLET BLOCKS
	//////////////////////////////////////////////////////////////////////////
	double size = 0.4;

	// create three objects that are added to the world
	bulletBox0 = new cBulletBox(world, size, size, size);
	world->addChild(bulletBox0);

	bulletBox1 = new cBulletBox(world, size, size, size);
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
	bulletBox1->setMass(0.05);


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
	world->setEnabled(false, true);

	bitmap = new cBitmap();
	camera->m_frontLayer->addChild(bitmap);

	//--------------------------------------------------------------------------
	// create message sender used to control delay and send message
	//--------------------------------------------------------------------------

	// create a thread which starts the main haptics rendering loop
	hapticsThread = new cThread();
	hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

	// setup callback when application exits
	atexit(close);
	sender = new Sender<hapticMessageM2S>();
	sender->Q = &forwardQ;
	sender->s = sServer;
	unsigned  uiThread1ID;
	HANDLE hth1 = (HANDLE)_beginthreadex(NULL, // security
		0,             // stack size
		ThreadX::ThreadStaticEntryPoint,// entry-point-function
		sender,           // arg list holding the "this" pointer
		0, // so we can later call ResumeThread()
		&uiThread1ID);
	//ResumeThread(hth1);
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
		std::cout << "MMT enabled" << std::endl;
		MMT.enable = true;
		MMT.Initialize();
		world->setEnabled(true, true);
		ATypeChange = AlgorithmType::AT_MMT;

		ISS.ISS_enabled = false;
		TDPA.TDPAon = false;
	}

	else if (a_key == GLFW_KEY_I) {
		std::cout << "ISS enabled" << std::endl;
		ISS.ISS_enabled = true;
		ISS.Initialize();
		ATypeChange = AlgorithmType::AT_ISS;

		MMT.enable = false;
		world->setEnabled(false, true);
		TDPA.TDPAon = false;
	}
	else if (a_key == GLFW_KEY_T) {
		std::cout << "TDPA enabled" << std::endl;
		TDPA.TDPAon = true;
		TDPA.Initialize();
		ATypeChange = AlgorithmType::AT_TDPA;

		MMT.enable = false;
		world->setEnabled(false, true);
		ISS.ISS_enabled = false;
	}
	else if (a_key == GLFW_KEY_N) {
		std::cout << "None enabled" << std::endl;

		MMT.enable = false;
		world->setEnabled(false, true);
		ATypeChange = AlgorithmType::AT_None;
		TDPA.TDPAon = false;
		ISS.ISS_enabled = false;
	}
	else if (a_key == GLFW_KEY_W) {
		std::cout << "WAVE enabled" << std::endl;
		WAVE.waveOn = true;
		WAVE.Initialize();
		ATypeChange = AlgorithmType::AT_WAVE;

		MMT.enable = false;
		world->setEnabled(false, true);		
		TDPA.TDPAon = false;
		ISS.ISS_enabled = false;
	}
	else if (a_key == GLFW_KEY_D) {
		std::cout << "Dynamic Delay :"<< !sender->dynamicDelay << std::endl;

		sender->dynamicDelay = !sender->dynamicDelay;
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
	
	cPrecisionClock clock;
	clock.reset();

	// main haptic simulation loop
	__int64 beginTime;
	QueryPerformanceCounter((LARGE_INTEGER *)&beginTime);



	while (simulationRunning)
	{
		// compute global reference frames for each object
		world->computeGlobalPositions(true);
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

		// Apply deadband on position
		DBPosition->GetCurrentSample(MasterPosition);
		DBPosition->ApplyZOHDeadband(MasterPosition, &PositionTransmitFlag);

		if (PositionTransmitFlag == true) {
		}

		// Apply deadband on velocity
		DBVelocity->GetCurrentSample(MasterVelocity);
		DBVelocity->ApplyZOHDeadband(MasterVelocity, &VelocityTransmitFlag);


		ISS.VelocityRevise(MasterVelocity);
		WAVE.getWave_l(WAVE.b, &WAVE.WV, cVector3d(MasterVelocity[0], MasterVelocity[1], MasterVelocity[2]));
		double ul[3] = { WAVE.WV.ul.x(), WAVE.WV.ul.y(), WAVE.WV.ul.z() };

		if (VelocityTransmitFlag == true) {
			memcpy(TDPA.E_trans, TDPA.E_in, 3*sizeof(double));
			memcpy(TDPA.E_in_last, TDPA.E_in, 3 * sizeof(double));
		}
		else
		{
			memcpy(TDPA.E_trans, TDPA.E_in_last, 3 * sizeof(double));
		}

#pragma region create message and send it
		/////////////////////////////////////////////////////////////////////
		// create message to send
		/////////////////////////////////////////////////////////////////////

		hapticMessageM2S msgM2S;
		for (int i = 0; i < 3; i++) {
			msgM2S.position[i] = MasterPosition[i];//modified by TDPA 
			msgM2S.linearVelocity[i] = MasterVelocity[i];//modified by TDPA 
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
		memcpy(msgM2S.energy, TDPA.E_trans, 3 * sizeof(double));//modified by TDPA 
		memcpy(msgM2S.waveVariable, ul, 3 * sizeof(double));

		__int64 curtime;
		QueryPerformanceCounter((LARGE_INTEGER *)&curtime);
		msgM2S.timestamp = curtime;
		msgM2S.ATypeChange = ATypeChange;
		ATypeChange = AlgorithmType::AT_KEEP;


		/////////////////////////////////////////////////////////////////////
		// push into send queues and prepare to send by sender thread.
		/////////////////////////////////////////////////////////////////////
		beginTime = curtime;
		//send(sServer, (char *)&msgM2S, sizeof(hapticMessageM2S), 0);
		forwardQ.push(msgM2S);
		freqCounterHaptics.signal(1);

#pragma endregion

		
#pragma region check receive queues and apply force
		/////////////////////////////////////////////////////////////////////
// check receive queues.
/////////////////////////////////////////////////////////////////////
		hapticMessageS2M msgS2M;


		int ret = recv(sServer, recData + unprocessedPtr, sizeof(recData) - unprocessedPtr, 0);
		if (ret > 0) {
			// we receive some char data and transform it to hapticMessageS2M.
			// if receive more than one hapticMessageS2M, only save the last one.
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
			
			QueryPerformanceCounter((LARGE_INTEGER *)&curtime);
			delay = ((double)(curtime - msgS2M.timestamp) / (double)cpuFreq.QuadPart) * 1000;

			//get force and energy from Slave2Master message
			memcpy(MasterForce, msgS2M.force, 3 * sizeof(double));
			memcpy(MasterTorque, msgS2M.torque, 3 * sizeof(double));
			MasterGripperForce = msgS2M.gripperForce;
			memcpy(TDPA.E_recv, msgS2M.energy, 3 * sizeof(double));
			MMT.SlavePar.parPosition = cVector3d(msgS2M.MMTParameters[0], msgS2M.MMTParameters[1], msgS2M.MMTParameters[2]);
			MMT.SlavePar.parStiffness = cVector3d(msgS2M.MMTParameters[3], msgS2M.MMTParameters[4], msgS2M.MMTParameters[5]);
			MMT.SlavePar.Flag = msgS2M.MMTParameters[6];
			
			double vl[3];
			memcpy(vl, msgS2M.waveVariable, 3 * sizeof(double));
			WAVE.WV.vl = cVector3d(vl[0], vl[1], vl[2]);

			

			//orce filter (use dot(f) and tau)
			if (FlagForceKalmanFilter)
			{
				ForceKalmanFilter.ApplyKalmanFilter(MasterForce);
				MasterForce[2] = ForceKalmanFilter.CurrentEstimation[2];
			}

			TDPA.ForceRevise(MasterVelocity, MasterForce);
			ISS.ForceRevise(MasterForce);
			WAVE.ForceRevise(MasterVelocity, &WAVE.WV, MasterForce);
			
			
			
		}
		
		if (MMT.SlavePar.Flag) {
			//update master's enviroment
			MMT.SlavePar.Flag = false;
			bulletBox1->setLocalPos(MMT.SlavePar.parPosition);
			MMT.MasterPar.parStiffness = MMT.SlavePar.parStiffness;
			
		}
		tool->computeInteractionForces();
		
		cHapticPoint* p = tool->getHapticPoint(0);
		MMT.ForceRevise(MasterForce, p->getLocalPosGoal(), p->getLocalPosProxy());

		cVector3d force=cVector3d(MasterForce[0], MasterForce[1], MasterForce[2]);
		cVector3d torque=cVector3d(MasterTorque[0], MasterTorque[1], MasterTorque[2]);
		double gripperForce = MasterGripperForce;
		tool->setDeviceLocalForce(force);
		tool->setDeviceLocalTorque(torque);
		tool->setGripperForce(gripperForce);
		tool->applyToDevice();
		/////////////////////////////////////////////////////////////////////
		// SIMULATION TIME    
		/////////////////////////////////////////////////////////////////////

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
		cStr(freqCounterHaptics.getFrequency(), 0) + " Hz    S2M delay" + cStr(delay, 3) + " ");

	// update position of label
	labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);


	/////////////////////////////////////////////////////////////////////
	// RENDER SCENE
	/////////////////////////////////////////////////////////////////////

	// update shadow maps (if any)
	world->updateShadowMaps(false, mirroredDisplay);

	char *ImgTemp = new char[864 * 270 * 8];
	int ret = recv(sServer_Image, ImgTemp, 864 * 270 * 8, 0);
	cImagePtr ImgPtr = cImage::create();
	//ImgPtr->allocate(864, 270, GL_RGBA);
	//ImgPtr->setSize();
	ImgPtr->setData((unsigned char*)ImgTemp, ret, true);
	ImgPtr->setProperties(864, 270, GL_RGBA, GL_UNSIGNED_BYTE);
	bitmap->loadFromImage(ImgPtr);
	// render world
	camera->renderView(width, height);
	ImgPtr->erase();
	// wait until all OpenGL commands are completed
	glFinish();

	// check for any OpenGL errors
	GLenum err;
	err = glGetError();
	if (err != GL_NO_ERROR) std::cout << "Error:  %s\n" << gluErrorString(err);
}
