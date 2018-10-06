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
Sender *sender;
Receiver *receiver;
threadsafe_queue<hapticMessageM2S> *forceQ;
threadsafe_queue<hapticMessageS2M> *commandQ;
SOCKET sclient;

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
	//--------------------------------------------------------------------------
	// socket communication setup
	//--------------------------------------------------------------------------

	sockVersion = MAKEWORD(2, 2);
	sender = new Sender();
	receiver = new Receiver();
	forceQ = new threadsafe_queue<hapticMessageM2S>();
	commandQ = new threadsafe_queue<hapticMessageS2M>();


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
	//��IP�Ͷ˿�  
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
	{  //����ʧ�� 
		printf("connect error !");
		closesocket(sclient);
		//return 0;
	}


	sender->s = sclient;
	sender->Q = forceQ;
	receiver->s = sclient;
	receiver->Q = commandQ;
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

	// open a connection to haptic device
	hapticDevice->open();

	// calibrate device (if necessary)
	hapticDevice->calibrate();

	// retrieve information about the current haptic device
	cHapticDeviceInfo info = hapticDevice->getSpecifications();

	// if the device has a gripper, enable the gripper to simulate a user switch
	hapticDevice->setEnableGripperUserSwitch(true);


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
	hapticDevice->close();

	// delete resources
	delete hapticsThread;
	delete handler;
}

//------------------------------------------------------------------------------
int counter = 0;
void updateHaptics(void)
{
	// simulation in now running
	simulationRunning = true;
	simulationFinished = false;

	// main haptic simulation loop
	while (simulationRunning)
	{
		/////////////////////////////////////////////////////////////////////
		// READ HAPTIC DEVICE
		/////////////////////////////////////////////////////////////////////

		// read position 
		cVector3d position;
		hapticDevice->getPosition(position);

		// read orientation 
		cMatrix3d rotation;
		hapticDevice->getRotation(rotation);

		// read gripper position
		double gripperAngle;
		hapticDevice->getGripperAngleRad(gripperAngle);

		// read linear velocity 
		cVector3d linearVelocity;
		hapticDevice->getLinearVelocity(linearVelocity);

		// read angular velocity
		cVector3d angularVelocity;
		hapticDevice->getAngularVelocity(angularVelocity);

		// read gripper angular velocity
		double gripperAngularVelocity;
		hapticDevice->getGripperAngularVelocity(gripperAngularVelocity);

		// read user-switch status (button 0)
		bool button0, button1, button2, button3;
		button0 = false;
		button1 = false;
		button2 = false;
		button3 = false;

		hapticDevice->getUserSwitch(0, button0);
		hapticDevice->getUserSwitch(1, button1);
		hapticDevice->getUserSwitch(2, button2);
		hapticDevice->getUserSwitch(3, button3);


		/////////////////////////////////////////////////////////////////////
		// create message to send
		/////////////////////////////////////////////////////////////////////

		hapticMessageM2S msgM2S;
		for (int i = 0; i < 3; i++) {
			msgM2S.position[i] = position(i);
			msgM2S.linearVelocity[i] = linearVelocity(i);
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

		/////////////////////////////////////////////////////////////////////
		// push into send queues and prepare to send by sender thread.
		/////////////////////////////////////////////////////////////////////
		//counter++;
		//if (counter == 1000) {
		//	counter = 0;
		std::cout << freqCounterHaptics.getFrequency() << std::endl;
			forceQ->push(msgM2S);
		//}
		/////////////////////////////////////////////////////////////////////
		// check receive queues.
		/////////////////////////////////////////////////////////////////////
		hapticMessageS2M msgS2M;
		if (!commandQ->empty()) {
			if (commandQ->try_pop(msgS2M)) {
				//todo  we reveive a message from slave side.
				cVector3d force(msgS2M.force[0], msgS2M.force[1], msgS2M.force[2]);
				cVector3d torque(msgS2M.torque[0], msgS2M.torque[0], msgS2M.torque[0]);
				double gripperForce = msgS2M.gripperForce;
				// send computed force, torque, and gripper force to haptic device	
				hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);
			}
		}

		//Sleep(1);
		// signal frequency counter
		freqCounterHaptics.signal(1);
	}

	// exit haptics thread
	simulationFinished = true;
}

//------------------------------------------------------------------------------