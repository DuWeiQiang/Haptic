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
\version   3.1.0 $Rev: 1869 $
*/
//===========================================================================
/*
//===========================================================================
edited for the Computational Haptics Laboratory SS 2010,
Lehrstuhl fuer Medientechnik, Technische Universitaet Muenchen, Germany
last revision : 02.07.2010
//===========================================================================
*/
//===========================================================================

//---------------------------------------------------------------------------
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
#include "src/HapticComm.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
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

//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())

//---------------------------------------------------------------------------
// DECLARED CONSTANTS
//---------------------------------------------------------------------------

// initial size (width/height) in pixels of the display window
const int WINDOW_SIZE_W = 600;
const int WINDOW_SIZE_H = 600;

// mouse menu options (right button)
const int OPTION_FULLSCREEN = 1;
const int OPTION_WINDOWDISPLAY = 2;

// maximum number of haptic devices supported in this demo
const int MAX_DEVICES = 1;//8;


//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera that renders the world in a window display
cCamera* camera;

// a light source to illuminate the objects in the virtual scene
cDirectionalLight *light;

// a little "chai3d" bitmap logo at the bottom of the screen
cBitmap* logo;

// width and height of the current window display
int displayW = 0;
int displayH = 0;

// a haptic device handler
cHapticDeviceHandler* handler;

// a table containing pointers to all haptic devices detected on this computer
cGenericHapticDevicePtr hapticDevices[MAX_DEVICES];

// a table containing pointers to label which display the position of
// each haptic device
cLabel* labels[MAX_DEVICES];
cLabel* deviceLabels[MAX_DEVICES];
cGenericObject* rootLabels;

// number of haptic devices detected
int numHapticDevices = 0;

// table containing a list of 3D cursors for each haptic device
cShapeSphere* cursors[MAX_DEVICES];

// table containing a list of lines to display velocity
cShapeLine* velocityVectors[MAX_DEVICES];

// material properties used to render the color of the cursors
cMaterial matCursorButtonON;
cMaterial matCursorButtonOFF;

// status of the main simulation haptics loop
bool simulationRunning = false;

// root resource path
string resourceRoot;

// damping mode ON/OFF
bool useDamping = false;

// force field mode ON/OFF
bool useForceField = true;

// has exited haptics simulation thread
bool simulationFinished = false;

//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a keyboard key is pressed
void keySelect(unsigned char key, int x, int y);

// callback when the right mouse button is pressed to select a menu item
void menuSelect(int value);

// function called before exiting the application
void close(void);

// callback of GLUT timer
void graphicsTimer(int data);

// main graphics callback
void updateGraphics(void);

// main haptics loop
void updateHaptics(void);

CHapticComm HapticNetwork;
cVector3d globalForce;

// some time information necessary for the linear predictor
double timestamp;
cLabel* labelRates;
cFrequencyCounter freqCounterGraphics;
int CALLBACK NetworkFeedbackCB(cVector3d* Force, double time_stamp);



// linear predictor
cVector3d ch_perceptualLinearPred(const cVector3d& rawForce, bool& received, double& update_timestamp);
unsigned int initial_samples_marker = 0;

// update received flag
bool update_received = false;


//===========================================================================
/*
DEMO:    device.cpp

This application illustrates the use of the haptic device handler
"cHapticDevicehandler" to access all of the haptic devices
"cGenericHapticDevice" connected to the computer.

In this example the application opens an OpenGL window and displays a
3D cursor for each device. Each cursor (sphere + reference frame)
represents the position and orientation of its respective device.
If the operator presses the device user button (if available), the color
of the cursor changes accordingly.

In the main haptics loop function  "updateHaptics()" , the position,
orientation and user switch status of each device are retrieved at
each simulation iteration. The information is then used to update the
position, orientation and color of the cursor. A force is then commanded
to the haptic device to attract the end-effector towards the device origin.
*/
//===========================================================================

int main(int argc, char* argv[])
{
	//-----------------------------------------------------------------------
	// INITIALIZATION
	//-----------------------------------------------------------------------

	printf("\n");
	printf("-----------------------------------\n");
	printf("CHAI 3D\n");
	printf("CH lab - Haptic UDP Client\n");
	printf("Copyright 2003-2009\n");
	printf("-----------------------------------\n");
	printf("\n\n");
	printf("Keyboard Options:\n\n");
	printf("[1] - Render attraction force\n");
	printf("[2] - Render viscous environment\n");
	printf("[x] - Exit application\n");
	printf("\n\n");

	// parse first arg to try and locate resources
	resourceRoot = string(argv[0]).substr(0, string(argv[0]).find_last_of("/\\") + 1);

	HapticNetwork.SetNetworkCallback(&NetworkFeedbackCB);

	globalForce *= 0;


	//--------------------------------------------------------------------------
	// OPEN GL - WINDOW DISPLAY
	//--------------------------------------------------------------------------

	// initialize GLUT
	glutInit(&argc, argv);

	// retrieve  resolution of computer display and position window accordingly
	int screenW = glutGet(GLUT_SCREEN_WIDTH);
	int screenH = glutGet(GLUT_SCREEN_HEIGHT);
	int windowPosX = (screenW - WINDOW_SIZE_W) / 2;
	int windowPosY = (screenH - WINDOW_SIZE_H) / 2;

	// initialize the OpenGL GLUT window
	glutInitWindowPosition(windowPosX, windowPosY);
	glutInitWindowSize(WINDOW_SIZE_W, WINDOW_SIZE_H);

	if (stereoMode == C_STEREO_ACTIVE)
		glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO);
	else
		glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

	// create display context and initialize GLEW library
	glutCreateWindow(argv[0]);

#ifdef GLEW_VERSION
	// initialize GLEW
	glewInit();
#endif

	// setup GLUT options
	glutDisplayFunc(updateGraphics);
	glutKeyboardFunc(keySelect);
	glutReshapeFunc(resizeWindow);
	glutSetWindowTitle("CH lab - Haptic UDP Client");

	// set fullscreen mode
	if (fullscreen)
	{
		glutFullScreen();
	}

	//-----------------------------------------------------------------------
	// 3D - SCENEGRAPH
	//-----------------------------------------------------------------------

	// create a new world.
	world = new cWorld();

	// set the background color of the environment
	// the color is defined by its (R,G,B) components.
	world->setBackgroundColor(0.0, 0.0, 0.0);

	// create a camera and insert it into the virtual world
	camera = new cCamera(world);
	world->addChild(camera);

	// position and oriente the camera
	camera->set(cVector3d(0.5, 0.0, 0.0),    // camera position (eye)
		cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
		cVector3d(0.0, 0.0, 1.0));   // direction of the "up" vector

	// set the near and far clipping planes of the camera
	// anything in front/behind these clipping planes will not be rendered
	camera->setClippingPlanes(0.01, 10.0);

	// create a light source and attach it to the camera
	light = new cDirectionalLight(world);
	camera->addChild(light);                   // attach light to camera
	light->setEnabled(true);                   // enable light source
	light->setLocalPos(cVector3d(2.0, 0.5, 1.0));  // position the light source
	light->setDir(cVector3d(-2.0, 0.5, 1.0));  // define the direction of the light beam


	//-----------------------------------------------------------------------
	// HAPTIC DEVICES / TOOLS
	//-----------------------------------------------------------------------

	// create a haptic device handler
	handler = new cHapticDeviceHandler();

	// read the number of haptic devices currently connected to the computer
	numHapticDevices = handler->getNumDevices();

	// limit the number of devices to MAX_DEVICES
	numHapticDevices = cMin(numHapticDevices, MAX_DEVICES);

	// create a node on which we will attach small labels that display the
	// position of each haptic device
	rootLabels = new cGenericObject();
	camera->m_frontLayer->addChild(rootLabels);

	// create a font
	cFontPtr font = NEW_CFONTCALIBRI20();
	// create a small label as title
	cLabel* titleLabel = new cLabel(font);
	rootLabels->addChild(titleLabel);

	// define its position, color and string message
	titleLabel->setLocalPos(0, 30, 0);
	titleLabel->m_fontColor.set(1.0, 1.0, 1.0);
	titleLabel->setText("Haptic Device Pos [mm]:");
	// create a label to display the haptic and graphic rate of the simulation
	labelRates = new cLabel(font);
	camera->m_frontLayer->addChild(labelRates);
	// for each available haptic device, create a 3D cursor
	// and a small line to show velocity
	int i = 0;
	while (i < numHapticDevices)
	{

		// get a handle to the next haptic device
		// a pointer to the current haptic device
		cGenericHapticDevicePtr newHapticDevice;
		handler->getDevice(newHapticDevice, i);

		// open connection to haptic device
		newHapticDevice->open();

		// initialize haptic device
		//newHapticDevice->initialize();
		newHapticDevice->calibrate(true);
		// store the handle in the haptic device table
		hapticDevices[i] = newHapticDevice;

		// retrieve information about the current haptic device
		cHapticDeviceInfo info = newHapticDevice->getSpecifications();

		// create a cursor by setting its radius
		cShapeSphere* newCursor = new cShapeSphere(0.01);

		// add cursor to the world
		world->addChild(newCursor);

		// add cursor to the cursor table
		cursors[i] = newCursor;
		
		// create a small line to illustrate velocity
		cShapeLine* newLine = new cShapeLine(cVector3d(0, 0, 0), cVector3d(0, 0, 0));
		velocityVectors[i] = newLine;

		// add line to the world
		world->addChild(newLine);

		// create a string that concatenates the device number and model name.
		string strID = cStr(i);
		string strDevice = "#" + strID + " - " + info.m_modelName;

		//// attach a small label next to the cursor to indicate device information
		cFontPtr fontlabel = NEW_CFONTCALIBRI20();
		cLabel* newLabel = new cLabel(fontlabel);
		camera->m_frontLayer->addChild(newLabel);
		newLabel->m_fontColor.set(1.0, 1.0, 1.0);
		newLabel->setText(strDevice);
		deviceLabels[i] = newLabel;
		

		// if the device provided orientation sensing (stylus), a reference
		// frame is displayed
		if (info.m_sensedRotation == true)
		{
			// display a reference frame
			newCursor->setShowFrame(true);

			// set the size of the reference frame
			newCursor->setFrameSize(0.05, 0.05);
		}

		// create a small label to indicate the position of the device
		cLabel* newPosLabel = new cLabel(fontlabel);
		rootLabels->addChild(newPosLabel);
		newPosLabel->setLocalPos(0, -20 * i, 0);
		newPosLabel->m_fontColor.set(0.6, 0.6, 0.6);
		labels[i] = newPosLabel;

		// increment counter
		i++;
	}


	// here we define the material properties of the cursor when the
	// user button of the device end-effector is engaged (ON) or released (OFF)

	// a light orange material color
	matCursorButtonOFF.m_ambient.set(0.5, 0.2, 0.0);
	matCursorButtonOFF.m_diffuse.set(1.0, 0.5, 0.0);
	matCursorButtonOFF.m_specular.set(1.0, 1.0, 1.0);

	// a blue material color
	matCursorButtonON.m_ambient.set(0.1, 0.1, 0.4);
	matCursorButtonON.m_diffuse.set(0.3, 0.3, 0.8);
	matCursorButtonON.m_specular.set(1.0, 1.0, 1.0);


	//-----------------------------------------------------------------------
	// START SIMULATION
	//-----------------------------------------------------------------------

	// simulation in now running
	simulationRunning = true;

	// create a thread which starts the main haptics rendering loop
	cThread* hapticsThread = new cThread();
	hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
	// setup callback when application exits
	atexit(close);

	// start the main graphics rendering loop
	glutTimerFunc(50, graphicsTimer, 0);
	glutMainLoop();

	// exit
	return (0);
}



int CALLBACK NetworkFeedbackCB(cVector3d* Force, double time_stamp) {
	freqCounterGraphics.signal(1);
	globalForce = *Force;
	timestamp = time_stamp;
	update_received = true;
	return 0;
}

//---------------------------------------------------------------------------

void resizeWindow(int w, int h)
{
	// update the size of the viewport
	displayW = w;
	displayH = h;
	glViewport(0, 0, displayW, displayH);

	// update position of labels
	rootLabels->setLocalPos(10, displayH - 70, 0);
}

//---------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
	// escape key
	if ((key == 27) || (key == 'x'))
	{
		// close everything
		close();

		// exit application
		exit(0);
	}

	// option 1:
	if (key == '1')
	{
		useForceField = !useForceField;
		if (useForceField)
		{
			printf("- Enable force field\n");
		}
		else
		{
			printf("- Disable force field\n");
		}
	}

	// option 2:
	if (key == '2')
	{
		useDamping = !useDamping;
		if (useDamping)
		{
			printf("- Enable viscosity\n");
		}
		else
		{
			printf("- Disable viscosity\n");
		}
	}


}

//---------------------------------------------------------------------------

void menuSelect(int value)
{
	switch (value)
	{
		// enable full screen display
	case OPTION_FULLSCREEN:
		glutFullScreen();
		break;

		// reshape window to original size
	case OPTION_WINDOWDISPLAY:
		glutReshapeWindow(WINDOW_SIZE_W, WINDOW_SIZE_H);
		break;
	}
}

//---------------------------------------------------------------------------

void close(void)
{
	// stop the simulation
	simulationRunning = false;

	// wait for graphics and haptics loops to terminate
	while (!simulationFinished) { cSleepMs(100); }

	// close all haptic devices
	int i = 0;
	while (i < numHapticDevices)
	{
		hapticDevices[i]->close();
		i++;
	}
}

//---------------------------------------------------------------------------
void graphicsTimer(int data)
{
	if (simulationRunning)
	{
		glutPostRedisplay();
	}

	glutTimerFunc(50, graphicsTimer, 0);
}
//---------------------------------------------------------------------------

void updateGraphics(void)
{
	// update content of position label
	for (int i = 0; i<numHapticDevices; i++)
	{
		// read position of device an convert into millimeters
		cVector3d pos;
		hapticDevices[i]->getPosition(pos);
		pos.mul(1000);

		// create a string that concatenates the device number and its position.
		string strID = cStr(i);
		string strLabel = "#" + strID + "  x: ";

		string xpos = cStr(pos.x(), 2);
		strLabel = strLabel + xpos + "   y: ";
		string ypos = cStr(pos.y(), 2);
		strLabel = strLabel + ypos + "  z: ";
		string zpos = cStr(pos.z(), 2);
		strLabel = strLabel + zpos;

		labels[i]->setText(strLabel);
		deviceLabels[i]->setLocalPos(pos.x() + (displayW - 100) / 2, pos.y() + (displayH - 100) / 2, pos.z());
	}
	labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0));
	labelRates->setLocalPos(0, 0);

	

	// render world
	camera->renderView(displayW, displayH);

	// Swap buffers
	glutSwapBuffers();

	// check for any OpenGL errors
	GLenum err;
	err = glGetError();
	if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));

	// inform the GLUT window to call updateGraphics again (next frame)
	if (simulationRunning)
	{
		glutPostRedisplay();
	}
}


//---------------------------------------------------------------------------

void updateHaptics(void)
{
	// main haptic simulation loop
	while (simulationRunning)
	{
		// for each device
		int i = 0;
		while (i < numHapticDevices)
		{
			// read position of haptic device
			cVector3d newPosition;
			hapticDevices[i]->getPosition(newPosition);

			HapticNetwork.UDPSendHaptics(&newPosition);

			// read orientation of haptic device
			cMatrix3d newRotation;
			hapticDevices[i]->getRotation(newRotation);

			// update position and orientation of cursor
			cursors[i]->setLocalPos(newPosition);
			cursors[i]->setLocalRot(newRotation);

			// read linear velocity from device
			cVector3d linearVelocity;
			hapticDevices[i]->getLinearVelocity(linearVelocity);


			// update arrow
			velocityVectors[i]->m_pointA = newPosition;
			velocityVectors[i]->m_pointB = cAdd(newPosition, linearVelocity);

			// read user button status
			bool buttonStatus;
			hapticDevices[i]->getUserSwitch(0, buttonStatus);

			// adjustthe  color of the cursor according to the status of
			// the user switch (ON = TRUE / OFF = FALSE)
			if (buttonStatus)
			{
				cursors[i]->setMaterial(matCursorButtonON);
			}
			else
			{
				cursors[i]->setMaterial(matCursorButtonOFF);
			}



			// hold-last-sample deadband reconstruction 	
			// send computed force to haptic device

			hapticDevices[i]->setForce(globalForce);


			// increment counter
			i++;
		}
	}

	// exit haptics thread
	simulationFinished = true;
}


