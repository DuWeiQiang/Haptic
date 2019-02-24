//===========================================================================
/*
    This file is part of the CHAI 3D visualization and haptics libraries.
    Copyright (C) 2003-2009 by CHAI 3D. All rights reserved.

    This library is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License("GPL") version 2
    as published by the Free Software Foundation.

    For using the CHAI 3D libraries with software that can not be combined
    with the GNU GPL, and for taking advantage of the additional benefits
    of our support services, please contact CHAI 3D about acquiring a
    Professional Edition License.

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   2.0.0 $Rev: 269 $

	
	//===========================================================================
		edited for the Computational Haptics Laboratory SS 2010, 
		Lehrstuhl fuer Medientechnik, Technische Universitaet Muenchen, Germany
		last revision: 25.06.2010
	//===========================================================================


 */
//===========================================================================

#define UDP_LISTENINGPORT 80808
#define UDP_REMOTEPORT 80809

//---------------------------------------------------------------------------
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>
#include <stdio.h> 
//#include <windows.h> 
#include <process.h>
#include <Winsock2.h>
#include <ws2tcpip.h>
#include <fstream>
#include <iostream>
#include <map>
#include <utility>	

//---------------------------------------------------------------------------
#include "chai3d.h"
#include "CODE.h"
//---------------------------------------------------------------------------
using namespace chai3d;

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
// DECLARED CONSTANTS
//---------------------------------------------------------------------------

using namespace  std;

// initial size (width/height) in pixels of the display window
const int WINDOW_SIZE_W         = 512;
const int WINDOW_SIZE_H         = 512;

// mouse menu options (right button)
const int OPTION_FULLSCREEN     = 1;
const int OPTION_WINDOWDISPLAY  = 2;


//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera that renders the world in a window display
cCamera* camera;

// a light source to illuminate the objects in the virtual scene
cLight *light;

// a little "chai3d" bitmap logo at the bottom of the screen
cBitmap* logo;

// width and height of the current window display
int displayW  = 0;
int displayH  = 0;

// a haptic device handler
//cHapticDeviceHandler* handler;

// a virtual tool representing the haptic device in the scene
//cGeneric3dofPointer* tool;

// radius of the tool proxy
double proxyRadius;

// a pointer the ODE object grasped by the tool
cODEGenericBody* graspObject;

// grasp position is respect to object
cVector3d graspPos;

// is grasp currently active?
bool graspActive = false;

// a small line used to display a grasp
cShapeLine* graspLine;

// maximum stiffness to be used with virtual objects in scene
double stiffnessMax;

// status of the main simulation haptics loop
bool simulationRunning = false;

// ODE world
cODEWorld* ODEWorld;

// ODE object
cODEGenericBody* ODEBody0;
cODEGenericBody* ODEBody1;
cODEGenericBody* ODEBody2;


cODEGenericBody* ODEGPlane0;
cODEGenericBody* ODEGPlane1;
cODEGenericBody* ODEGPlane2;
cODEGenericBody* ODEGPlane3;
cODEGenericBody* ODEGPlane4;
cODEGenericBody* ODEGPlane5;

// root resource path
string resourceRoot;

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

// main graphics callback
void updateGraphics(void);

// main haptics loop
void updateHaptics(void);

// main haptics loop
void UDPServerThread(void);

// creates a cube mesh
void createCube(cMesh* a_mesh, double a_size);


struct ClientStruct {         
  struct sockaddr_in gRemoteAddr;
  cVector3d Position;
  cVector3d Force;   
  cVector3d Velocity; 
  cGeneric3dofPointer* tool;
  cShapeSphere* cursor;
  double UpdateTime;
};           

// key(client address) - value (pointer to client structure) pairs
map<ULONG, ClientStruct*> Clients;

int gSendSocket;


//===========================================================================
/*
    DEMO:    ODE_cubic.cpp

    This example illustrates the use of the ODE framework for simulating
    haptic interaction with dynamic bodies. In this scene we create 3
	cubic meshes that we individually attach to ODE bodies. Haptic interactions
	are computer by using the finger-proxy haptic model and forces are
	propagated to the ODE representation.
 */
//===========================================================================

int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------

    printf ("\n");
    printf ("-----------------------------------\n");
    printf ("CHAI 3D\n");
    printf ("Demo: 40-ODE-cubic\n");
    printf ("Copyright 2003-2009\n");
    printf ("-----------------------------------\n");
    printf ("\n\n");
    printf ("Instructions:\n\n");
    printf ("- Use haptic device and user switch to manipulate cubes \n");
    printf ("\n\n");
    printf ("Keyboard Options:\n\n");
    printf ("[1] - Enable gravity\n");
    printf ("[2] - Disable gravity\n");
    printf ("[x] - Exit application\n");
    printf ("\n\n");

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);

	proxyRadius = 0.03;

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
    camera->set( cVector3d (3.0, 0.0, 0.3),    // camera position (eye)
        cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
        cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // create a light source and attach it to the camera
    light = new cLight(world);
    camera->addChild(light);                   // attach light to camera
    light->setEnabled(true);                   // enable light source
    light->setPos(cVector3d( 2.0, 0.5, 1.0));  // position the light source
    light->setDir(cVector3d(-2.0, 0.5, 1.0));  // define the direction of the light beam
    light->m_ambient.set(0.6, 0.6, 0.6);
    light->m_diffuse.set(0.8, 0.8, 0.8);
    light->m_specular.set(0.8, 0.8, 0.8);



    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------
 
    // define a maximum stiffness that can be handled by the current
    // haptic device. The value is scaled to take into account the
    // workspace scale factor
    stiffnessMax = 20;//info.m_maxForceStiffness / workspaceScaleFactor;

    // create a small white line that will be enabled every time the operator
    // grasps an object. The line indicated the connection between the
    // position of the tool and the grasp position on the object
    graspLine = new cShapeLine(cVector3d(0,0,0), cVector3d(0,0,0));
    world->addChild(graspLine);
    graspLine->m_ColorPointA.set(1.0, 1.0, 1.0);
    graspLine->m_ColorPointB.set(1.0, 1.0, 1.0);
    graspLine->setShowEnabled(false);


    //-----------------------------------------------------------------------
    // COMPOSE THE VIRTUAL SCENE
    //-----------------------------------------------------------------------

    // create an ODE world to simulate dynamic bodies
    ODEWorld = new cODEWorld(world);

    // add ODE world as a node inside world
    world->addChild(ODEWorld);

    // set some gravity
    ODEWorld->setGravity(cVector3d(0.0, 0.0, -9.81));

    // create a new ODE object that is automatically added to the ODE world
    ODEBody0 = new cODEGenericBody(ODEWorld);
    ODEBody1 = new cODEGenericBody(ODEWorld);
    ODEBody2 = new cODEGenericBody(ODEWorld);

    // create a virtual mesh  that will be used for the geometry
    // representation of the dynamic body
    cMesh* object0 = new cMesh(world);
    cMesh* object1 = new cMesh(world);
    cMesh* object2 = new cMesh(world);

    // crate a cube mesh
    double boxSize = 0.4;
    createCube(object0, boxSize);
    createCube(object1, boxSize);
    createCube(object2, boxSize);

    // define some material properties for each cube
    cMaterial mat0, mat1, mat2;
    mat0.m_ambient.set(0.8, 0.1, 0.4);
    mat0.m_diffuse.set(1.0, 0.15, 0.5);
    mat0.m_specular.set(1.0, 0.2, 0.8);
    mat0.setStiffness(0.5 * stiffnessMax);
    mat0.setDynamicFriction(0.8);
    mat0.setStaticFriction(0.8);
    object0->setMaterial(mat0);

    mat1.m_ambient.set(0.2, 0.6, 0.0);
    mat1.m_diffuse.set(0.2, 0.8, 0.0);
    mat1.m_specular.set(0.2, 1.0, 0.0);
    mat1.setStiffness(0.5 * stiffnessMax);
    mat1.setDynamicFriction(0.8);
    mat1.setStaticFriction(0.8);
    object1->setMaterial(mat1);

    mat2.m_ambient.set(0.0, 0.2, 0.6);
    mat2.m_diffuse.set(0.0, 0.2, 0.8);
    mat2.m_specular.set(0.0, 0.2, 1.0);
    mat2.setStiffness(0.5 * stiffnessMax);
    mat2.setDynamicFriction(0.8);
    mat2.setStaticFriction(0.8);
    object2->setMaterial(mat2);

    // add mesh to ODE object
    ODEBody0->setImageModel(object0);
    ODEBody1->setImageModel(object1);
    ODEBody2->setImageModel(object2);

    // create a dynamic model of the ODE object. Here we decide to use a box just like
    // the object mesh we just defined
    ODEBody0->createDynamicBox(boxSize, boxSize, boxSize);
    ODEBody1->createDynamicBox(boxSize, boxSize, boxSize, false, cVector3d(1,1,1));
    ODEBody2->createDynamicBox(boxSize, boxSize, boxSize);

		
    // define some mass properties for each cube
    ODEBody0->setMass(0.05);
    ODEBody1->setMass(0.05);
    ODEBody2->setMass(0.05);

    // set position of each cube
    cVector3d tmpvct;
    tmpvct = cVector3d(0.0,-0.6, -0.5);
    ODEBody0->setPosition(tmpvct);
    tmpvct = cVector3d(0.0, 0.6, -0.5);
    ODEBody1->setPosition(tmpvct);
    tmpvct = cVector3d(0.0, 0.0, -0.5);
    ODEBody2->setPosition(tmpvct);

    // rotate central cube of 45 degrees (just to show hoe this works!)
    cMatrix3d rot;
    rot.identity();
    rot.rotate(cVector3d(0,0,1), cDegToRad(45));
    ODEBody0->setRotation(rot);

    // we create 6 static walls to contains the 3 cubes within a limited workspace
    ODEGPlane0 = new cODEGenericBody(ODEWorld);
    ODEGPlane1 = new cODEGenericBody(ODEWorld);
    ODEGPlane2 = new cODEGenericBody(ODEWorld);
    ODEGPlane3 = new cODEGenericBody(ODEWorld);
    ODEGPlane4 = new cODEGenericBody(ODEWorld);
    ODEGPlane5 = new cODEGenericBody(ODEWorld);

    double size = 1.0;
    ODEGPlane0->createStaticPlane(cVector3d(0.0, 0.0,  2.0 *size), cVector3d(0.0, 0.0 ,-1.0));
    ODEGPlane1->createStaticPlane(cVector3d(0.0, 0.0, -size), cVector3d(0.0, 0.0 , 1.0));
    ODEGPlane2->createStaticPlane(cVector3d(0.0,  size, 0.0), cVector3d(0.0,-1.0, 0.0));
    ODEGPlane3->createStaticPlane(cVector3d(0.0, -size, 0.0), cVector3d(0.0, 1.0, 0.0));
    ODEGPlane4->createStaticPlane(cVector3d( size, 0.0, 0.0), cVector3d(-1.0,0.0, 0.0));
    ODEGPlane5->createStaticPlane(cVector3d(-0.8 * size, 0.0, 0.0), cVector3d( 1.0,0.0, 0.0));

    //////////////////////////////////////////////////////////////////////////
    // Create some reflexion
    //////////////////////////////////////////////////////////////////////////

    // we create an intermediate node to which we will attach
    // a copy of the object located inside the world
    cGenericObject* reflexion = new cGenericObject();

    // set this object as a ghost node so that no haptic interactions or
    // collision detecting take place within the child nodes added to the
    // reflexion node.
    reflexion->setAsGhost(true);

    // add reflexion node to world
    world->addChild(reflexion);

    // turn off culling on each object (objects now get rendered on both sides)
    object0->setUseCulling(false, true);
    object1->setUseCulling(false, true);
    object2->setUseCulling(false, true);

    // create a symmetry rotation matrix (z-plane)
    cMatrix3d rotRefexion;
    rotRefexion.set(1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, -1.0);
    reflexion->setRot(rotRefexion);
    reflexion->setPos(0.0, 0.0, -2.005);

    // add objects to the world
    reflexion->addChild(ODEWorld);
  //  reflexion->addChild(tool);

    //////////////////////////////////////////////////////////////////////////
    // Create a Ground
    //////////////////////////////////////////////////////////////////////////

    // create mesh to model ground surface
    cMesh* ground = new cMesh(world);
    world->addChild(ground);

    // create 4 vertices (one at each corner)
    double groundSize = 2.0;
    int vertices0 = ground->newVertex(-groundSize, -groundSize, 0.0);
    int vertices1 = ground->newVertex( groundSize, -groundSize, 0.0);
    int vertices2 = ground->newVertex( groundSize,  groundSize, 0.0);
    int vertices3 = ground->newVertex(-groundSize,  groundSize, 0.0);

    // compose surface with 2 triangles
    ground->newTriangle(vertices0, vertices1, vertices2);
    ground->newTriangle(vertices0, vertices2, vertices3);

    // compute surface normals
    ground->computeAllNormals();

    // position ground at the right level
    ground->setPos(0.0, 0.0, -1.0);

    // define some material properties and apply to mesh
    cMaterial matGround;
    matGround.setStiffness(stiffnessMax);
    matGround.setDynamicFriction(0.7);
    matGround.setStaticFriction(1.0);
    matGround.m_ambient.set(0.0, 0.0, 0.0);
    matGround.m_diffuse.set(0.0, 0.0, 0.0);
    matGround.m_specular.set(0.0, 0.0, 0.0);
    ground->setMaterial(matGround);

    // enable and set transparency level of ground
    ground->setTransparencyLevel(0.7);
    ground->setUseTransparency(true);


    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

    // initialize GLUT
    glutInit(&argc, argv);

    // retrieve the resolution of the computer display and estimate the position
    // of the GLUT window so that it is located at the center of the screen
    int screenW = glutGet(GLUT_SCREEN_WIDTH);
    int screenH = glutGet(GLUT_SCREEN_HEIGHT);
    int windowPosX = (screenW - WINDOW_SIZE_W) / 2;
    int windowPosY = (screenH - WINDOW_SIZE_H) / 2;

    // initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(WINDOW_SIZE_W, WINDOW_SIZE_H);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    glutCreateWindow(argv[0]);
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("Haptic UDP Server");

    // create a mouse menu (right button)
    glutCreateMenu(menuSelect);
    glutAddMenuEntry("full screen", OPTION_FULLSCREEN);
    glutAddMenuEntry("window display", OPTION_WINDOWDISPLAY);
    glutAttachMenu(GLUT_RIGHT_BUTTON);


    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // simulation in now running
    simulationRunning = true;

    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->set(updateHaptics, CHAI_THREAD_PRIORITY_HAPTICS);

    cThread* serverThread = new cThread();
    serverThread->set(UDPServerThread, CHAI_THREAD_PRIORITY_GRAPHICS);

    // start the main graphics rendering loop
    glutMainLoop();

    // close everything
    close();

    // exit
    return (0);
}

//---------------------------------------------------------------------------

void resizeWindow(int w, int h)
{
    // update the size of the viewport
    displayW = w;
    displayH = h;
    glViewport(0, 0, displayW, displayH);
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
        // enable gravity
        ODEWorld->setGravity(cVector3d(0.0, 0.0, -9.81));
    }

    // option 2:
    if (key == '2')
    {
        // disable gravity
        ODEWorld->setGravity(cVector3d(0.0, 0.0, 0.0));
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

    // close haptic device
  //  tool->stop();
    }

//---------------------------------------------------------------------------

void updateGraphics(void)
{
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

void SendUDPForce (ClientStruct* cClient) {
	char TempBuf[sizeof(double)*3];

	cClient->gRemoteAddr.sin_port = htons(UDP_REMOTEPORT);  

	if (gSendSocket>0) {

		// copy the cVector3d force calculated for this client to the temporary buffer
		memcpy(&TempBuf, &cClient->Force[0], sizeof(double)*3);

		// send that force to the remote client
		sendto(gSendSocket, TempBuf, sizeof(TempBuf), 0,  (const sockaddr*) &cClient->gRemoteAddr, sizeof(sockaddr_in));
	}
}

void updateHaptics(void)
{
	ClientStruct* cClient;
	map<ULONG,ClientStruct*>::iterator it;

    // simulation clock
    cPrecisionClock simClock;
    simClock.start(true);

    // main haptic simulation loop
    while(simulationRunning)
    {
        // compute global reference frames for each object
        world->computeGlobalPositions(true);

		
		// run through all the connected clients
		for ( it=Clients.begin() ; it != Clients.end(); it++ ) {
			
			//mapped value = second, key value = first
			cClient = (*it).second; 

			// the tool belonging to the current client
			cGeneric3dofPointer* tool = cClient->tool;

			// read the remote "device" position
			tool->m_deviceGlobalPos = tool->m_workspaceScaleFactor * cClient->Position;			

			// compute interaction forces for this tool
			tool->computeInteractionForces();

			// update the position of the cursor belonging to the client
			cClient->cursor->setPos(tool->getProxyGlobalPos());
			
			cGenericObject* object = tool->m_proxyPointForceModel->m_contactPoint0->m_object;
            if (object != NULL)
            {
                // check if object is attached to an external ODE parent
                cGenericType* externalParent = object->getExternalParent();
                cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(externalParent);
                if (ODEobject != NULL)
                {
                    // get position of tool
                    cVector3d pos = tool->m_proxyPointForceModel->m_contactPoint0->m_globalPos;

                    // retrieve the haptic interaction force being applied to the tool
                    cVector3d force = tool->m_lastComputedGlobalForce;

                    // apply haptic force to ODE object
                    cVector3d tmpfrc = cNegate(force);
                    ODEobject->addGlobalForceAtGlobalPos(tmpfrc, pos);
				}
			}

			cClient->Force = tool->m_lastComputedGlobalForce;
			SendUDPForce (cClient);
		}

        // retrieve simulation time and compute next interval
        double time = simClock.getCurrentTimeSeconds();
        double nextSimInterval = cClamp(time, 0.00001, 0.001);


        // reset clock
        simClock.reset();
        simClock.start();

        // update simulation
        ODEWorld->updateDynamics(nextSimInterval);
    }

    // exit haptics thread
    simulationFinished = true;
}

void UDPServerThread() {

    #define DEFAULT_BUFFERSIZE (sizeof(double)*6)

	struct sockaddr_in gLocalAddr;	 
	int gListenSocket;
	int err;
	int flags;

	WORD wVersionRequested;
	WSADATA wsaData;

	// which address are we receiving the current packet from?
	struct sockaddr_in from;
	int fromlen = sizeof(from);

	// flags for the recvFrom() function
	flags = 0;

	wVersionRequested = MAKEWORD( 2, 2 );
 
    // Initialize Winsock
    err = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (err != 0) {
        wprintf(L"WSAStartup failed: %d\n", err);
        return;
    }

 
	if ( LOBYTE( wsaData.wVersion ) != 2 ||
			HIBYTE( wsaData.wVersion ) != 2 ) {
		WSACleanup( );
	}

	// local socket specifications
	// Both the sin_port field and the sin_addr field are in network byte order.
	gLocalAddr.sin_family = AF_INET;	// AF_INET for the Internet family
    gLocalAddr.sin_port = htons(UDP_LISTENINGPORT);	// The sin_port field specifies UDP port number of the address
    gLocalAddr.sin_addr.s_addr = INADDR_ANY;	// sin_addr field specifies the Internet address
	memset(&(gLocalAddr.sin_zero), '\0', 8);	// sin_zero field must always be zero

	
	// The socket() function shall create an unbound socket in a communications domain, and return a file descriptor 
	// that can be used in later function calls that operate on sockets

	// the "send" socket
	if ((gSendSocket=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==INVALID_SOCKET ) {
		wprintf(L"creating socket failed\n");
		return;
	}	


	// the "listen" socket
	if ((gListenSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) != INVALID_SOCKET) {
		if ( bind(gListenSocket, (struct sockaddr *)&gLocalAddr, sizeof(gLocalAddr)) >= 0) {
			char ListenBuffer[DEFAULT_BUFFERSIZE];
			int iReadBytes;
			int err,p;
			char ip[16];
			map<ULONG,ClientStruct*>::iterator it;
			ClientStruct* cClient;

			cPrecisionClock netClock;
			netClock.start(true);
			

			// ch lab
			// notes:
			// The recvfrom() function permits the application to retrieve the source address of received data.
			// This is helpful in our case since to distinguish between connected clients as we operate in UDP (connectionless) mode.
			// The sending address is stored in the second last argument.
			while ( (iReadBytes = recvfrom(gListenSocket, ListenBuffer, sizeof(ListenBuffer), flags, (struct sockaddr *) &from, &fromlen))>=0 ) {
				err = WSAGetLastError();

				sprintf(ip, "%d.%d.%d.%d",
					from.sin_addr.S_un.S_un_b.s_b1,
					from.sin_addr.S_un.S_un_b.s_b2,
					from.sin_addr.S_un.S_un_b.s_b3,
					from.sin_addr.S_un.S_un_b.s_b4);

				// do we already have the client corresponding to this address in our list?
				it = Clients.find(from.sin_addr.S_un.S_addr);

				// no, we don't - so we add it to our list
				if (it==Clients.end()) { 
				    char hostname[NI_MAXHOST];
					char servInfo[NI_MAXSERV];

					cClient = new ClientStruct;
					cClient->gRemoteAddr = from;
					cClient->Force *= 0;
					cClient->Position *= 0;
					cClient->Velocity *= 0;	
					cClient->UpdateTime = netClock.getCurrentTimeSeconds();


					// create a 3D tool and add it to the world
					cGeneric3dofPointer* tool = new cGeneric3dofPointer(world);
					tool->setShowEnabled(true);

					world->addChild(tool);
					tool->setWorkspaceRadius(1.3);

					// define a radius for the tool (graphical display)
					tool->setRadius(proxyRadius);

					// hide the device sphere. only show proxy.
					tool->m_deviceSphere->setShowEnabled(true);
					tool->m_proxySphere->setShowEnabled(false);
					tool->setWorkspaceRadius(1.3);
					tool->m_workspaceScaleFactor = 12.37;	// workspace scale factor for the phantom omni device

					// set the physical readius of the proxy.
					tool->m_proxyPointForceModel->setProxyRadius(proxyRadius);
					tool->m_proxyPointForceModel->m_collisionSettings.m_checkBothSidesOfTriangles = false;
					tool->m_proxyPointForceModel->m_useDynamicProxy = true;

					// ajust the color of the tool
					tool->m_materialProxy = tool->m_materialProxyButtonPressed;
					cClient->tool = tool;

					cShapeSphere* newCursor = new cShapeSphere(proxyRadius);
					world->addChild(newCursor);
					cClient->cursor = newCursor;

					// translate a socket address to a node name and service location
				    err = getnameinfo((struct sockaddr *) &from,
                           sizeof (struct sockaddr),
                           hostname,
                           NI_MAXHOST, servInfo, NI_MAXSERV, NI_NUMERICSERV);

					// label for the client tool in the shared VE
					cLabel* newLabel = new cLabel();
					newLabel->m_string.assign("  ");
					newLabel->m_string.append(hostname);
					newLabel->setPos(0.00, 0.02, 0.00);
					newLabel->m_fontColor.set(1.0, 1.0, 1.0);
					newCursor->addChild(newLabel);

					// insert a new client as defined above into our map list
					Clients.insert(std::pair<ULONG,ClientStruct*>(from.sin_addr.S_un.S_addr,cClient));
					printf("Added Client: %s\n", hostname );
				} else {
					// yes, we do - so we update this client's data with that from the packet that we just received (see immediately below)
					cClient = (*it).second;
				}

				if (iReadBytes == 3*sizeof(double)) {

					// we are receiving a cVector3d position correctly (we just check for correctness of size)

					cVector3d newPosition;
					memcpy(&newPosition[0], ListenBuffer, 3*sizeof(double));
					//memcpy(&newVelocity[0], &ListenBuffer[3*sizeof(double)], 3*sizeof(double));

					double now = netClock.getCurrentTimeSeconds();
					//cClient->Velocity = 0;//newVelocity;//(cClient->Position - newPosition)/cClamp(now-cClient->UpdateTime,0.00001, 0.001);
					cClient->Position = newPosition;
					cClient->UpdateTime = now;	
				
				} else {
					printf("Wrong datagram received.. (len:%d)\n", iReadBytes, ip, err );
				}				
				
			} // while loop 

		} else {
			fprintf(stderr, "Could not bind socket (Port: %d)\n", UDP_LISTENINGPORT);
		}

	} else {
		fprintf(stderr, "Could not create socket (Port: %d)\n", UDP_LISTENINGPORT);
	}
}


//---------------------------------------------------------------------------

void createCube(cMesh* a_mesh, double a_size)
{
    const double HALFSIZE = a_size / 2.0;
    int vertices [6][6];

    // face -x
    vertices[0][0] = a_mesh->newVertex(-HALFSIZE,  HALFSIZE, -HALFSIZE);
    vertices[0][1] = a_mesh->newVertex(-HALFSIZE, -HALFSIZE, -HALFSIZE);
    vertices[0][2] = a_mesh->newVertex(-HALFSIZE, -HALFSIZE,  HALFSIZE);
    vertices[0][3] = a_mesh->newVertex(-HALFSIZE,  HALFSIZE,  HALFSIZE);

    // face +x
    vertices[1][0] = a_mesh->newVertex( HALFSIZE, -HALFSIZE, -HALFSIZE);
    vertices[1][1] = a_mesh->newVertex( HALFSIZE,  HALFSIZE, -HALFSIZE);
    vertices[1][2] = a_mesh->newVertex( HALFSIZE,  HALFSIZE,  HALFSIZE);
    vertices[1][3] = a_mesh->newVertex( HALFSIZE, -HALFSIZE,  HALFSIZE);

    // face -y
    vertices[2][0] = a_mesh->newVertex(-HALFSIZE,  -HALFSIZE, -HALFSIZE);
    vertices[2][1] = a_mesh->newVertex( HALFSIZE,  -HALFSIZE, -HALFSIZE);
    vertices[2][2] = a_mesh->newVertex( HALFSIZE,  -HALFSIZE,  HALFSIZE);
    vertices[2][3] = a_mesh->newVertex(-HALFSIZE,  -HALFSIZE,  HALFSIZE);

    // face +y
    vertices[3][0] = a_mesh->newVertex( HALFSIZE,   HALFSIZE, -HALFSIZE);
    vertices[3][1] = a_mesh->newVertex(-HALFSIZE,   HALFSIZE, -HALFSIZE);
    vertices[3][2] = a_mesh->newVertex(-HALFSIZE,   HALFSIZE,  HALFSIZE);
    vertices[3][3] = a_mesh->newVertex( HALFSIZE,   HALFSIZE,  HALFSIZE);

    // face -z
    vertices[4][0] = a_mesh->newVertex(-HALFSIZE,  -HALFSIZE, -HALFSIZE);
    vertices[4][1] = a_mesh->newVertex(-HALFSIZE,   HALFSIZE, -HALFSIZE);
    vertices[4][2] = a_mesh->newVertex( HALFSIZE,   HALFSIZE, -HALFSIZE);
    vertices[4][3] = a_mesh->newVertex( HALFSIZE,  -HALFSIZE, -HALFSIZE);

    // face +z
    vertices[5][0] = a_mesh->newVertex( HALFSIZE,  -HALFSIZE,  HALFSIZE);
    vertices[5][1] = a_mesh->newVertex( HALFSIZE,   HALFSIZE,  HALFSIZE);
    vertices[5][2] = a_mesh->newVertex(-HALFSIZE,   HALFSIZE,  HALFSIZE);
    vertices[5][3] = a_mesh->newVertex(-HALFSIZE,  -HALFSIZE,  HALFSIZE);

    // create triangles
    for (int i=0; i<6; i++)
    {
    a_mesh->newTriangle(vertices[i][0], vertices[i][1], vertices[i][2]);
    a_mesh->newTriangle(vertices[i][0], vertices[i][2], vertices[i][3]);
    }

    // set material properties to light gray
    a_mesh->m_material.m_ambient.set(0.5f, 0.5f, 0.5f, 1.0f);
    a_mesh->m_material.m_diffuse.set(0.7f, 0.7f, 0.7f, 1.0f);
    a_mesh->m_material.m_specular.set(1.0f, 1.0f, 1.0f, 1.0f);
    a_mesh->m_material.m_emission.set(0.0f, 0.0f, 0.0f, 1.0f);

    // compute normals
    a_mesh->computeAllNormals();

    // compute collision detection algorithm
    a_mesh->createAABBCollisionDetector(1.01 * proxyRadius, true, false);
}
