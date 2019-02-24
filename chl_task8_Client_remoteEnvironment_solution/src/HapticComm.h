#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#pragma once

#include "chai3d.h"

#include <malloc.h>
#include <stdio.h> 
#include <windows.h> 
#include <process.h>
#include <fstream>
#include <iostream>
#include <string.h>

#ifdef WIN32
#include <winsock.h>         
typedef int socklen_t;
typedef char raw_type;
#endif

#pragma comment(lib, "Ws2_32.lib")

using namespace chai3d;


// name of the remote server to which you want to connect to
//#define REMOTE_HOST "tueilmt-galaga"
#define REMOTE_HOST "localhost"

#define REMOTE_UDP_PORT 80808 
#define UDP_LISTENINGPORT 80809


class CHapticComm
{
public:
	CHapticComm(void);
	~CHapticComm(void);

	// send haptic device position to the remote server
	void UDPSendHaptics(cVector3d* Pos);

	void SetNetworkCallback(int (CALLBACK *CallbackPointer)(cVector3d*, double));

	void UDPClientThread();

private:
	struct sockaddr_in gRemoteAddr;
	struct sockaddr_in gLocalAddr;

	// file descriptor for send socket
	int gSendSocket;

	// file descriptor for receive socket
	int gListenSocket;

	bool gDoListen;

	void initNetwork();

	// pointer to the NetworkCB() function
	int (CALLBACK *NetworkCB)(cVector3d*, double);

	HANDLE gListenThread;
};
