#include "HapticComm.h"

CHapticComm::CHapticComm(void)
{
	NetworkCB = NULL;
	gSendSocket = 0;
	gListenSocket = 0;

	initNetwork();
}

CHapticComm::~CHapticComm(void)
{
	gDoListen = false;

	Sleep(300);

	closesocket(gSendSocket);
	closesocket(gListenSocket);

	WSACleanup();
}

static void UDPClientThreadWrapper(void *arg) {
	CHapticComm * pThis;
	pThis = (CHapticComm*)arg;
	pThis->UDPClientThread();
}


void CHapticComm::SetNetworkCallback(int (CALLBACK *CallbackPointer)(cVector3d*, double)) {
	NetworkCB = CallbackPointer;
}

void CHapticComm::UDPClientThread() {

#define DEFAULT_BUFFERSIZE (sizeof(double)*5)

	struct sockaddr_in gLocalAddr;

	int err;
	int flags;

	WORD wVersionRequested;
	WSADATA wsaData;

	struct sockaddr_in from;
	int fromlen = sizeof(from);

	flags = 0;

	wVersionRequested = MAKEWORD(2, 2);

	// Initialize Winsock
	err = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (err != 0) {
		wprintf(L"WSAStartup failed: %d\n", err);
		return;
	}

	if (LOBYTE(wsaData.wVersion) != 2 ||
		HIBYTE(wsaData.wVersion) != 2) {
		WSACleanup();
	}

	// local listen port settings
	gLocalAddr.sin_family = AF_INET;
	gLocalAddr.sin_port = htons(UDP_LISTENINGPORT);
	gLocalAddr.sin_addr.s_addr = INADDR_ANY;
	memset(&(gLocalAddr.sin_zero), '\0', 8);


	// create local listen socket 
	if ((gListenSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) != INVALID_SOCKET) {

		// bind it to the local host's Internet address
		if (bind(gListenSocket, (struct sockaddr *)&gLocalAddr, sizeof(gLocalAddr)) >= 0){
			char ListenBuffer[DEFAULT_BUFFERSIZE];
			int iReadBytes;
			int err;

			gDoListen = true;

			// blocking call for the client thread
			while (((iReadBytes = recvfrom(gListenSocket, ListenBuffer, sizeof(ListenBuffer), flags, (struct sockaddr *) &from, &fromlen)) >= 0) && (gDoListen)) {
				err = WSAGetLastError();

				if (iReadBytes == 4 * sizeof(double)) {
					// received force
					cVector3d newForce;
					memcpy(&newForce(0), ListenBuffer, 3 * sizeof(double));

					// corresponding timestamp
					double time_stamp;
					memcpy(&time_stamp, &ListenBuffer[sizeof(double)* 3], sizeof(double));

					// update the global force to be displayed 
					if (NetworkCB) NetworkCB(&newForce, time_stamp);

				}
				else {
					printf("Wrong datagram received.. (len:%d)\n", iReadBytes);
				}

			}

		}
		else {
			fprintf(stderr, "Could not bind socket (Port: %d)\n", UDP_LISTENINGPORT);
		}

	}
	else {
		fprintf(stderr, "Could not create socket (Port: %d)\n", UDP_LISTENINGPORT);
	}
}


void CHapticComm::initNetwork() {
	int err;
	int flags;

	WORD wVersionRequested;
	WSADATA wsaData;

	flags = 0;

	gDoListen = true;


	wVersionRequested = MAKEWORD(2, 2);

	// Initialize Winsock
	err = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (err != 0) {
		wprintf(L"WSAStartup failed: %d\n", err);
		return;
	}

	struct hostent *h;

	h = NULL;


	// ip address of the remote server we will connect to
	h = gethostbyname(REMOTE_HOST);	// get IP address of peer	
	if (h == NULL) {				// problems getting IP address?
		int dwError = WSAGetLastError();
		if (dwError != 0) {
			if (dwError == WSAHOST_NOT_FOUND) {
				printf("Host not found\n");
				return;
			}
			else if (dwError == WSANO_DATA) {
				printf("No data record found\n");
				return;
			}
			else {
				printf("Function failed with error: %ld\n", dwError);
				return;
			}
		}
		return;
	}

	// address and port settings for the remote server
	gRemoteAddr.sin_family = AF_INET;
	memcpy((char*)&gRemoteAddr.sin_addr.s_addr, h->h_addr_list[0], h->h_length);
	memset(&(gRemoteAddr.sin_zero), '\0', 8);
	gRemoteAddr.sin_port = htons(REMOTE_UDP_PORT);

	// create a send socket
	if ((gSendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == INVALID_SOCKET) {
		wprintf(L"creating socket failed: %d\n", h);
		return;
	}

	// create a new thread to listen for incoming force feedback
	gListenThread = (HANDLE)_beginthread(UDPClientThreadWrapper, 0, this);

}

// send haptic device position to the remote server
void CHapticComm::UDPSendHaptics(cVector3d* Pos) {
	char TempBuf[sizeof(double)* 3];
	if (gSendSocket>0) {
		memcpy(&TempBuf, &Pos[0], sizeof(double)* 3);
		sendto(gSendSocket, TempBuf, sizeof(TempBuf), 0, (const sockaddr*)&gRemoteAddr, sizeof(gRemoteAddr));
	}
}
