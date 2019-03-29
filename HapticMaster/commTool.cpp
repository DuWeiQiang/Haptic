#include "commTool.h"

int Basethread::Init() {
	{
		unsigned  uiThread1ID;
		HANDLE hth1 = (HANDLE)_beginthreadex(NULL, // security
			0,             // stack size
			ThreadStaticEntryPoint,// entry-point-function
			this,           // arg list holding the "this" pointer
			1, // so we can later call ResumeThread()
			&uiThread1ID);
		return 0;
	}
}

unsigned int __stdcall Basethread::ThreadStaticEntryPoint(void * pThis)
{
	ThreadX * pthX = (ThreadX*)pThis;   // the tricky cast
	pthX->ThreadEntryPoint();    // nowTimes call the true entry-point-function

								 // A thread terminates automatically if it completes execution,
								 // or it can terminate itself with a call to _endthread().

	return 1;          // the thread exit code
}

void Basethread::ThreadEntryPoint()
{
	// This is the desired entry-point-function but to get
	// here we have to use a 2 step procedure involving
	// the ThreadStaticEntryPoint() function.
	printf("ThreadX Thread\n");
}

template<typename S, typename R>
int Communicator<S,R>::Init(bool type, const char* addr, u_short remoteport, u_short myPort) {
	int err;
	WORD wVersionRequested;
	WSADATA wsaData;
	wVersionRequested = MAKEWORD(2, 2);

	// Initialize Winsock
	err = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (err != 0) {
		wprintf(L"WSAStartup failed: %d\n", err);
		return err;
	}

	SOCKET slisten;
	slisten = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (slisten == INVALID_SOCKET)
	{
		printf("socket error !");
		return 0;
	}

	//绑定IP和端口  
	sockaddr_in sin;
	sin.sin_family = AF_INET;
	sin.sin_port = htons(listenPort);
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

	printf("prepare to accepting remote socket\n");
	remoteSocket = accept(slisten, (SOCKADDR *)&remoteAddr, &nAddrlen);
	if (remoteSocket == INVALID_SOCKET)
	{
		printf("accept error !");
	}
	printf("accept remote address:%d \r\n", inet_ntoa(remoteAddr.sin_addr), ntohs(remoteAddr.sin_port));

	unsigned long on_windows = 1;
	if (ioctlsocket(remoteSocket, FIONBIO, &on_windows) == SOCKET_ERROR) {
		printf("non-block error");
	}

	return 0;
}