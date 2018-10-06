#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <WINSOCK2.H>
#include <STDIO.H>
#include <iostream>
#include <string>
#include <windows.h>
#include <process.h>
#pragma comment(lib,"ws2_32.lib")  

#include <queue>
#include <mutex>
#include <memory>
#include <condition_variable>
struct hapticMessageM2S {
	// position, cVector3d contains 3 double values
	double position[3];

	// orientation, cMatrix3d contains 3 double values
	double rotation[9];

	// gripper position
	double gripperAngle;

	// linear velocity, cVector3d contains 3 double values
	double linearVelocity[3];

	// angular velocity, cVector3d contains 3 double values
	double angularVelocity[3];

	// gripper angular velocity
	double gripperAngularVelocity;

	// user-switch status (button 0)
	int button0, button1, button2, button3;
};


struct hapticMessageS2M {
	double force[3];
	double torque[3];
	double gripperForce;
};

template<typename T>
class threadsafe_queue
{
public:
	threadsafe_queue() {}
	~threadsafe_queue() {}

	void push(T new_data)
	{
		std::lock_guard<std::mutex> lk(mut);            // 1.全局加锁
		data_queue.push(std::move(new_data));           // 2.push时独占锁
		cond.notify_one();
	}
	void wait_and_pop(T& val)
	{
		std::unique_lock<std::mutex> ulk(mut);                    // 3.全局加锁
		cond.wait(ulk, [this]() { return !data_queue.empty(); });  // 4.front 和 pop_front时独占锁
		val = std::move(data_queue.front());
		data_queue.pop();
	}
	std::shared_ptr<T> wait_and_pop()
	{
		std::unique_lock<std::mutex> ulk(mut);
		cond.wait(ulk, [this]() { return !data_queue.empty(); });
		std::shared_ptr<T> val(std::make_shared<T>(std::move(data_queue.front())));
		data_queue.pop();
		return val;
	}
	bool try_pop(T& val)
	{
		std::lock_guard<std::mutex> lk(mut);
		if (data_queue.empty())
			return false;
		val = std::move(data_queue.front());
		data_queue.pop();
		return true;
	}
	std::shared_ptr<T> try_pop()
	{
		std::shared_ptr<T> val;
		std::lock_guard<std::mutex> lk(mut);
		if (data_queue.empty())
			return val;
		val = std::make_shared<T>(std::move(data_queue.front()));
		data_queue.pop();
		return val;
	}
	bool empty()
	{
		std::lock_guard<std::mutex> lk(mut);
		return data_queue.empty();
	}
private:
	std::queue<T> data_queue;
	std::mutex mut;
	std::condition_variable cond;
};




class ThreadX
{

public:
	SOCKET s;
	int N;
	
	// In C++ you must employ a free (C) function or a static
	// class member function as the thread entry-point-function.
	ThreadX() {
		N = 10;
	}
	static unsigned __stdcall ThreadStaticEntryPoint(void * pThis)
	{
		ThreadX * pthX = (ThreadX*)pThis;   // the tricky cast
		pthX->ThreadEntryPoint();    // now call the true entry-point-function

									 // A thread terminates automatically if it completes execution,
									 // or it can terminate itself with a call to _endthread().

		return 1;          // the thread exit code
	}

	virtual void ThreadEntryPoint()
	{
		// This is the desired entry-point-function but to get
		// here we have to use a 2 step procedure involving
		// the ThreadStaticEntryPoint() function.
		printf("ThreadX Thread\n");
	}
	virtual ~ThreadX() {};
};

class Sender :public ThreadX
{
public:
	threadsafe_queue<hapticMessageS2M> *Q;
private:	
	void ThreadEntryPoint() {
		printf("Sender Thread\n");
		while (true) {
			if (!Q->empty()) {
				//std::cout << "Sender helloworld" << sizeof(hapticMessageM2S) << std::endl;
				hapticMessageS2M temp;
				Q->try_pop(temp);
				send(s, (char *)&temp, sizeof(hapticMessageS2M), 0);
			}
		}
	}
	virtual ~Sender() {};
};

class Receiver :public ThreadX
{
public:
	threadsafe_queue<hapticMessageM2S> *Q;
private:
	void ThreadEntryPoint() {
		printf("Receiver Thread\n");
		char recData[255];
		unsigned int unprocessedPtr = 0;
		while (true) {
			int ret = recv(s, recData + unprocessedPtr, 255 - unprocessedPtr, 0);
			if (ret>0) {
				// we receive some char data and transform it to hapticMessageM2S.
				unprocessedPtr += ret;

				unsigned int hapticMsgL = sizeof(hapticMessageM2S);
				for (unsigned int i = 0; i < unprocessedPtr / hapticMsgL; i++) {
					Q->push(*(hapticMessageM2S*)(recData + i* hapticMsgL));
				}
				unsigned int processedPtr = (unprocessedPtr / hapticMsgL) * hapticMsgL;
				unprocessedPtr %= hapticMsgL;

				for (unsigned int i = 0; i < unprocessedPtr; i++) {
					recData[i] = recData[processedPtr + i];
				}
			}
		}
	}
	virtual ~Receiver() {};
};

//int main1(int argc, char* argv[])
//{
//	//初始化WSA  
//	WORD sockVersion = MAKEWORD(2, 2);
//	WSADATA wsaData;
//	threadsafe_queue<hapticMessageM2S> *forceQ = new threadsafe_queue<hapticMessageM2S>();
//	threadsafe_queue<hapticMessageM2S> *commandQ = new threadsafe_queue<hapticMessageM2S>();
//
//
//	if (WSAStartup(sockVersion, &wsaData) != 0)
//	{
//		return 0;
//	}
//
//	//创建套接字  
//	SOCKET slisten = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
//	if (slisten == INVALID_SOCKET)
//	{
//		printf("socket error !");
//		return 0;
//	}
//
//	//绑定IP和端口  
//	sockaddr_in sin;
//	sin.sin_family = AF_INET;
//	sin.sin_port = htons(8888);
//	sin.sin_addr.S_un.S_addr = INADDR_ANY;
//	if (bind(slisten, (LPSOCKADDR)&sin, sizeof(sin)) == SOCKET_ERROR)
//	{
//		printf("bind error !");
//	}
//
//	//开始监听  
//	if (listen(slisten, 5) == SOCKET_ERROR)
//	{
//		printf("listen error !");
//		return 0;
//	}
//
//	//循环接收数据  
//	SOCKET sClient;
//	sockaddr_in remoteAddr;
//	int nAddrlen = sizeof(remoteAddr);
//
//	printf("等待连接...\n");
//	sClient = accept(slisten, (SOCKADDR *)&remoteAddr, &nAddrlen);
//	if (sClient == INVALID_SOCKET)
//	{
//		printf("accept error !");
//	}
//	printf("接受到一个连接：%s:%d \r\n", inet_ntoa(remoteAddr.sin_addr), ntohs(remoteAddr.sin_port));
//
//	Sender *sender = new Sender();
//	Receiver *receiver = new Receiver();
//	sender->s = sClient;
//	sender->Q = commandQ;
//	receiver->s = sClient;
//	receiver->Q = forceQ;
//	unsigned  uiThread1ID;
//	HANDLE hth1 = (HANDLE)_beginthreadex(NULL, // security
//		0,             // stack size
//		ThreadX::ThreadStaticEntryPoint,// entry-point-function
//		sender,           // arg list holding the "this" pointer
//		CREATE_SUSPENDED, // so we can later call ResumeThread()
//		&uiThread1ID);
//	ResumeThread(hth1);
//	// From here on there are two separate threads executing
//	// our one program.
//	unsigned  uiThread2ID;
//	HANDLE hth2 = (HANDLE)_beginthreadex(NULL, // security
//		0,             // stack size
//		ThreadX::ThreadStaticEntryPoint,// entry-point-function
//		receiver,           // arg list holding the "this" pointer
//		CREATE_SUSPENDED, // so we can later call ResumeThread()
//		&uiThread2ID);
//	ResumeThread(hth2);
//	// This main thread can call the silly() function if it wants to.
//	//WaitForSingleObject(hth1, INFINITE);
//	//WaitForSingleObject(hth2, INFINITE);
//	while (true) {
//		if (!forceQ->empty()) {
//			hapticMessageM2S temp;
//			forceQ->try_pop(temp);
//			commandQ->push(temp);
//		}
//	}
//
//
//	DWORD   dwExitCode;
//	GetExitCodeThread(hth1, &dwExitCode);
//	printf("initial thread 1 exit code = %u\n", dwExitCode);
//	GetExitCodeThread(hth2, &dwExitCode);
//	printf("initial thread 1 exit code = %u\n", dwExitCode);
//
//	closesocket(slisten);
//	WSACleanup();
//	return 0;
//}
