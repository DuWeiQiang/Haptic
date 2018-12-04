#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <WINSOCK2.H>
#include <STDIO.H>
#include <iostream>
#include <string>
#include <windows.h>
#include <process.h>
#include <timeapi.h>
#pragma comment(lib,"ws2_32.lib")  
#pragma comment( lib,"winmm.lib" )
#include <queue>
#include <mutex>
#include <memory>
#include <condition_variable>

#include <fcntl.h>
#include <ws2tcpip.h>
#include <event2/event.h>
#include <event2/buffer.h>
#include <event2/bufferevent.h>
enum AlgorithmType { AT_None, AT_TDPA, AT_ISS, AT_MMT, AT_KEEP };

struct hapticMessageM2S {
	__int64 time;

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

	double userSwitches;

	double energy[3];

	double waveVariable[3];

	// user-switch status (button 0)
	int button0, button1, button2, button3;

	AlgorithmType ATypeChange;
};


struct hapticMessageS2M {
	__int64 time;
	double force[3];
	double torque[3];
	double gripperForce;
	double energy[3];
	double waveVariable[3];
	double MMTParameters[7];
};

template<typename T>
class threadsafe_queue
{
public:
	threadsafe_queue() {}
	~threadsafe_queue() {}

	size_t length() {
		std::lock_guard<std::mutex> lk(mut);            // 1.全局加锁
		return data_queue.size();
	}

	T front() {
		std::lock_guard<std::mutex> lk(mut);            // 1.全局加锁
		return data_queue.front();
	}

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

struct socketInfo {
	UINT_PTR fd;
	struct bufferevent *bev;
};

struct myMessage {
	int oPort;
	__int64 timestamp;
	char msgPtr[1024];
	int msgLength;
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
		pthX->ThreadEntryPoint();    // nowTimes call the true entry-point-function

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
	threadsafe_queue<myMessage> *Q;
	struct socketInfo(*p)[2];
private:
	void ThreadEntryPoint() {
		printf("Sender sThread\n");
		while (true) {
			__int64 currentTime;
			struct evbuffer *output = NULL;
			if (Q->length()) {
				QueryPerformanceCounter((LARGE_INTEGER *)&currentTime);
				if (currentTime - Q->front().timestamp > 3609*1000) {
					myMessage temp;
					Q->try_pop(temp);
					if(temp.oPort != INVALID_SOCKET){
						if (temp.oPort == (*p)[0].fd) {
							std::cout << "hello" << std::endl;
							output = bufferevent_get_output((*p)[0].bev);
						}
						else if(temp.oPort == (*p)[1].fd) {
							std::cout << "hello2" << std::endl;
							output = bufferevent_get_output((*p)[1].bev);
						}
						if (output != NULL)
							evbuffer_add(output, temp.msgPtr, temp.msgLength);
					}
						
				}

			}


			//myMessage temp;
			//if (Q->try_pop(temp)) {
			//	//std::cout << "Sender helloworld" << sizeof(hapticMessageM2S) << std::endl;


			//	send(s, (char *)&temp, sizeof(myMessage), 0);
			//}
			//Sleep(1);
			//std::this_thread::sleep_for(std::chrono::microseconds(500));
		}
	}
	virtual ~Sender() {};
};

template<typename T>
class Receiver :public ThreadX
{
public:
	threadsafe_queue<T> *Q;
private:
	void ThreadEntryPoint() {
		printf("Receiver Thread\n");
		char recData[1000];
		unsigned int unprocessedPtr = 0;
		while (true) {
			int ret = recv(s, recData + unprocessedPtr, 1000 - unprocessedPtr, 0);
			if (ret>0) {
				// we receive some char data and transform it to hapticMessageM2S.
				unprocessedPtr += ret;

				unsigned int hapticMsgL = sizeof(T);
				for (unsigned int i = 0; i < unprocessedPtr / hapticMsgL; i++) {
					Q->push(*(T*)(recData + i* hapticMsgL));
				}
				unsigned int processedPtr = (unprocessedPtr / hapticMsgL) * hapticMsgL;
				unprocessedPtr %= hapticMsgL;

				for (unsigned int i = 0; i < unprocessedPtr; i++) {
					recData[i] = recData[processedPtr + i];
				}
			}
			Sleep(1);
			//std::this_thread::sleep_for(std::chrono::microseconds(500));
		}
	}
	virtual ~Receiver() {};
};
