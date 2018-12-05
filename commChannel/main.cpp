
/* For fcntl */
#include <fcntl.h>
#include <ws2tcpip.h>
#include <event2/event.h>
#include <event2/buffer.h>
#include <event2/bufferevent.h>
#include <event2/thread.h>
#include <assert.h>

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include "commTool.h"
#define MAX_LINE 16384

socketInfo socketQueue[2];

int ConnectIndex = 0;

threadsafe_queue<myMessage> msgQ;

char
rot13_char(char c)
{
	/* We don't want to use isalpha here; setting the locale would change
	* which characters are considered alphabetical. */
	if ((c >= 'a' && c <= 'm') || (c >= 'A' && c <= 'M'))
		return c + 13;
	else if ((c >= 'n' && c <= 'z') || (c >= 'N' && c <= 'Z'))
		return c - 13;
	else
		return c;
}

void
readcb(struct bufferevent *bev, void *ctx)
{
	struct evbuffer *input, *output = NULL;
	size_t n;
	int i;
	input = bufferevent_get_input(bev);
	myMessage msg;

	struct socketInfo(*p)[2] = (struct socketInfo(*)[2])ctx;
	int myFD = bufferevent_getfd(bev);
	if (myFD == (*p)[0].fd) {
		//send to 1 fd

		msg.oPort = (*p)[1].fd;
		QueryPerformanceCounter((LARGE_INTEGER *)&msg.timestamp);
		msg.msgLength = evbuffer_remove(input, msg.msgPtr, sizeof(msg.msgPtr));
		msgQ.push(msg);
		//std::cout << msgQ.length() << std::endl;
		//if ((*p)[1].fd != INVALID_SOCKET) {
		//	//bufferevent_read_buffer(bev,
		//	//	bufferevent_get_output((*p)[1].bev));
		//	output = bufferevent_get_output((*p)[1].bev);
		//}
	}
	else {
		msg.oPort = (*p)[0].fd;
		QueryPerformanceCounter((LARGE_INTEGER *)&msg.timestamp);
		msg.msgLength = evbuffer_remove(input, msg.msgPtr, sizeof(msg.msgPtr));
		msgQ.push(msg);
		//send to 0 fd
		//if ((*p)[0].fd != INVALID_SOCKET) {
		//	//bufferevent_read_buffer(bev,
		//	//	bufferevent_get_output((*p)[0].bev));
		//	output = bufferevent_get_output((*p)[0].bev);
		//}
	}
	
	//char buf[1024];
	//while (evbuffer_get_length(input)) {
	//	int n = evbuffer_remove(input, buf, sizeof(buf));
	//	for (i = 0; i < n; ++i)
	//		buf[i] = rot13_char(buf[i]);
	//	if (output != NULL)
	//		evbuffer_add(output, buf, n);
	//}
	
	//while ((line = evbuffer_readln(input, &n, EVBUFFER_EOL_LF))) {
	//	for (i = 0; i < n; ++i)
	//		line[i] = rot13_char(line[i]);
	//	evbuffer_add(output, line, n);
	//	evbuffer_add(output, "\n", 1);
	//	free(line);
	//}

	//if (evbuffer_get_length(input) >= MAX_LINE) {
		/* Too long; just process what there is and go on so that the buffer
		* doesn't grow infinitely long. */
	//char buf[1024];
	//while (evbuffer_get_length(input)) {
	//	int n = evbuffer_remove(input, buf, sizeof(buf));
	//	for (i = 0; i < n; ++i)
	//		buf[i] = rot13_char(buf[i]);
	//	evbuffer_add(output, buf, n);
	//}
	//evbuffer_add(output, "\n", 1);
	//}
}

void
errorcb(struct bufferevent *bev, short error, void *ctx)
{
	if (error & BEV_EVENT_EOF) {
		/* connection has been closed, do any clean up here */
		/* ... */
	}
	else if (error & BEV_EVENT_ERROR) {
		/* check errno to see what error occurred */
		/* ... */
	}
	else if (error & BEV_EVENT_TIMEOUT) {
		/* must be a timeout event handle, handle it */
		/* ... */
	}
	
	struct socketInfo(*p)[2] = (struct socketInfo(*)[2])ctx;
	int myFD = bufferevent_getfd(bev);
	printf("%d Closed\n", myFD);
	if (myFD == (*p)[0].fd) {
		//send to 1 fd		
		(*p)[0].fd = INVALID_SOCKET;
	}
	else {
		(*p)[1].fd = INVALID_SOCKET;
	}


	bufferevent_free(bev);
}

void
do_accept(evutil_socket_t listener, short event, void *arg)
{
	struct event_base *base = (struct event_base *)arg;
	struct sockaddr_storage ss;
	socklen_t slen = sizeof(ss);
	int fd = accept(listener, (struct sockaddr*)&ss, &slen);
	
	if (fd < 0) {
		perror("accept");
	}
	
	struct sockaddr_in ip_adr_get;
	int ip_adr_len;
	ip_adr_len = sizeof(ip_adr_get);
	getpeername(fd, (sockaddr*)&ip_adr_get, &ip_adr_len);
	printf("IP address is: %s\n", inet_ntoa(ip_adr_get.sin_addr));
	printf("Port is: %d\n", (int)ntohs(ip_adr_get.sin_port));

	struct bufferevent *bev;
	evutil_make_socket_nonblocking(fd);
	bev = bufferevent_socket_new(base, fd, BEV_OPT_CLOSE_ON_FREE);
	bufferevent_setcb(bev, readcb, NULL, errorcb, socketQueue);
	bufferevent_setwatermark(bev, EV_READ, 0, MAX_LINE);
	bufferevent_enable(bev, EV_READ | EV_WRITE);

	while (socketQueue[ConnectIndex].fd != INVALID_SOCKET) {
		ConnectIndex++;
	}
	socketQueue[ConnectIndex].fd = fd;
	socketQueue[ConnectIndex].bev = bev;
	ConnectIndex = 0;
}

void
run(void)
{
	evutil_socket_t listener;
	struct sockaddr_in sin;
	struct event_base *base;
	struct event *listener_event;

#ifdef _WIN32
	WSADATA wsa_data;
	WSAStartup(0x0201, &wsa_data);
#endif
	evthread_use_windows_threads();
	base = event_base_new();
	if (!base)
		return; /*XXXerr*/

	sin.sin_family = AF_INET;
	sin.sin_addr.s_addr = INADDR_ANY;
	sin.sin_port = htons(4242);

	listener = socket(AF_INET, SOCK_STREAM, 0);
	evutil_make_socket_nonblocking(listener);

#ifndef WIN32
	{
		int one = 1;
		setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
	}
#endif

	if (bind(listener, (struct sockaddr*)&sin, sizeof(sin)) < 0) {
		perror("bind");
		return;
	}

	if (listen(listener, 16)<0) {
		perror("listen");
		return;
	}

	listener_event = event_new(base, listener, EV_READ | EV_PERSIST, do_accept, (void*)base);
	/*XXX check it */
	event_add(listener_event, NULL);

	event_base_dispatch(base);
}

int
main(int c, char **v)
{
	socketQueue[0].fd = INVALID_SOCKET;
	socketQueue[1].fd = INVALID_SOCKET;

	Sender *sender = new Sender();
	sender->Q = &msgQ;
	sender->p = &socketQueue;
	unsigned  uiThread1ID;
	HANDLE hth1 = (HANDLE)_beginthreadex(NULL, // security
		0,             // stack size
		ThreadX::ThreadStaticEntryPoint,// entry-point-function
		sender,           // arg list holding the "this" pointer
		CREATE_SUSPENDED, // so we can later call ResumeThread()
		&uiThread1ID);
	ResumeThread(hth1);

	setvbuf(stdout, NULL, _IONBF, 0);

	run();
	return 0;
}