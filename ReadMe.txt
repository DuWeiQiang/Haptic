	state:
		
		the base framework without any algorithm. 
		successful connect with omnet. 
		process of send and receive has been merged into updateHaptics Thread
		TDPA algorithm
		ISS algorithm
	simulation step:
		(with omnet)
			1. open omnet, run socket/omnet.ini
			2. run server application
			3. run client application
		(without omnnet)
			1. modify source code
				(server project)/main line 370: socketClientInit----> socketServerInit
				(client project)/main line 188: 4242------> 8888
			2. run server application
			3. run client application
	caution:
		this version's frequency is 1000Hz.
		The reason that the frequency of previous version is not 1KHz is the progeam compile mode is debug. 
		In debug mode, function computeInteractionForces will comsume more than 1ms.
		So, be careful, it is better to use release mode.

		in previous verison, I use frequency counter to detect whether 1ms has reached. 
		fix bug: when using omnet, the frequency of code is lowwer than 1khz.
		in this version, the code of counter has been delete. I user "tool->applyToDevice();" to timing.
	future work:
		rebuild the struct of the code
		add more algorithm
