	state:
		add game control

		add dynamic delay control

		add send slave video back to master

		add delay control.

		add position reset to fix position drift

		add MMT algorithm.

		fix some small bugs: 1. 1khz problem : 1ms counter related code(main.c in salve 1148-1151) cause the process frequency lower than 1KHz, move these codes to line 1148.
		bugs in main.c in slave 938 and 226 have been fixed.

		modify algorithms into 3D

		build a new project named HapticMixture. This new project used to solve 1khz compute frequency problem of bullet library

		restruct the solution: rename haptic proj into hapticmaster

		add bullet library.  The reason why we use bullet because it can restrict y and z axis movement which don't provided by ode library.
		add ground picture.
		add github repository.

		add algorithm switch.pls press N:none algorithm, I:ISS, T:TDPA
		rebuild the code struct:			
		the base framework without any algorithm. 
		successful connect with omnet. 
		process of send and receive has been merged into updateHaptics Thread
		TDPA algorithm
		ISS algorithm
		1KHz
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
		add more algorithm

