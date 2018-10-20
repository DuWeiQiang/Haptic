commit 06cf2bb8:
	state:
		the base framework without any algorithm. 
		successful connect with omnet. 
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
		this version's frequency is not 1000Hz, should be fixed in the furture. pls do not modify (client project)/main line 398.
		lower value of count could increase transmit speed at the beginning so that to many packet blocked in server's packet and increase latency.
	future work:
		add algorithm



commit this version:
	state:
		base framework with TDPA algorithm
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
		this version's frequency is not 1000Hz, should be fixed in the furture. pls do not modify (client project)/main line 638.
		lower value of count could increase transmit speed at the beginning so that to many packet blocked in server's packet and increase latency which will cause unstability.
	future work;
		modify code structure
		add more algorithm.