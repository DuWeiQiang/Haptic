/******************** Haptic Communication Library *************************
 * Software License Agreement (BSD License)                                *
 *                                                                         *
 *  Copyright (c) 2017,				                                       *
 *  Lehrstuhl für Medientechnik                                            *
 *  Technische Universität München, Germany                                *
 *  All rights reserved.                                                   *
 *                                                                         *
 *  Redistribution and use in source and binary forms, with or without     *
 *  modification, are permitted provided that the following conditions     *
 *  are met:                                                               *
 *                                                                         *
 *  - Redistributions of source code must retain the above copyright       *
 *     notice, this list of conditions and the following disclaimer.       *
 *  - Redistributions in binary form must reproduce the above              *
 *     copyright notice, this list of conditions and the following         *
 *     disclaimer in the documentation and/or other materials provided     *
 *     with the distribution.                                              *
 *  - Neither the name of Technische Universität München nor the names of  *
 *     its contributors may be used to endorse or promote products derived *
 *     from this software without specific prior written permission.       *
 *                                                                         *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    *
 *  'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS      *
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE         *
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,    *
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,   *
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;       *
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER       *
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT     *
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN      *
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE        *
 *  POSSIBILITY OF SUCH DAMAGE.                                            *
 ***************************************************************************
 *
 * Description : Haptic data reduction for haptic teleoperation. Perceptual deadband based
 * haptic data reduction decreases the high haptic rate.
 * 
 *
 * Author: Burak Cizmeci
 * e-mail: burak.cizmeci@tum.de/gmail.com
 * Contributors: 
 * Xiao Xu: Time Domain Passivity control for Communication delays
 * 
 *
 * IMPORTANT NOTE: The author of this code doesn't guarantee you safety on your hardware
 * even it is the same hardware configuration. It is your responsibility to take care 
 * of safety issues in case you copy and use this application
 *
 /** \file     HapticCommLib.cpp
    \brief    Haptic Communication Library CPP file
 *
 * Version 20 February 2017
 */



#include "HapticCommLib.h"

/***************** DeadbandDataReduction ********************/
/**
*	This function initializes the haptic data reduction related parameters
*	@author Burak Cizmeci
*	@param deadband paremter
* 	@date 23/11/2016
*/
DeadbandDataReduction::DeadbandDataReduction(double db) {

	DeadbandParameter = db;
	PreviousSample[0] = 10.0;
	PreviousSample[1] = 10.0;
	PreviousSample[2] = 10.0;

}

/***************** ~DeadbandDataReduction ********************/
/**
*	This function cleans safely the haptic data reduction related parameters
*	@author Burak Cizmeci
*	@param no need to give any input
* 	@date 23/11/2016
*/
DeadbandDataReduction::~DeadbandDataReduction() {

	printf("closing deadband class\n");

}
/***************** GetCurrentSample *************************/
/**
*	This function gets the recently captured haptic sample
*	@author Burak Cizmeci
*	@param needs the current haptic samples
* 	@date 23/11/2016
*/
void DeadbandDataReduction::GetCurrentSample(double* Sample){

	CurrentSample[0] = Sample[0];
	CurrentSample[1] = Sample[1];
	CurrentSample[2] = Sample[2];

}

/***************** ApplyZOHDeadband *************************/
/**
*	This function applies perceptual deadband based haptic data
*   reduction and makes a decision on the transmission of the 
*   current haptic sample. If no-transmission is decided, the
*   recent transmitted sample is displayed (zero order hold).
*	@author Burak Cizmeci
*	@param needs pointers for the updated sample and transmission flag
* 	@date 23/11/2016
*/
void DeadbandDataReduction::ApplyZOHDeadband(double* updatedSample,bool* TransmitFlag){

	// Compute the difference between recently transmitted signal and the current signal
	Difference[0] = fabs(CurrentSample[0]-PreviousSample[0]);
	Difference[1] = fabs(CurrentSample[1]-PreviousSample[1]);
	Difference[2] = fabs(CurrentSample[2]-PreviousSample[2]);
	DifferenceMag = sqrt(pow(Difference[0],2)+pow(Difference[1],2)+pow(Difference[2],2));

	// Compute the magnitude of the recently transmitted signal
	PreviousMag = sqrt(pow(PreviousSample[0],2)+pow(PreviousSample[1],2)+pow(PreviousSample[2],2));

	// Check whether the difference is above the perceptual threshold
	if(DifferenceMag/(PreviousMag+0.00001) >= DeadbandParameter) {
	
		// Transmit the current signal
		*TransmitFlag = true;
		updatedSample[0] = CurrentSample[0];
		updatedSample[1] = CurrentSample[1];
		updatedSample[2] = CurrentSample[2];

		// Update the previous sample for next iteration
		PreviousSample[0] = CurrentSample[0];
		PreviousSample[1] = CurrentSample[1];
		PreviousSample[2] = CurrentSample[2];
	
	}else {
	
		// Do not transmit the current signal
		*TransmitFlag = false;
		// Replicate the recently received signal (ZOH)
		updatedSample[0] = PreviousSample[0];
		updatedSample[1] = PreviousSample[1];
		updatedSample[2] = PreviousSample[2];
	
	}

	

}


KalmanFilter::KalmanFilter() {

	NoiseVar[0] = 200.0;
	NoiseVar[1] = 200.0;
	NoiseVar[2] = 200.0;

	ProcNoiseVar[0] = 1.0;
	ProcNoiseVar[1] = 1.0;
	ProcNoiseVar[2] = 1.0;

	PreviousEstimation[0] = 0.0;
	PreviousEstimation[1] = 0.0;
	PreviousEstimation[2] = 0.0;
	

}

KalmanFilter::~KalmanFilter() {




}

void KalmanFilter::ApplyKalmanFilter(double* CurrentSample) {


	for(int i=0; i<3;i++) {
		
		Innovation[i] = CurrentSample[i] - PreviousEstimation[i];
		InnovationVar[i] = PredictionErrorVar[i] + NoiseVar[i];
		Gain[i] = PredictionErrorVar[i]/InnovationVar[i];

		CurrentEstimation[i] = PreviousEstimation[i] + Gain[i]*Innovation[i];

		// update values for next iteration
		PredictionErrorVar[i] = PredictionErrorVar[i] + ProcNoiseVar[i] - Gain[i]*PredictionErrorVar[i];
		PreviousEstimation[i] = CurrentEstimation[i];
		
	
	}

}