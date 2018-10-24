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
/** \file     HapticCommLib.h
    \brief    Haptic Communication Library header file
 *
 * Version 20 February 2017
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <list>
#include <queue>


// structure holding a haptic sample including its timestamp
typedef struct {

	double Sample[3]; // Sample values
	unsigned int GenTime; // generation clock time for delay computing

}HapticSample;

// Perceptaul haptic data reduction class

class DeadbandDataReduction{

public:
	  DeadbandDataReduction(double db); // initializes a deadband class for a 3 DoF haptic signal
     ~DeadbandDataReduction(); // kills the deadband class


	 double DeadbandParameter; // variable holding the deadband parameter 0<DB<1

	 void GetCurrentSample(double* Sample); // copies a new sample for deadband computation
	 void ApplyZOHDeadband(double* updatedSample,bool* TransmitFlag); // performs the deadband data reduction, decides the transmission state


private:

	double CurrentSample[3]; // variable holding the current haptic sample
	double PreviousSample[3]; // variable holding the previosly transmitted haptic sample
	double Difference[3]; 
	double DifferenceMag;
	double PreviousMag;


};


class KalmanFilter{

public:
	KalmanFilter();
	~KalmanFilter();

	double CurrentEstimation[3];
	double PreviousEstimation[3];
	double PredictionErrorVar[3];

	void ApplyKalmanFilter(double* CurrentSample);

private:

	double Innovation[3]; // I
	double InnovationVar[3]; // S
	double NoiseVar[3]; // R
	double Gain[3]; // K
	double ProcNoiseVar[3]; // Q

};
