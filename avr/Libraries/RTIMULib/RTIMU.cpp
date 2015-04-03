////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "RTIMU.h"
#include "RTIMUSettings.h"
#include "CalLib.h"

#include "RTIMUMPU9150.h"
#include "RTIMULSM9DS0.h"

RTIMU *RTIMU::createIMU(RTIMUSettings *settings)
{
    switch (settings->m_imuType) {
    case RTIMU_TYPE_MPU9150:
        return new RTIMUMPU9150(settings);

    case RTIMU_TYPE_LSM9DS0:
        return new RTIMULSM9DS0(settings);
		
    case RTIMU_TYPE_AUTODISCOVER:
        if (settings->discoverIMU(settings->m_imuType, settings->m_I2CSlaveAddress)) {
            return RTIMU::createIMU(settings);
        }
        return NULL;

    default:
        return 0;
    }
}


RTIMU::RTIMU(RTIMUSettings *settings)
{
    m_settings = settings;

    m_calibrationMode = false;
    m_calibrationValid = false;
}

RTIMU::~RTIMU()
{
}

void RTIMU::setCalibrationData()
{
    float maxDelta = -1;
    float delta;
	CALLIB_DATA calData;                                    

	m_calibrationValid = false;

	if (calLibRead(0, &calData)) {					
		if (calData.magValid != 1) {
			return;
		}

		//  find biggest range

		for (int i = 0; i < 3; i++) {
			if ((calData.magMax[i] - calData.magMin[i]) > maxDelta)
				maxDelta = calData.magMax[i] - calData.magMin[i];
		}
		if (maxDelta < 0) {
			return;
		}
		maxDelta /= 2.0f;                                       // this is the max +/- range

		for (int i = 0; i < 3; i++) {
			delta = (calData.magMax[i] - calData.magMin[i]) / 2.0f;
			m_compassCalScale[i] = maxDelta / delta;            // makes everything the same range
			m_compassCalOffset[i] = (calData.magMax[i] + calData.magMin[i]) / 2.0f;
		}
		m_calibrationValid = true;
	}
}
