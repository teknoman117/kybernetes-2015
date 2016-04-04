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

#include <Wire.h>

#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h" 
#include "CalLib.h"

#include <EEPROM.h>
#include <NewPing.h>
#include <PinChangeInt.h>

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define  SERIAL_PORT_SPEED  57600

// --------------- IMU -----------------
// IMU objects
RTIMU         *imu;                                    // the IMU object
RTFusionRTQF   fusion;                                 // the fusion object
RTIMUSettings  settings;                               // the settings object

// Orientation
float X = 0.0f;
float Y = 0.0f;
float Z = 0.0f;

unsigned long  lastIMUCheck;
bool imuValid;
bool imuInitialized;
// -------------------------------------

// --------------- SONARS --------------
#define SONAR_NUM     3 // Number or sensors.
#define MAX_DISTANCE 220 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(3, A3, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(2, A4, MAX_DISTANCE),
  NewPing(4, A2, MAX_DISTANCE),
};
// -------------------------------------

// -------------- BUMPER ----------
const int bumperA = 10;
const int bumperB = 11;
volatile bool bumpersChanged = false;
// --------------------------------

void setup()
{
  delay(1000);
  
  // Initialize Hardware
  Serial.begin(SERIAL_PORT_SPEED);
  Wire.begin();
  
  // Initialize the IMU
  imu = RTIMU::createIMU(&settings);                        // create the imu object
  int error = imu->IMUInit();
  if (error < 0) 
  {
    imuInitialized = false;
    Serial.println("IMUSTATUS:ERROR");
  }
  
  // IMU was successfully started
  else
  {
    imuInitialized = true;
    pinMode(12,OUTPUT);
    digitalWrite(12,LOW);
    if (imu->getCalibrationValid())
    {
      Serial.println("IMUSTATUS:INITIALIZED;Using compass calibration");
      digitalWrite(12,HIGH);
    }
    else 
    {
      Serial.println("IMUSTATUS:INITIALIZED;No valid compass calibration data");
      digitalWrite(12,LOW);
    }
  }
  
  // Initialize variables
  lastIMUCheck = millis();
  imuValid = false;
  
  // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  pingTimer[0] = millis() + 75;

  // Set the starting time for each sensor.  
  for (uint8_t i = 1; i < SONAR_NUM; i++) 
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
    
  // Register interrupts
  pinMode(bumperA, INPUT);
  pinMode(bumperB, INPUT);
  attachPinChangeInterrupt(bumperA, bumperChanged, CHANGE);
  attachPinChangeInterrupt(bumperB, bumperChanged, CHANGE);
}

void loop()
{  
  // Get the current time
  unsigned long now = millis();
  
  // get the latest data if ready yet
  if (imuInitialized && imu->IMURead())     
  {         
    // Compute the delta
    float delta = ((float)(now - lastIMUCheck)) / 1000.0f;
    
    // If the gyro is valid
    if(imuValid)
    {
      // Integrate the gyroscope reading
      RTVector3 v = ((RTVector3&) imu->getGyro());
      X += v.x() * delta;
      Y += v.y() * delta;
      Z += v.z() * delta;
    
      // Normalize angles to standard values 
      //    0 degrees = north, or in this case starting orientation of the gyro
      //    negative degrees = turned left
      //    positive degrees = turned right
      float twopi = 2.0f * M_PI;
      while(X > M_PI)  X -= twopi;
      while(X < -M_PI) X += twopi;
      while(Y > M_PI)  Y -= twopi;
      while(Y < -M_PI) Y += twopi;
      while(Z > M_PI)  Z -= twopi;
      while(Z < -M_PI) Z += twopi;
      
      // Publish the IMU packet
      Serial.print("IMU:");
      Serial.print(X * (180.0f / M_PI));
      Serial.print(";");
      Serial.print(Y * (180.0f / M_PI));
      Serial.print(";");
      Serial.println(Z * (180.0f / M_PI));
      
      // Last IMU check
      lastIMUCheck = now;
    }
    
    // Otherwise we are still checking on the IMU
    else if(now - lastIMUCheck >= 125)
    {
      if(imu->IMUGyroBiasValid())
      {
        Serial.println("IMUSTATUS:READY");
        imuValid = true;
      }
      lastIMUCheck = now; 
    }
  }
  
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
  
  if(bumpersChanged)
  { 
    bumpersChanged = false;
    Serial.print("BUMPER:");
    Serial.print(digitalRead(bumperA));
    Serial.print(";");
    Serial.print(digitalRead(bumperB));
    Serial.println();
  }
}

// If ping received, set the sensor distance to array.
void echoCheck()     
{ 
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  Serial.print("SONAR:");
  Serial.print(cm[0]);
  for (uint8_t i = 1; i < SONAR_NUM; i++) 
  {
    Serial.print(";");
    Serial.print(cm[i]);
  }
  Serial.println();
}

void bumperChanged()
{
  bumpersChanged = true;
}

