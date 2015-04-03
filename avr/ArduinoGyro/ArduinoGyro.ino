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
#include "CalLib.h"
#include <EEPROM.h>

RTIMU *imu;                                           // the IMU object
RTIMUSettings settings;                               // the settings object
bool valid = false;

// Orientation
float xOrientation = 0.0f;
float yOrientation = 0.0f;
float zOrientation = 0.0f;

// Time tracking
unsigned long lastRate;

// Command system variables
unsigned char  command = 0;        // stores a processed command
unsigned char  commandBytes = 0;   // stores the bytes this command expects
unsigned long  lastUpdate = 0;

void setup()
{
  int errcode;
  
  // Start the communication busses
  Serial.begin(57600);
  Wire.begin();
  
  // Try to start the IMU
  imu = RTIMU::createIMU(&settings);
  if ((errcode = imu->IMUInit()) < 0) 
  {
    Serial.print("Failed to init IMU: "); 
    Serial.println(errcode);
    while(1) {}
  }
  
  // Last rate is now
  lastRate = millis();
}

void loop()
{  
  // Get the current time and delta time
  unsigned long now = millis();
  float delta = ((float) (now - lastRate)) / 1000.0f;
  
    // Check if we are collecting bytes for a command and if we have achieved that number
  if((commandBytes > 0) && Serial.available() >= commandBytes) 
  {
    // If this is the set gyro value
    if(command == 3)
    {
      // Buffer to store the received value
      float Z = 0.0f;
      Serial.readBytes((char *) &Z, 4);
      zOrientation = Z * (M_PI * 180.0f);
    }
    
    // Clear command bytes
    commandBytes = 0;
  } 
  
  // Else if we aren't waiting for comand bytes and there is a command pair in the buffer
  else if((commandBytes == 0) && Serial.available() >= 2) 
  {
    // Check that the byte is the prefix of a command
    if(Serial.read() == '#')
    {
      // Get the command
      char cmd = Serial.read();
      
      // Send a synchronization token
      if(cmd == 'a')
      {
        command = 1;
        commandBytes = 0;
        
        // Write a string into the byte stream to look for
        Serial.print("#SYNCH");
        Serial.println();
      } 
      
      // Command to reset the Z axis gyro (reset the heading)
      else if(cmd == 'z')
      {
        // Set command
        command = 2;
        commandBytes = 0;
         
        // Reset the gyro values
        zOrientation = 0;
      }
      
      // Command to set the Z axis gyro to a given value
      else if(cmd == 's')
      {
        // Set command to set gyro
        command = 3;
        commandBytes = 4;
      
        // Wait for data reception to complete by setting command bytes  
      }
    }
  }
  
  // If we have new IMU data
  if (imu->IMURead())    
  {                                
    // Check if the IMU data is valid (as in calibrating)
    if ((now - lastRate) >= 125 && !valid) 
    {
      valid = imu->IMUGyroBiasValid();
    }
    
    // If the gyro is valid
    if(valid)
    {
      // Integrate the gyroscope reading
      RTVector3 v = ((RTVector3&) imu->getGyro());
      xOrientation += v.x() * delta;
      yOrientation += v.y() * delta;
      zOrientation += v.z() * delta;
    
      // Normalize angles to standard values 
      //    0 degrees = north, or in this case starting orientation of the gyro
      //    negative degrees = turned left
      //    positive degrees = turned right
      float twopi = 2.0f * M_PI;
      while(xOrientation > M_PI)  xOrientation -= twopi;
      while(xOrientation < -M_PI) xOrientation += twopi;
      while(yOrientation > M_PI)  yOrientation -= twopi;
      while(yOrientation < -M_PI) yOrientation += twopi;
      while(zOrientation > M_PI)  zOrientation -= twopi;
      while(zOrientation < -M_PI) zOrientation += twopi;
      
      // Write the floating point data to our host
      float X = xOrientation * (180.0f / M_PI);
      float Y = yOrientation * (180.0f / M_PI);
      float Z = zOrientation * (180.0f / M_PI);
      Serial.write((uint8_t *) &X, 4);
      Serial.write((uint8_t *) &Y, 4);
      Serial.write((uint8_t *) &Z, 4);
      //Serial.print(xOrientation);
      //Serial.print(' ');
      //Serial.print(yOrientation);
      //Serial.print(' ');
      //Serial.println(zOrientation);
      //Serial.print(X);
      //Serial.print(' ');
      //Serial.print(Y);
      //Serial.print(' ');
      //Serial.println(Z);
    
      // Last imu update
      lastRate = now;
    }
  }
}

