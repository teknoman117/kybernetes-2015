/*
 *  MotorController.ino
 *
 *  Copyright (c) 2013 Nathaniel Lewis, Robotics Society at UC Merced
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Servo.h>
#include <stdint.h>
#include <avr/wdt.h>

// Encoder Inputs
#define encoderCHAInput PIND3
#define encoderCHBInput PIND2
#define encodersDDR     DDRD
#define encodersPortIn  PIND

unsigned char _state = 0;
int32_t        odometer = 0;

// Servos
#define throttleOutput  13
#define steeringOutput  11
#define throttle_stop   1550
#define steering_center 1550

Servo   steeringServo;
Servo   throttleServo;
short   steeringTarget = 0;
short   throttleTarget = 0;
int32_t positionTarget = 0;

// Radio Inputs
#define throttleInput   PIND7
#define steeringInput   PIND6
#define radioDDR        DDRD
#define radioPortIn     PIND

short   throttleValue = 1500;
short   steeringValue = 1500;
unsigned long sS, sT;
unsigned char _state_s_ = 0, _state_t_ = 0;

// Wheel Clear Sensors
const int leftClearInput = A0;
const int backClearInput = A1;
const int rightClearInput = A2;

// Command system
unsigned char  command = 0;        // stores a processed command
unsigned char  commandBytes = 0;   // stores the bytes this command expects
unsigned long  lastUpdate = 0;

// Setup the initial state of the controller
void setup() {
  // Start the serial uplink
  Serial.begin(57600); 
  wdt_enable(WDTO_120MS);
  
  // Configure the radio ports
  radioDDR &= ~_BV(throttleInput) & ~_BV(steeringInput);
  sS = sT = micros();
  PCMSK2 = _BV(throttleInput) | _BV(steeringInput);
  PCICR = _BV(PCIE2);

  // Configure encoders
  encodersDDR &= ~_BV(encoderCHAInput) & ~_BV(encoderCHBInput);
  attachInterrupt(1, encoderTickA, CHANGE);
  attachInterrupt(0, encoderTickB, CHANGE);
  
  // Power up the motor drivers and get a signal out (a 10k resistor acts as a pull down so the hb25 will fire up)
  pinMode(throttleOutput, OUTPUT);
  digitalWrite(throttleOutput, LOW);
  throttleServo.writeMicroseconds(throttle_stop);
  throttleServo.attach(throttleOutput);

  // Initalize steering servos
  pinMode(steeringOutput, OUTPUT);
  steeringServo.writeMicroseconds(steering_center);
  steeringServo.attach(steeringOutput);
  
  // Ensure interrupts are online
  sei();
}

// Main loop of execution
void loop() {
  // Check if we are collecting bytes for a command and if we have achieved that number
  if((commandBytes > 0) && Serial.available() >= commandBytes) 
  {
    // Set motor target velocity
    if(command == 1)
    {
      // Takes a 2 byte short integer
      Serial.readBytes((char *) &throttleTarget, 2);
    } 
    
    // Set the steering target
    else if(command == 2)
    {
      // Takes a 2 byte short integer
      Serial.readBytes((char *) &steeringTarget, 2);
      
      // Set the steering servo
      steeringServo.writeMicroseconds(steeringTarget + steering_center);
    }
    
    // Set the position target
    else if(command == 3)
    {
      // Takes a 4 byte position, then a 2 byte maximum throttle
      Serial.readBytes((char *) &positionTarget, 4);
      Serial.readBytes((char *) &throttleTarget, 2); 
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
      
      // Set the throttle
      if(cmd == 't')
      {
        command = 1;
        commandBytes = 2;  
      } 
      
      // Set the steering target
      else if(cmd == 's')
      {
        command = 2;
        commandBytes = 2;
      } 
      
      // Set the position target
      else if(cmd == 'p')
      {
        command = 3;
        commandBytes = 6;
      }
      
      // Request synchronization
      else if(cmd == 'a')
      {
        command = 4;
        commandBytes = 0;
        
        // Write a string into the byte stream to look for
        Serial.print("#SYNCH");
        Serial.println();
      }
    }
  }
  
  // Move if motion is enabled
  unsigned char en = (throttleValue > 1650);
  if(!en) 
  {
    // If not enabled, apply 5% brake force to engine
    throttleServo.writeMicroseconds(throttle_stop); 
  } else if((positionTarget != 0) && (abs(odometer) >= abs(positionTarget)))
  {
    // Since we have achieved the target, apply 5% braking force
    throttleServo.writeMicroseconds(throttle_stop);
    positionTarget = 0;
    odometer = 0;  
    throttleTarget = 0;
  } else
  {
    // If we don't need to stop, continue at the perscribed throttle
    throttleServo.writeMicroseconds(throttleTarget + throttle_stop);  
  }
  
  // Check if we should upload a telemetry packet
  if((millis() - lastUpdate) >= 50)
  {
    // Store the current time
    lastUpdate = millis();
    
    // Write if motion is enabled
    Serial.write(&en, 1);
    
    // Write if we are on target
    unsigned char ot = (positionTarget == 0);
    Serial.write(&ot, 1);
    
    // Calculate the current odometer value in centimeters
    float inches = ((float)odometer) * 0.008622;
    Serial.write((unsigned char *) &inches, 4);
    
    // Send the radio values
    Serial.write((unsigned char *) &throttleValue, 2);
    Serial.write((unsigned char *) &steeringValue, 2);
  }
        
  // Reset the watchdog so we don't reset unless this code can't be called???
  wdt_reset();
}

// Int1 (Encoder CHA) ticked    (Note the cool XOR shit in the Servo article)
void encoderTickA()
{ 
  _state = ((encodersPortIn >> encoderCHAInput) ^ (encodersPortIn >> encoderCHBInput)) & 0x01;
  if(_state) {
    odometer++;
  } 
  else { 
    odometer--;
  }
}

// Int0 (Encoder CHB) ticked 
void encoderTickB()
{ 
  _state = ((encodersPortIn >> encoderCHAInput) ^ (encodersPortIn >> encoderCHBInput)) & 0x01;
  if(_state) {
    odometer--;
  } 
  else {
    odometer++;
  }
}

// ISR for the RC values
ISR(PCINT2_vect)
{
  unsigned char s = (radioPortIn >> steeringInput) & 0x03;

  // Check the state of throttle control
  if(_state_t_ == 0 && !(s & 0x02)) {
    _state_t_ = 1;  
  } 
  else if(_state_t_ == 1 && (s & 0x02)) {
    sT = micros();
    _state_t_ = 2;
  } 
  else if(_state_t_ == 2 && !(s & 0x02)) {
    throttleValue = micros() - sT; 
    _state_t_ = 1;
  }

  // Check the state of the steering control
  if(_state_s_ == 0 && !(s & 0x01)) {
    _state_s_ = 1;  
  } 
  else if(_state_s_ == 1 && (s & 0x01)) {
    sS = micros();
    _state_s_ = 2;
  } 
  else if(_state_s_ == 2 && !(s & 0x01)) {
    steeringValue = micros() - sS; 
    _state_s_ = 1;
  }
}




