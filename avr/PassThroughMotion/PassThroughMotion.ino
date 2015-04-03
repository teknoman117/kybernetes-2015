/*
 *  KybernetesMotion2.ino
 *
 *  Copyright (c) 2015 Nathaniel Lewis
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

// Servos Pins
#define throttleOutput  5
#define steeringOutput  4
#define throttle_stop   1550
#define steering_center 1550

// Servo state
Servo   steeringServo;
Servo   throttleServo;

// Radio Inputs
#define throttleInput   PIND7
#define steeringInput   PIND6
#define radioDDR        DDRD
#define radioPortIn     PIND

// Radio state
short   throttleValue = 1500;
short   steeringValue = 1500;
unsigned long sS, sT;
unsigned char _state_s_ = 0, _state_t_ = 0;

short center = 0;
bool  reverseCapable;
bool  previousDirectionForward;
bool  previouslyMoving;
bool  braking;

void ArmThrottle()
{
  throttleServo.writeMicroseconds(throttle_stop);
  throttleServo.attach(throttleOutput);
}

void ArmSteering()
{
  steeringServo.writeMicroseconds(steering_center);
  steeringServo.attach(steeringOutput);
}

void setup() 
{
  // Start the serial uplink
  Serial.begin(57600); 
  
  // Configure the radio ports
  radioDDR &= ~_BV(throttleInput) & ~_BV(steeringInput);
  sS = sT = micros();
  PCMSK2 = _BV(throttleInput) | _BV(steeringInput);
  PCICR = _BV(PCIE2);
  
  // Power up the motor drivers and get a signal out (a 10k resistor acts as a pull down so the hb25 will fire up)
  pinMode(throttleOutput, OUTPUT);
  digitalWrite(throttleOutput, LOW);
  ArmThrottle();

  // Initalize steering servos
  pinMode(steeringOutput, OUTPUT);
  digitalWrite(steeringOutput, LOW);
  ArmSteering();
  
  // Ensure interrupts are online
  sei();
  
  // Get the center
  delay(1000);
  center = throttleValue;
  reverseCapable = false;
  previousDirectionForward = true;
  previouslyMoving = false;
  braking = false;
}

void loop() 
{
  short t = throttleValue - center;
  if(abs(t) < 20) t = 0;
  
  steeringServo.writeMicroseconds(steeringValue);
  throttleServo.writeMicroseconds((throttleValue - center) + throttle_stop);
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

