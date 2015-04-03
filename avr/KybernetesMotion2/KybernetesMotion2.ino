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
#include <avr/wdt.h>

// Encoder Inputs
#define encoderCHAInput PIND3
#define encoderCHBInput PIND2
#define encodersDDR     DDRD
#define encodersPortIn  PIND

// Encoder State
unsigned char _state = 0;
int32_t        odometer = 0;

// Servos Pins
#define throttleOutput  5
#define steeringOutput  4
#define throttle_stop   1550
#define steering_center 1550

// Servo state
Servo   steeringServo;
Servo   throttleServo;
short   steeringTarget = 0;
short   throttleTarget = 0;
int32_t positionTarget = 0;

// We need to remember what directional mode we are in for braking purposes
typedef enum _direction_state_t : unsigned char
{
  Forward, 
  Reverse
} DirectionState;

// Radio Inputs
#define throttleInput   PIND7
#define steeringInput   PIND6
#define radioDDR        DDRD
#define radioPortIn     PIND

// Radio state
volatile short         throttleValue = 1500;
volatile short         steeringValue = 1500;
volatile unsigned long sS, sT;
volatile unsigned char _state_s_ = 0, _state_t_ = 0;

// Arming state
typedef enum _arm_state_t : unsigned char
{
  Killed,          // Can not be armed
  Idle,            // Can be armed
  Armed,           // Currently armed
  Disarming,       // In the process of disarming
} ArmState;

#define  armThreshold 1700
#define  timeout 15000
ArmState state;
int      lastCommand;
int      lastCheck;
int      lastHeartbeat;

void EnsureDisarmed()
{
  throttleServo.detach();
  steeringServo.detach();
}

void Disarm(int now)
{
  state = Disarming;
  odometer = 0;
  lastCheck = now;
  steeringTarget = 0;
  throttleTarget = 0;
  throttleServo.writeMicroseconds(throttle_stop);
}

void Arm()
{
  state = Armed;
  steeringTarget = 0;
  throttleTarget = 0;
  throttleServo.attach(throttleOutput);
  steeringServo.attach(steeringOutput);
}

void SendAlert(const char *m)
{
  Serial.print("ALERT:");
  Serial.println(m);
}

void SendHeartbeat(int now)
{
  if(now - lastHeartbeat >= 500) 
  {
     SendAlert("HEARTBEAT");
     lastHeartbeat = now;
  }
}

int readline(int readch, char *buffer, int len)
{
  static int pos = 0;
  int rpos;

  if (readch > 0) {
    switch (readch) {
      case '\n': // Ignore new-lines
        break;
      case '\r': // Return on CR
        rpos = pos;
        pos = 0;  // Reset position index ready for next time
        return rpos;
      default:
        if (pos < len-1) {
          buffer[pos++] = readch;
          buffer[pos] = 0;
        }
    }
  }
  // No end of line has been found, so return -1.
  return -1;
}

void setup() 
{
  // put your setup code here, to run once:
  // Start the serial uplink
  Serial.begin(57600); 
  
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

  // Initalize steering servos
  pinMode(steeringOutput, OUTPUT);
  digitalWrite(steeringOutput, LOW);
  steeringServo.writeMicroseconds(steering_center);
  
  // Ensure we are disarmed
  EnsureDisarmed();
  
  // Initialize the arming system
  state = Killed;
  lastCommand = millis();
  lastCheck = millis();
  lastHeartbeat = millis();
  
  // Ensure interrupts are online
  sei();
  
  // Delay to make sure the motor controller resets
  SendAlert("RESET");
  delay(1000);
  SendAlert("READY");
  
  // Turn on the watchdog timer
  wdt_enable(WDTO_120MS);
}

void loop() 
{
  // Get the current time
  int now = millis();
  
  // Check if we should transition to another state
  if(state == Armed)
  {
      if(throttleValue < armThreshold)
      {
        SendAlert("KILLED");
        Disarm(now);
      }
    
      else if(now - lastCommand >= timeout)
      {
        SendAlert("TIMEOUT");
        Disarm(now);
      }
  }
  
  // Verify the arm/disarm state
  else if(state == Killed)
  {
    EnsureDisarmed();
    if(throttleValue >= armThreshold)
    {
      state = Idle;
      SendAlert("IDLE");
    }    
  }
  
  // If we are in the idle state
  else if(state == Idle)
  {
    EnsureDisarmed();
    if(throttleValue < armThreshold)
    {
      state = Killed;
      SendAlert("KILLED");
    } 
  }
  
  // If we are in the disarming state
  else if(state == Disarming)
  {
    // Check odometer
    if(now - lastCheck > 500)
    {
      lastCheck = now;
      if(odometer == 0)
      {
        EnsureDisarmed();
        state = Killed;
      }
      odometer = 0;
    } 
  }
  
  // Process commands
  static char commandBuffer[128];
  int pos = readline(Serial.read(), commandBuffer, 128);
  if(pos > 0)
  {
    char *saveptr1, *saveptr2;
    char *command = strtok_r(commandBuffer, ":", &saveptr1);
    if(command == NULL)
      goto commanderror;
      
    // Get the parameters string
    char *params = strtok_r(NULL, ":", &saveptr1);
    if(params == NULL)
      goto commanderror;
      
    // Process the command
    if(!strcmp(command, "ARM"))
    {
       // Get the code
       int code = atoi(strtok_r(params, ";", &saveptr2));
       
       // Can we arm?
       if(state == Armed || state == Idle)
       {
         Serial.print("ARM:OK;");
         Serial.println(code); 
         Arm();
       }
       else
       {
         Serial.print("ARM:FAIL;");
         Serial.println(code); 
       }
    }
    else if(!strcmp(command, "DISARM"))
    {
       // Get the code
       int code = atoi(strtok_r(params, ";", &saveptr2));
       
       // Can we arm?
       if(state == Armed)
       {
         Disarm(now);
       }
         
       Serial.print("DISARM:OK;");
       Serial.println(code); 
    }
    else if(!strcmp(command, "ARMSTAT"))
    {
       // Get the code
       int code = atoi(strtok_r(params, ";", &saveptr2));
       
       // Can we arm?
       if(state == Armed)
         Serial.print("ARMSTAT:ARMED;");
       else if(state == Disarming)
         Serial.print("ARMSTAT:DISARMING;");
       else if(state == Idle)
         Serial.print("ARMSTAT:IDLE;");
       else if(state == Killed)
         Serial.print("ARMSTAT:KILLED;");
         
       Serial.println(code); 
    }
    else if(!strcmp(command, "PING"))
    {
       // Get the code
       int code = atoi(strtok_r(params, ";", &saveptr2));
       
       Serial.print("PING:OK;");
       Serial.println(code); 
    }
    else if(!strcmp(command, "STEER"))
    {
       // Get the input
       char *target = strtok_r(params, ";", &saveptr2);
       if(target == NULL)
         goto commanderror;
       char *code = strtok_r(NULL, ";", &saveptr2);
       if(code == NULL)
         goto commanderror;
       int coden = atoi(code);
       
       if(state == Armed)
       {
         steeringTarget = atoi(target);
         Serial.print("STEER:OK;");
       } else
         Serial.print("STEER:FAIL;");
       
       Serial.println(coden); 
    }
    else if(!strcmp(command, "VELOCITY"))
    {
       // Get the input
       char *target = strtok_r(params, ";", &saveptr2);
       if(target == NULL)
         goto commanderror;
       char *code = strtok_r(NULL, ";", &saveptr2);
       if(code == NULL)
         goto commanderror;
       int coden = atoi(code);
       
       if(state == Armed)
       {
         throttleTarget = abs(atoi(target));
         Serial.print("STEER:OK;");
       } else
         Serial.print("STEER:FAIL;");
       
       Serial.println(coden); 
    }
    lastCommand = now;
  }
  commanderror:
  
  throttleServo.writeMicroseconds(throttleTarget + throttle_stop);
  steeringServo.writeMicroseconds(steeringTarget + steering_center);
  
  // Reset the watchdog so we don't reset unless this code can't be called???
  SendHeartbeat(now);
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

