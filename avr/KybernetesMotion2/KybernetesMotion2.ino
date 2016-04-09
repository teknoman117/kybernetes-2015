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

template<typename T> T clamp(T x, T a, T b)
{
  if(x > b)
    return b;
  else if(x < a)
    return a;

  return x;
}

// velocity pid controller (see http://www2.widener.edu/~crn0001/Engr314/Digital%20PID%20Controllers-2.pdf)
struct pid_t
{
  // Device state
  double Target;
  double Output;
  
  double Input;
  double InputT_1;
  double InputT_2;

  double OutputMaximum;
  double OutputMinimum;
  
  double Kc;
  double tI;
  double tD;

  // State
  bool   enabled;
  int    lastUpdate;
  int    updateInterval;

  // Constructor sets default settings
  pid_t() 
  {
    OutputMinimum = -350;
    OutputMaximum = 350;

    Kc = 0.2;
    tI = 0.5;
    tD = 0.1;

    enabled        = false;
    updateInterval = 20;
  }

  // Reset the state to "prior to movement"
  void Reset(int now)
  {
    Target     = 0.0;
    Output     = 0.0;
    Input      = 0.0;
    InputT_1   = 0.0;
    InputT_2   = 0.0;

    lastUpdate = now;
  }

  // Set enabled helper function - calls reset when switching from disabled to enabled
  void SetEnabled(int now, bool enabled_)
  {
    if(enabled_ && enabled_ != enabled)
      Reset(now);

    enabled = enabled_;
  }

  // Set tunings
  void SetTunings(int now, double p, double i, double d)
  {
    double updateIntervalSeconds = ((double)updateInterval)/1000.0;

    // Compute constants
    Kc = p;
    tI = updateIntervalSeconds / i;
    tD = d / updateIntervalSeconds;
    
    Reset(now);
  }

  // Compute 
  bool Compute(int now, bool debug)
  {
    if(!enabled || now - lastUpdate < updateInterval)
      return false;

    int hurr = micros();
    double e = Target - Input;

    double pTerm = InputT_1 - Input;
    double iTerm = tI*e;
    double dTerm = -tD*(Input - 2*InputT_1 + InputT_2);
    
    Output += Kc * (pTerm + iTerm + dTerm);
    Output = clamp(Output, OutputMinimum, OutputMaximum);
    int duration = micros() - hurr;
    
    if(debug)
    {
      Serial.print("DEBUG:PIDINFO;");
      Serial.print(Target);
      Serial.print(";");
      Serial.print(Output);
      Serial.print(";");
      Serial.print(e);
      Serial.print(";");
      Serial.print(pTerm);
      Serial.print(";");
      Serial.print(iTerm);
      Serial.print(";");
      Serial.print(dTerm);
      Serial.print(";");
      Serial.print(Input);
      Serial.print(";");
      Serial.print(InputT_1);
      Serial.print(";");
      Serial.print(InputT_2);
      Serial.print(";");
      Serial.println(duration);
    }

    // Update history
    InputT_2 = InputT_1;
    InputT_1 = Input;
    lastUpdate = now;

    return true;
  }
};

// Encoder Inputs
#define encoderCHAInput PIND3
#define encoderCHBInput PIND2
#define encodersDDR     DDRD
#define encodersPortIn  PIND

// Radio Inputs
#define throttleInput   PIND7
#define steeringInput   PIND6
#define radioDDR        DDRD
#define radioPortIn     PIND

// Servos Outputs and Centers
#define throttleOutput  5
#define steeringOutput  4
#define throttle_stop   1525
#define steering_center 1550

// State Settings
#define  armThreshold 1700
#define  timeout      2500

typedef enum _arm_state_t : unsigned char
{
  Killed,          // Can not be armed
  Idle,            // Can be armed
  Armed,           // Currently armed
  Disarming,       // In the process of disarming
} ArmState;

/*// We need to remember what directional mode we are in for braking purposes
typedef enum _direction_state_t : unsigned char
{
  Forward, 
  Reverse
} DirectionState;*/

// Encoder State
volatile int16_t encoder;

// Servo state
Servo       steeringServo;
short       steeringTarget;
Servo       throttleServo;
pid_t       throttlePID;
bool        debugMode = false;

// Radio state
volatile short         throttleValue = 1500;
volatile short         steeringValue = 1500;
volatile unsigned long sS, sT;
volatile unsigned char _state_s_ = 0, _state_t_ = 0;

// Arming state
ArmState state;
int      lastCommand;
int      lastDisarmCheck;
int      lastHeartbeat;

//-------------------------------------------------------- ARM STATE UTILITIES --------------------------------------------

void EnsureDisarmed(int now)
{
  // Ensure everthing is disabled
  throttleServo.detach();
  steeringServo.detach();
  
  throttlePID.SetEnabled(now, false);
}

void Disarm(int now)
{
  state = Disarming;
  
  // Put the vehicle into braking mode (assuming rock crawler turnigy ESC mode)
  throttleServo.writeMicroseconds(steering_center);
  throttleServo.writeMicroseconds(throttle_stop);

  // Used to check if the vehicle has stopped moving
  lastDisarmCheck = now;
  encoder         = 0;

  // Disable the PID controller
  throttlePID.SetEnabled(now, false);
  
  SendAlert("DISARMING");
}

void Arm(int now)
{
  state = Armed;

  // Reset our variables
  steeringTarget = 0;
  encoder        = 0;
  
  // Reset and enable PID
  throttlePID.SetEnabled(now, true);

  // Restart the servos
  throttleServo.attach(throttleOutput);
  steeringServo.attach(steeringOutput);
  SendAlert("ARMED");
}

//-------------------------------------------------------- ALERT HELPERS --------------------------------------------
 
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

//-------------------------------------------------------- COMMUNICATION HELPERS --------------------------------------------

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

//-------------------------------------------------------- MAIN AREA --------------------------------------------

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
  EnsureDisarmed(millis());
  
  // Initialize the arming system
  state = Killed;
  lastCommand = lastDisarmCheck = lastHeartbeat = millis();
  
  // Ensure interrupts are online
  sei();
  
  // Delay to make sure the motor controller resets
  delay(1000);
  Serial.println("STATUS:READY");
  
  // Turn on the watchdog timer
  wdt_enable(WDTO_120MS);
}

void loop() 
{
  // Get the current time
  int now = millis();

//-------------------------------------------------------- STATE TRANSISTIONS --------------------------------------------

  // Check if we should transition to another state
  if(state == Armed)
  {
    if(throttleValue < armThreshold)
    {
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
    EnsureDisarmed(now);
    if(throttleValue >= armThreshold)
    {
      state = Idle;
      SendAlert("IDLE");
    }    
  }
  
  // If we are in the idle state
  else if(state == Idle)
  {
    EnsureDisarmed(now);
    if(throttleValue < armThreshold)
    {
      state = Killed;
      SendAlert("KILLED");
    } 
  }
  
  // If we are in the disarming state
  else if(state == Disarming)
  {
    // Disarming only finished once the robot has stopped moving
    if(now - lastDisarmCheck > 500)
    {
      lastDisarmCheck = now;
      if(encoder == 0)
      {
        EnsureDisarmed(now);
        state = Killed;
        SendAlert("KILLED");
      }
      encoder = 0;
    } 
  }

//-------------------------------------------------------- COMMAND PROCESSING --------------------------------------------
  
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
         Arm(now);
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
         throttlePID.Target = atof(target);
         Serial.print("VELOCITY:OK;");
       } else
         Serial.print("VELOCITY:FAIL;");
       
       Serial.println(coden); 
    }
    else if(!strcmp(command, "SETPID"))
    {
       // Get the input
       char *kp = strtok_r(params, ";", &saveptr2);
       if(kp == NULL)
         goto commanderror;
       char *ki = strtok_r(NULL, ";", &saveptr2);
       if(ki == NULL)
         goto commanderror;
       char *kd = strtok_r(NULL, ";", &saveptr2);
       if(kd == NULL)
         goto commanderror;
       char *code = strtok_r(NULL, ";", &saveptr2);
       if(code == NULL)
         goto commanderror;
       int coden = atoi(code);

       throttlePID.SetTunings(now, atof(kp), atof(ki), atof(kd));
       
       Serial.print("SETPID:OK;");
       Serial.println(coden); 
    }
    else if(!strcmp(command, "SETDEBUG"))
    {
       // Get the input
       char *on = strtok_r(params, ";", &saveptr2);
       if(on == NULL)
         goto commanderror;
       char *code = strtok_r(NULL, ";", &saveptr2);
       if(code == NULL)
         goto commanderror;
       int onv = atoi(on);
       int coden = atoi(code);

       debugMode = (onv > 0);
       
       Serial.print("SETDEBUG:OK;");
       Serial.println(coden); 
    }
    lastCommand = now;
  }
  
commanderror:

  // Assign the servo's control values from the PID controller
  throttlePID.Input = static_cast<double>(encoder);
  if(throttlePID.Compute(now, debugMode))
  {
    encoder = 0;
    throttleServo.writeMicroseconds(static_cast<short>(throttlePID.Output) + throttle_stop);
    steeringServo.writeMicroseconds(steeringTarget + steering_center);
  }
  
  // Reset the watchdog so we don't reset unless this code can't be called???
  SendHeartbeat(now);
  wdt_reset();
}

//-------------------------------------------------------- INTERRUPT SERVICE ROUTINES --------------------------------------------

// Int1 (Encoder CHA) ticked    (Note the cool XOR shit in the Servo article)
void encoderTickA()
{ 
  encoder += (((encodersPortIn >> encoderCHAInput) ^ (encodersPortIn >> encoderCHBInput)) & 0x01) ? 1 : -1; 
}

// Int0 (Encoder CHB) ticked 
void encoderTickB()
{ 
  encoder += (((encodersPortIn >> encoderCHAInput) ^ (encodersPortIn >> encoderCHBInput)) & 0x01) ? -1 : 1; 
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

