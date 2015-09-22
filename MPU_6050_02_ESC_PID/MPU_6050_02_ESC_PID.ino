#include "Config.h"

#include <Servo.h>
#include <PID_v1.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

#define DEBUG


///////////////////////////////////
// user inputs
uint8_t setpoint_changed = SETPOINT_UNCHANGED;
int thrust = 0;
double setpoint_ac;
double setpoint_bd;
double setpoint_yw;
double last_setpoint_ac;
double last_setpoint_bd;
double last_setpoint_yw;

struct pid_terms {
    float kp;
    float ki;
    float kd;
};
struct user_input_values {
    pid_terms pid_yw[2];
    pid_terms pid_ac[2];
    pid_terms pid_bd[2];
};

user_input_values user_inputs ;
//
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// function pointer to what should be happing in the loop()
void (*process)(void);
//
//////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// MPU setup
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           			// [w, x, y, z]         quaternion container
VectorInt16 aa;         			// [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     			// [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    			// [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    			// [x, y, z]            gravity vector
float euler[3];         			// [psi, theta, phi]    Euler angle container
float ypr[3] = {0.0f, 0.0f, 0.0f};
float ypr_last[3] = {0.0f, 0.0f, 0.0f};
//bool have_first = false;
//
////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////
// ESC 
Servo esc_a;
Servo esc_b;
Servo esc_c;
Servo esc_d;
//
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// PID settings
double output_ac = 0.0;
double output_bd = 0.0;
double output_yw = 0.0;

double input_ypr[3] = {0.0, 0.0, 0.0};

PID yw_pid(&input_ypr[YW], &output_yw, &setpoint_yw, 0.7,   0.0001, 0.3,   DIRECT);
PID ac_pid(&input_ypr[AC], &output_ac, &setpoint_ac, 0.777, 0.0001, 0.333, REVERSE);
PID bd_pid(&input_ypr[BD], &output_bd, &setpoint_bd, 0.777, 0.0001, 0.333, REVERSE);
//
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// MISC items
long last_mpu_read;

uint16_t system_check = INIT_CLEARED;

float input_values[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };

#ifdef DEBUG 
long log_line = 0;
long last_log;
#endif
                         
////////////////////////////////////////////////////////////////


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

//////////////////////////////////////////////////////////////////////
// setup
void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial);
#endif

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  user_inputs.pid_yw[0].kp = 0.5;
  user_inputs.pid_yw[0].ki = 0.1;
  user_inputs.pid_yw[0].kd = 0.2;
  user_inputs.pid_ac[0].kp = 0.66;
  user_inputs.pid_ac[0].ki = 0.1;
  user_inputs.pid_ac[0].kd = 0.1;
  user_inputs.pid_bd[0].kp = 0.66;
  user_inputs.pid_bd[0].ki = 0.1;
  user_inputs.pid_bd[0].kd = 0.1;

  user_inputs.pid_yw[1].kp = 0.6;
  user_inputs.pid_yw[1].ki = 0.0;
  user_inputs.pid_yw[1].kd = 0.3;  
  user_inputs.pid_ac[1].kp = 0.8;
  user_inputs.pid_ac[1].ki = 0.0;
  user_inputs.pid_ac[1].kd = 0.2;  
  user_inputs.pid_bd[1].kp = 0.8;
  user_inputs.pid_bd[1].ki = 0.0;
  user_inputs.pid_bd[1].kd = 0.2;    

  input_values[0] = 0.0;
  input_values[1] = 0.0;
  input_values[2] = user_inputs.pid_ac[0].kp;
  input_values[3] = user_inputs.pid_ac[0].ki;
  input_values[4] = user_inputs.pid_ac[0].kd;

  init_esc();
  init_pid();
  init_i2c();  
  init_mpu();  

  process = &wait_for_stable;

}
//////////////////////////////////////////////////////////////////////


int b = 0;
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop()
{
  
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
  }

  if ( read_mpu() )
  {
    if( thrust > 0 ) 
    { 
        process() ;
    }
    else {
      if (millis() - last_log > LOG_FREQUENCY)
      {
        last_log = millis();
        Serial.print(ypr[0],4);
        Serial.print("\t");
        Serial.print(ypr[1],4);    
        Serial.print("\t");
        Serial.print(ypr[2],4);    
        Serial.print("\t");
        Serial.println();
    
        if( abs(ypr[2] - ypr_last[2]) > 30 )
        {
          b = 1;
        }
    
        if( b == 1 )
        {
          Serial.println("big change !!!");      
        }
      }
    }
    
  }
  else
  {
    // mpu was not read
    if ( millis() - last_mpu_read > LOG_FREQUENCY )
    {
      // no sucessful mpu reads for awhile
      // something is wrong

      //disarm_esc();
    }
  }
}

void serialEvent() {
  readCsvToVector(input_values);

  thrust = constrain(input_values[0], MIN_INPUT_THRUST, MAX_INPUT_THRUST);  // todo: determine max when arming

  if( setpoint_ac != input_values[1] ) 
  {  
    setpoint_changed |= SETPOINT_CHANGED_AC;
    last_setpoint_ac = setpoint_ac; 
    setpoint_ac = input_values[1];
  }
  /*
  if( user_inputs.last_setpoint.bd != input_values[1] ) 
  {  
    user_inputs.setpoint_changed |= SETPOINT_CHANGED_BD;
    user_inputs.last_setpoint.bd = user_inputs.setpoint.bd; 
    user_inputs.setpoint.bd = xxx_input_values[1];    
  }
  if( user_inputs.last_setpoint.yw != input_values[1] ) 
  {  
    user_inputs.setpoint_changed |= SETPOINT_CHANGED_YW;
    user_inputs.last_setpoint.yw = user_inputs.setpoint.yw; 
    user_inputs.setpoint.yw = xxx_input_values[1];    
  }
  */
  
  user_inputs.pid_ac[0].kp = input_values[2];
  user_inputs.pid_ac[0].ki = input_values[3];
  user_inputs.pid_ac[0].kd = input_values[4];
  
}

