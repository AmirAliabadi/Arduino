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
float input_values[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 };

uint8_t setpoint_changed = SETPOINT_UNCHANGED;

int thrust = 0;
double setpoint_ac;
double setpoint_bd;
double setpoint_yw;
double last_setpoint_ac;
double last_setpoint_bd;
double last_setpoint_yw;
float pid_ac_kp[2];
float pid_ac_ki[2];
float pid_ac_kd[2];
float pid_bd_kp[2];
float pid_bd_ki[2];
float pid_bd_kd[2];
float pid_yw_kp[2];
float pid_yw_ki[2];
float pid_yw_kd[2];
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
float ypr[3]      = {0.0f, 0.0f, 0.0f};
float ypr_last[3] = {0.0f, 0.0f, 0.0f};

float yw_zero = 0.0;
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
double output_ypr[3] = {0.0, 0.0, 0.0};
double input_ypr[3] = {0.0, 0.0, 0.0};

PID yw_pid(&input_ypr[YW], &output_ypr[YW], &setpoint_yw, 0.7,   0.0001, 0.3,   DIRECT);
PID ac_pid(&input_ypr[AC], &output_ypr[AC], &setpoint_ac, 0.777, 0.0001, 0.333, REVERSE);
PID bd_pid(&input_ypr[BD], &output_ypr[BD], &setpoint_bd, 0.777, 0.0001, 0.333, REVERSE);
//
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// MISC items
long last_mpu_read;

uint16_t system_check = INIT_CLEARED;

#ifdef DEBUG 
long log_line = 0;
long last_log = 0;
#endif
long last_blink = 0;
                         
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

  pid_yw_kp[0] = 0.5;   pid_yw_ki[0] = 0.1;  pid_yw_kd[0] = 0.2;
  pid_ac_kp[0] = 0.66;  pid_ac_ki[0] = 0.4;  pid_ac_kd[0] = 0.1;
  pid_bd_kp[0] = 0.66;  pid_bd_ki[0] = 0.4;  pid_bd_kd[0] = 0.1;

  pid_yw_kp[1] = 0.6;   pid_yw_ki[1] = 0.0;  pid_yw_kd[1] = 0.3;  
  pid_ac_kp[1] = 0.888; pid_ac_ki[1] = 0.0;  pid_ac_kd[1] = 0.222;  
  pid_bd_kp[1] = 0.888; pid_bd_ki[1] = 0.0;  pid_bd_kd[1] = 0.222;  
    
  init_esc();
  init_pid();
  init_i2c();  
  init_mpu();  

  process = &wait_for_stable;

}
//////////////////////////////////////////////////////////////////////


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop()
{ 
  if ( millis() - last_blink > (system_check & INIT_ESC_ARMED == INIT_ESC_ARMED ? (thrust == 0 ? BLINK_FREQUENCY : BLINK_FREQUENCY/2) : BLINK_FREQUENCY/16) )
  {
    last_blink = millis();
    
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));  
  }
  
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
  }

  if ( read_mpu() )
  {
    process() ;
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
  
  pid_ac_kp[0] = input_values[2];
  pid_ac_ki[0] = input_values[3];
  pid_ac_kd[0] = input_values[4];
  
}

