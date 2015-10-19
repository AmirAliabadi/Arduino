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
float input_values[12] = { 0.0, // thrust
                        2.5, 0.1, 0.75,  // Conservative P/I/D
                        2.5, 0.1, 0.75,  // AGGRESSIVE P/I/D
                        2.5, 0.0, 1.0, // YAW P/I/D 
                        12.6,
                        0.0 }; // battery voltage level

#define INPUT_THRUST          input_values[0]
#define INPUT_CON_PID_P       input_values[1]
#define INPUT_CON_PID_I       input_values[2]
#define INPUT_CON_PID_D       input_values[3]
#define INPUT_AGG_PID_P       input_values[4]
#define INPUT_AGG_PID_I       input_values[5]
#define INPUT_AGG_PID_D       input_values[6]
#define INPUT_YAW_PID_P       input_values[7]
#define INPUT_YAW_PID_I       input_values[8]
#define INPUT_YAW_PID_D       input_values[9]
#define INPUT_VOLTAGE_LEVEL   input_values[10]
#define INPUT_SETPOINT_PITCH  input_values[11]


uint8_t setpoint_changed = SETPOINT_UNCHANGED;

int thrust = 0;
float voltage = 12.6;
float setpoint[3] = {0,0,0};
float last_setpoint[3] = {0,0,0};
float pid_xx_kp[2];
float pid_xx_ki[2];
float pid_xx_kd[2];

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
int32_t gyro1[3];
//int16_t gyro2[3];
int16_t ax, ay, az, gx, gy, gz;

float ypr[3]      = {0.0f, 0.0f, 0.0f};
float ypr_last[3] = {0.0f, 0.0f, 0.0f};
float yw_offset   = 0.0;
//
////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////
// ESC 
Servo esc_a;
Servo esc_b;
Servo esc_c;
Servo esc_d;

int va = MIN_ESC_SIGNAL;
int vb = MIN_ESC_SIGNAL;
int vc = MIN_ESC_SIGNAL;
int vd = MIN_ESC_SIGNAL;
//
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// PID settings
float input_ypr[3] = {0.0, 0.0, 0.0};
float output_ypr[3] = {0.0, 0.0, 0.0};

PID yw_pid(&input_ypr[YW], &output_ypr[YW], &setpoint[YW], 0, 0, 0, DIRECT);
PID ac_pid(&input_ypr[AC], &output_ypr[AC], &setpoint[AC], 0, 0, 0, REVERSE);
PID bd_pid(&input_ypr[BD], &output_ypr[BD], &setpoint[BD], 0, 0, 0, REVERSE);
//
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// MISC items
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

  init_i2c();  
  init_mpu();  
  init_pid();  
  // init_esc();  

  process = &arm_esc_process;
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
  while (!mpuInterrupt) // && fifoCount < packetSize)
  {
    read_throttle();        if(mpuInterrupt) break;
    //read_setpoint(AC);      if(mpuInterrupt) break;
    //read_setpoint(BD);      if(mpuInterrupt) break;
    //read_setpoint(YW);      if(mpuInterrupt) break;    
    read_pid_tunings(0);    if(mpuInterrupt) break;
    read_pid_tunings(1);    if(mpuInterrupt) break;
    read_battery_voltage(); if(mpuInterrupt) break;    
    
    process();
  }

  read_mpu();
  
  read_throttle();
  //read_setpoint(AC);
  //read_setpoint(BD);
  //read_setpoint(YW);
  read_battery_voltage();
  read_pid_tunings(0);
  read_pid_tunings(1);
  
  process();

}

void serialEvent() {
  readCsvToVector();
}

