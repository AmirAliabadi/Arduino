#include "Config.h"

//#include "MyServo.h"
//#define _ESC_ MyServo

#include "ESC.h"
#define _ESC_ ESC


#include <PID_v1.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050_9Axis_MotionApps41.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

byte update_pid_settings_needed = 1;
byte selected_pot_tuning = 0;
byte aserial_data_mode = 0;

float alpha = 0.88;
int pid_refresh_rate = 10;

#define DEBUG
//#define USE_INTERRUPTS
#define CASCADE_PIDS

/*
 * Single Boom PID values
 * ----------------------
 * 5.07, 0.00, 0.00
 * 0.443, 0.034, 0.112
 * 
 * 2.49, 0.00, 0.00       works but slugish
 * 0.376, 0.004, 0.004
 * 
 * alpha 0.02, pid rate 20
 * 2.615, 0.085, 0.065
 * 0.41, 0, 0.004
 * 
 * found some checkin notes about the d-term causing chatter
 *                           4.70,  0.0, 0.00,    // Stable P/I/D // .89,0,.23
 *                           1.20,  0.0, 0.16,    // Rate P/I/D
 *                           
 *  3.135, 0.0, 0.0
 *  0.672, 0.01, 0.013
 *  alpha 0.2, PID Refresh 5.0
 * 
 * *
 * Single Boom Quad PID
 * ---------------------
 * 6.885  0.00  0.405
 * 0.551  0.00  0.184
 * 
 * * 2.865 0.00 0.00
 * * 0.429 0.005 0.038
 */

///////////////////////////////////`
// user inputs
float input_values[17] = { 0,                       // thrust
                           3.135, 0.000, 0.000,     // Stable Pitch/Role PID P/I/D // .89, 0, .23
                           0.672, 0.010, 0.013,     // Rate Pitch/Role PID P/I/D
                           2.000, 0.000, 0.000,     // Stable Yaw 1.0, 0, 0.2,                 // YAW P/I/D
                           0.110, 0.000, 0.000,     // Stable Yaw 1.0, 0, 0.2,                 // YAW P/I/D
                           0.0, // setpoint yaw
                           0.0, // setpoint roll
                           0.0, // setpoint pitch
                           12.6                     // battery voltage level
                           };

// PID values:
// 4.7 / 0 / 0 | 1.2 / 0 / 0.16
// 2.681 / 0 / 0 | 0.967 / 0.125 / 0.096                           

#define INPUT_THRUST          input_values[0]

#define INPUT_STB_PID_P       input_values[1]
#define INPUT_STB_PID_I       input_values[2]
#define INPUT_STB_PID_D       input_values[3]

#define INPUT_RAT_PID_P       input_values[4]
#define INPUT_RAT_PID_I       input_values[5]
#define INPUT_RAT_PID_D       input_values[6]

#define INPUT_YAW_PID_P       input_values[7]
#define INPUT_YAW_PID_I       input_values[8]
#define INPUT_YAW_PID_D       input_values[9]

#define INPUT_YAW_RATE_PID_P       input_values[10]
#define INPUT_YAW_RATE_PID_I       input_values[11]
#define INPUT_YAW_RATE_PID_D       input_values[12]

#define INPUT_SETPOINT_YAW    input_values[13]
#define INPUT_SETPOINT_ROLL   input_values[14]
#define INPUT_SETPOINT_PITCH  input_values[15]
#define INPUT_VOLTAGE_LEVEL   input_values[16]

uint8_t setpoint_changed = SETPOINT_UNCHANGED;


float setpoint[3] = {0, 0, 0};
float last_setpoint[3] = {0, 0, 0};

//
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// function pointer to what should be happing in the loop()
void (*process)(void);
void (*send_serial)(byte mode);
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

#ifdef CASCADE_PIDS    
VectorInt16 gyro;
VectorInt16 gyro1;
#endif

float ypr[3]      = {0.0f, 0.0f, 0.0f};
float ypr_last[3] = {0.0f, 0.0f, 0.0f};
float yw_offset   = 0.0;
float ac_offset   = 0.0;
float bd_offset   = 0.0;
//
////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////
// ESC
_ESC_ esc_a;
_ESC_ esc_b;
_ESC_ esc_c;
_ESC_ esc_d;

uint16_t va = MIN_ESC_SIGNAL;
uint16_t vb = MIN_ESC_SIGNAL;
uint16_t vc = MIN_ESC_SIGNAL;
uint16_t vd = MIN_ESC_SIGNAL;

uint16_t v_ac = 0;
uint16_t v_bd = 0;
//
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// PID settings
float input_ypr[3]    = {0.0f, 0.0f, 0.0f};
float output_ypr[3]   = {0.0f, 0.0f, 0.0f};
float input_gyro[3]   = {0.0f, 0.0f, 0.0f};
float output_rate[3]  = {0.0f, 0.0f, 0.0f};

// input / output /setpoint
PID pid_stable[3]  = {
  PID(&input_ypr[YW], &output_ypr[YW], &setpoint[YW], 0, 0, 0, DIRECT),
  PID(&input_ypr[BD], &output_ypr[BD], &setpoint[BD], 0, 0, 0, DIRECT), 
  PID(&input_ypr[AC], &output_ypr[AC], &setpoint[AC], 0, 0, 0, DIRECT)
};

#ifdef CASCADE_PIDS
PID pid_rate[3] = {
  PID(&input_gyro[YW], &output_rate[YW], &output_ypr[YW], 0, 0, 0, DIRECT),
  PID(&input_gyro[BD], &output_rate[BD], &output_ypr[BD], 0, 0, 0, DIRECT),
  PID(&input_gyro[AC], &output_rate[AC], &output_ypr[AC], 0, 0, 0, DIRECT)
};
#endif

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


//// ================================================================
//// ===               INTERRUPT DETECTION ROUTINE                ===
//// ================================================================
//
#ifdef USE_INTERRUPTS
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;

//  read_mpu();
//  read_throttle();
//  read_setpoint();
//  read_battery_voltage();
//  update_pid_settings();
//  
//  process();  
}
#endif

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
  
  disarm_esc();
  init_i2c();
  init_pid();
  init_mpu();

  process = &wait_for_stable_process; //check_if_stable_process; //arm_esc_process;

  send_serial = &SerialSend_A;
}
//////////////////////////////////////////////////////////////////////


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop()
{
  if ( millis() - last_blink > (system_check & INIT_ESC_ARMED == INIT_ESC_ARMED ? (INPUT_THRUST == 0 ? BLINK_FREQUENCY : BLINK_FREQUENCY / 2) : BLINK_FREQUENCY / 16) )
  {
    last_blink = millis();

    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  if (!dmpReady) return;

#ifdef USE_INTERRUPTS
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt) // && fifoCount < packetSize)
  {
    read_throttle();        if (mpuInterrupt) break;
    read_setpoint();        if (mpuInterrupt) break;
    read_battery_voltage(); if (mpuInterrupt) break;
    update_pid_settings();  if (mpuInterrupt) break;

    process();
  }  
#endif

  read_mpu();
  read_throttle();
  read_setpoint();
  read_battery_voltage();
  update_pid_settings();
  
  process();

}


