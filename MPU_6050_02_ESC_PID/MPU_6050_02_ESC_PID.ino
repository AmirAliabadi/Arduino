#include "Config.h"

#include <Servo.h>
#include <PID_v1.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050_9Axis_MotionApps41.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

byte selected_pot_tuning = 0;
byte serial_data_mode = 0;

#define DEBUG

///////////////////////////////////
// user inputs
float input_values[14] = { 0,                       // thrust
                           3.0,  0.0, 0.00,       // Stable P/I/D // .89,0,.23
                           0.96,  0.0, 0.096,    // i was .125 Rate P/I/D
                           1.0, 0, 0.2,                 // YAW P/I/D
                           12.6,                    // battery voltage level
                           0.0,
                           0.0,
                           0.0 };

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
#define INPUT_VOLTAGE_LEVEL   input_values[10]
#define INPUT_SETPOINT_YAW    input_values[11]
#define INPUT_SETPOINT_ROLL   input_values[12]
#define INPUT_SETPOINT_PITCH  input_values[13]

uint8_t setpoint_changed = SETPOINT_UNCHANGED;


float setpoint[3] = {0, 0, 0};
float last_setpoint[3] = {0, 0, 0};

/*
int thrust = 0;
float voltage = 12.6;
float pid_xx_kp[2];
float pid_xx_ki[2];
float pid_xx_kd[2];

float pid_yw_kp[2];
float pid_yw_ki[2];
float pid_yw_kd[2]; */
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
VectorInt16 gyro;

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
float input_ypr[3]    = {0.0f, 0.0f, 0.0f};
float output_ypr[3]   = {0.0f, 0.0f, 0.0f};
float input_gyro[3]   = {0.0f, 0.0f, 0.0f};
float output_rate[3]  = {0.0f, 0.0f, 0.0f};

// input / output /setpoit
PID yw_pid(&input_ypr[YW], &output_ypr[YW], &setpoint[YW], 0, 0, 0, DIRECT);

PID ac_pid(&input_ypr[AC], &output_ypr[AC], &setpoint[AC], 0, 0, 0, DIRECT);
PID bd_pid(&input_ypr[BD], &output_ypr[BD], &setpoint[BD], 0, 0, 0, DIRECT);

PID ac_rat(&input_gyro[AC], &output_rate[AC], &output_ypr[AC], 0, 0, 0, DIRECT);
PID bd_rat(&input_gyro[BD], &output_rate[BD], &output_ypr[BD], 0, 0, 0, DIRECT);

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
  Serial.begin(115200); //115200
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
  if ( millis() - last_blink > (system_check & INIT_ESC_ARMED == INIT_ESC_ARMED ? (INPUT_THRUST == 0 ? BLINK_FREQUENCY : BLINK_FREQUENCY / 2) : BLINK_FREQUENCY / 16) )
  {
    last_blink = millis();

    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt) // && fifoCount < packetSize)
  {  
    read_throttle();        if (mpuInterrupt) break;
    
    if( serial_data_mode == 0 ) read_setpoint(AC);
    else if( serial_data_mode == 1 ) read_setpoint(BD);
    else if( serial_data_mode == 1) read_setpoint(YW);
    
    read_pid_tunings(0);    if (mpuInterrupt) break;
    //read_pid_tunings(1);    if (mpuInterrupt) break;

    read_battery_voltage(); if (mpuInterrupt) break;
    process();
  }

  read_mpu();
  read_throttle();
  if( serial_data_mode == 0 ) read_setpoint(AC);
  else if( serial_data_mode == 1 ) read_setpoint(BD);
  else if( serial_data_mode == 1) read_setpoint(YW);

  read_pid_tunings(0);
  //read_pid_tunings(1);
  
  read_battery_voltage();
  process();

}

void serialEvent() {
  // readCsvToVector();
  SerialReceive();
}

