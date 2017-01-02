#include "Config.h"

#include <EEPROM.h>             //Include the EEPROM.h library so we can store information onto the EEPROM
#include <Wire.h>
#include <MyPID.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050_9Axis_MotionApps41.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

byte update_pid_settings_needed = 1;
byte selected_pot_tuning = 0;
byte aserial_data_mode = 0;

float alpha = 0.44;

#define DEBUG
// #define CASCADE_PIDS

MyPID yaw_pid    (0, 0, 0, 0, 0, 200, -200);
MyPID att_pid_ac (0, 0, 0, 0, 0, 200, -200);
MyPID att_pid_bd (0, 0, 0, 0, 0, 200, -200);
MyPID rate_pid_ac(0, 0, 0, 0, 0, 200, -200);
MyPID rate_pid_bd(0, 0, 0, 0, 0, 200, -200);

///////////////////////////////////`
// user inputs
#ifdef CASCADE_PIDS
float input_values[17] = { 0,                       // thrust
                           0.355, 0.000, 0.000,     // Stable Pitch/Role PID P/I/D // .89, 0, .23
                           0.897, 0.146, 0.030,     // Rate Pitch/Role PID P/I/D
                           0.000, 0.000, 0.000,     // Stable Yaw 1.0, 0, 0.2,               // YAW P/I/D
                           0.810, 0.000, 0.000,     // Rate Yaw 1.0, 0, 0.2,                 // YAW P/I/D
                           0.0, // setpoint yaw
                           0.0, // setpoint roll
                           0.0, // setpoint pitch
                           12.6                     // battery voltage level
                           };
#else
float input_values[17] = { 0,                       // thrust
                           1.0,   0.000, 0.100,     // Stable Pitch/Role PID P/I/D // .89, 0, .23
                           0.0,   0.000, 0.000,     // Rate Pitch/Role PID P/I/D
                           0.000, 0.000, 0.000,     // Stable Yaw 1.0, 0, 0.2,               // YAW P/I/D
                           0.810, 0.000, 0.000,     // Rate Yaw 1.0, 0, 0.2,                 // YAW P/I/D
                           0.0, // setpoint yaw
                           0.0, // setpoint roll
                           0.0, // setpoint pitch
                           12.6                     // battery voltage level
                           };
#endif
                  

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
void (*read_mpu)(void);
//
//////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// MPU setup
MPU6050 mpu;

// MPU control/status vars
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

//#ifdef CASCADE_PIDS    
VectorInt16 gyro;
VectorInt16 gyro1;
//#endif

float ypr[3]      = {0.0f, 0.0f, 0.0f};
float ypr_last[3] = {0.0f, 0.0f, 0.0f};
float yw_offset   = 0.0;
float ac_offset   = 0.0;
float bd_offset   = 0.0;
//
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
//
uint16_t va = 0;
uint16_t vb = 0;
uint16_t vc = 0;
uint16_t vd = 0;

uint16_t v_ac = 0;
uint16_t v_bd = 0;
//
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// PID settings
float current_attitude[3]       = {0.0f, 0.0f, 0.0f};
float attitude_correction[3]    = {0.0f, 0.0f, 0.0f}; 
float current_rate[3]           = {0.0f, 0.0f, 0.0f}; 
float rate_correction[3]        = {0.0f, 0.0f, 0.0f};

//
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// MISC items
uint16_t system_check = INIT_CLEARED;

#ifdef DEBUG
unsigned long log_line = 0;
unsigned long last_log = 0;
#endif

struct EEPROMData {
  char id[3];
  int ax_offset;
  int ay_offset;
  int az_offset;
  int gx_offset;
  int gy_offset;
  int gz_offset;
} eeprom_data;


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

  attachInterrupt(digitalPinToInterrupt(3), ppmRising, RISING);  

  process  = &do_blink;
  read_mpu = &do_blink;
  disarm_esc();

  EEPROM.get(0, eeprom_data);
  if(eeprom_data.id[0] == 'A' && eeprom_data.id[1] == 'A') {

    init_i2c();
    init_mpu();
    init_pid();
  
    blink_pattern_4  
    process     = &wait_for_stable_process;
      
  } else {
    blink_pattern_5
    Serial.println("#Please run calibaration");
  }

  pinMode(5, OUTPUT);
}
//////////////////////////////////////////////////////////////////////

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//////////////////////////////////////////////////////////////////////
unsigned long last_tx = 0;
void loop()
{
  do_blink();

  if( millis() - last_tx > 50 ) {
    if( mpuInterrupt ) read_mpu();    
    read_throttle();
    if( mpuInterrupt ) read_mpu();  
    read_setpoint();
    if( mpuInterrupt ) read_mpu();  
    update_pid_settings();

    last_tx = millis();  
  }

  if( mpuInterrupt ) read_mpu();   
  read_battery_voltage();

  if( mpuInterrupt ) read_mpu(); 
  process();
   
  //do_log();
  
}


////////////////////////////////////////////////////////
volatile unsigned long last_ppm_clock = 99999;
volatile unsigned long current_ppm_clock = 0;
volatile unsigned long ppm_dt = 0;
volatile boolean ppm_read = true;
volatile boolean ppm_sync = false;
volatile unsigned short ppm_current_channel = 99;
volatile unsigned long ppm_channels[11] = {0,0,0,0,0,0,0,0,0,0,0};

void ppmRising() {
  ppm_read = false;
    {
      current_ppm_clock = micros();
      ppm_dt = current_ppm_clock - last_ppm_clock;
      if( ppm_dt >= 3500 ) {
        ppm_sync = true;
        ppm_current_channel = 0;
        ppm_channels[ppm_current_channel] = ppm_dt;         
      }
      else {
        if( ppm_sync ) {
          ppm_current_channel++;
          if( ppm_current_channel > 7 ) ppm_sync = false;
          else ppm_channels[ppm_current_channel] = ppm_dt; 
        }
      }
      last_ppm_clock = current_ppm_clock;   
    }
  ppm_read = true;
}
