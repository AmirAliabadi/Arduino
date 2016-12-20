#include "Config.h"

#include <EEPROM.h>             //Include the EEPROM.h library so we can store information onto the EEPROM

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

float alpha = 0.44;

#define DEBUG
//#define CASCADE_PIDS


class PID {
  public:
  
  PID(float p, float i, float d, float ref, float in, float maxCtrl, float minCtrl) {
    Kp = p;
    Ki = i;
    Kd = d;
    prevTime = millis();
    prevRef = ref;
    prevInput = in;
    
    minLimit = minCtrl;
    maxLimit = maxCtrl;
    
    iTerm = 0;
  }
  
  void resetITerm() {
    iTerm = 0;
    prevTime = micros();
  }

  float calculate(float ref, float input) {
    // Calculate sampling time
    unsigned long dt = (micros() - prevTime); // Convert to seconds
    float dt_float = dt * 0.001 ;
    
    float error = ref - input;
    pTerm = Kp * (ref - input);
    dTerm = -Kd * (input - prevInput)/dt_float; // dError/dt = - dInput/dt
    iTerm += Ki * error * dt;
    
    // Calculate control
    float output = pTerm + iTerm + dTerm;
    
    // Anti-windup
    if (output > maxLimit) {
      iTerm -= output - maxLimit;
      output = maxLimit;
    } else if ( output < minLimit ){
      iTerm += minLimit - output;
      output = minLimit;
    } else {
      //output is output
    }
    
    // Update state
    prevTime = micros();
    prevRef = ref;
    prevInput = input;
    
    return output;
  }

  void setControlCoeffs(float* pidVector) {
    Kp = pidVector[0];
    Ki = pidVector[1];
    Kd = pidVector[2];
  }
  
  private:
  unsigned long prevTime;
  float prevRef;
  float prevInput;
  float pTerm;
  float iTerm;
  float dTerm;
  float minLimit;
  float maxLimit;
  float Kp, Ki, Kd;
};


PID yaw_pid    (0, 0, 0, 0, 0, 300, 300);
PID att_pid_ac (0, 0, 0, 0, 0, 300, 300);
PID att_pid_bd (0, 0, 0, 0, 0, 300, 300);
PID rate_pid_ac(0, 0, 0, 0, 0, 300, 300);
PID rate_pid_bd(0, 0, 0, 0, 0, 300, 300);


long last_blink = 0;
int blink_point = 0;
int blink_pattern[4] = {1000,1000,1000,1000};
#define blink_reset blink_point = 0; last_blink = millis(); digitalWrite(LED_PIN, LOW); 
#define blink_pattern_1 blink_reset blink_pattern[0] = 1000; blink_pattern[1] = 1000;  blink_pattern[2] = 1000;  blink_pattern[3] = 1000;
#define blink_pattern_2 blink_reset blink_pattern[0] = 1000; blink_pattern[1] = 200;   blink_pattern[2] = 1000;  blink_pattern[3] = 200;
#define blink_pattern_3 blink_reset blink_pattern[0] = 1000; blink_pattern[1] = 200;   blink_pattern[2] = 200;   blink_pattern[3] = 1000;
#define blink_pattern_4 blink_reset blink_pattern[0] = 1000; blink_pattern[1] = 200;   blink_pattern[2] = 200;   blink_pattern[3] = 200;
#define blink_pattern_5 blink_reset blink_pattern[0] = 100;  blink_pattern[1] = 100;   blink_pattern[2] = 100;   blink_pattern[3] = 100;

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
long log_line = 0;
long last_log = 0;
#endif
long stable_check_loop = 0;

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

//  pinMode(5, OUTPUT);
//  pinMode(6, OUTPUT);
}
//////////////////////////////////////////////////////////////////////

void do_blink()
{
  if( millis() - last_blink > blink_pattern[blink_point] )
  {
    if(++blink_point > 3) blink_point = 0;
    
    last_blink = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));    
  }
}

void loop()
{
//  do_blink();

  read_mpu();
  read_throttle();
  read_setpoint();
  read_battery_voltage();
  update_pid_settings();
  process();

  do_log();
}

