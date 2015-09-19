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
float ypr_offset[3] = {0.0f, 0.0f, 0.0f};
float ypr_last[3] = {0.0f, 0.0f, 0.0f};

bool have_first = false;


////////////////////////////////////////////////////////////////
// ESC 
Servo esc_a;
Servo esc_b;
Servo esc_c;
Servo esc_d;

float va = MIN_ESC_SIGNAL;
float vb = MIN_ESC_SIGNAL;
float vc = MIN_ESC_SIGNAL;
float vd = MIN_ESC_SIGNAL;

float v_ac = 0;
float v_bd = 0;
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// PID settings
double setpoint_ac = 0.0;
double setpoint_bd = 0.0;
double setpoint_yw = 0.0;

double last_setpoint_ac = 0.0;
double last_setpoint_bd = 0.0;
double last_setpoint_yw = 0.0;

double output_ac = 0.0;
double output_bd = 0.0;
double output_yw = 0.0;

double input_ypr[3] = {0.0, 0.0, 0.0};

PID yw_pid(&input_ypr[YW], &output_yw, &setpoint_yw, 0.7, 0.950, 0.011, DIRECT);
PID ac_pid(&input_ypr[AC], &output_ac, &setpoint_ac, .5, 0.0055, 0.201, REVERSE);
PID bd_pid(&input_ypr[BD], &output_bd, &setpoint_bd, .5, 0.0055, 0.201, REVERSE);

double kp = 0;
double ki = 0;
double kd = 0;
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// MISC items
long last_mpu_read;

boolean dmp_ready = false;
boolean esc_ready = false;
boolean pid_ready = false;

float thrust = 0.0;

// thrust, setpoint_ac, Kp, Ki, Kd, 
float input_values[10] = {0.0,0.0,
                          0.5,0.055,0.201,
                          0.0,0.0,0.0,0.0,0.0};

#ifdef DEBUG 
long log_line = 0;
long last_log;
#endif
                          

// float plotter_packet[11] = {0,0,0,0,0,0,0,0,0,0,0,0,0};
                          
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
  
  delay(100);
  
  init_esc();
  init_pid();  
  
  process = &balance_process;

}
//////////////////////////////////////////////////////////////////////

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
    process();
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
}



