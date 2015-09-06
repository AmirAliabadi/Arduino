#include <Servo.h>
#include <PID_v1.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

#define DEBUG

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define DELAY  50 // DEBUG Logging interval

//////////////////////////////////////////////////////////////////
// function pointer to what should be happing in the loop()
void (*process)(void);
//
//////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// MPU setup
MPU6050 mpu;

// 798	-738	1310	34	31	31
// 952	-584	1464	34	31	31
#define MPU6050_ACCEL_OFFSET_X 952
#define MPU6050_ACCEL_OFFSET_Y -584
#define MPU6050_ACCEL_OFFSET_Z 1464
#define MPU6050_GYRO_OFFSET_X  34
#define MPU6050_GYRO_OFFSET_Y  31
#define MPU6050_GYRO_OFFSET_Z  31

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

/*
                A
                |
                |
        D -----[Y]----- B Pitch
                |
                |Roll
                C
*/
#define YW 0
#define AC 2
#define BD 1
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// throttle
#define THROTTLE_PIN 0

////////////////////////////////////////////////////////////////
// PID Tunning with a POT
#define Kp_PIN 1
#define Ki_PIN 2
#define Kd_PIN 3

////////////////////////////////////////////////////////////////
// ESC Settings
#define ESC_ARM_DELAY 3000
//#define MAX_SIGNAL 2000		// Simulate throttle at full
#define MAX_THRUST 1400   // safety setting while testing.
#define MIN_THRUST 1130   // motor is off below this value
#define MIN_SIGNAL 1100		// Minimum ESC signal to ARM less than or equal to this should turn off motor completely
#define MOTOR_PIN_A 9		// ESC signal wire conected to pin 9
#define MOTOR_PIN_B 111		// ESC signal wire conected to pin ??
#define MOTOR_PIN_C 6		// ESC signal wire conected to pin 6
#define MOTOR_PIN_D 112		// ESC signal wire conected to pin ??

Servo esc_a;
Servo esc_b;
Servo esc_c;
Servo esc_d;
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// PID settings
double setpoint_ac = 0.0;
double setpoint_bd = 0.0;
double setpoint_yw = 0.0;

double output_ac = 0.0;
double output_bd = 0.0;
double output_yw = 0.0;

double input_ypr[3] = {0.0, 0.0, 0.0};

PID yw_pid(&input_ypr[YW], &output_yw, &setpoint_yw, 0.7, 0.950, 0.011, DIRECT);
PID ac_pid(&input_ypr[AC], &output_ac, &setpoint_ac, 2.0, 0.002, 0.7, REVERSE);
PID bd_pid(&input_ypr[BD], &output_bd, &setpoint_bd, 0.7, 0.950, 0.011, REVERSE);
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// MISC items
long last_mpu_read;
long mpu_debug_info_hz;
boolean dmp_ready = false;
boolean esc_ready = false;
boolean pid_ready = false;


float thrust = 0.0;
#define NEUTRAL_THRUST 0.0

float input_values[10];

uint8_t buffer[20]; 
//(1250 - 1100)
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
// main autopilot routine
void process_pilot()
{
  thrust = read_throttle();

  double kp = read_kp();
  double ki = read_ki();
  double kd = read_kd();

  yw_pid.SetTunings(kp, ki, kd);  
  ac_pid.SetTunings(kp, ki, kd);
  bd_pid.SetTunings(kp, ki, kd);

  float va = MIN_SIGNAL;
  float vb = MIN_SIGNAL;
  float vc = MIN_SIGNAL;
  float vd = MIN_SIGNAL;
  
  if(thrust > NEUTRAL_THRUST) {
    yw_pid.Compute();
    ac_pid.Compute();
    bd_pid.Compute();

    //////////////////////////////////////////////////////
    // compute the boom velocity
    /*
    float v_ac = (abs(output_yw - 100) / 100) * thrust;
    float v_bd = (   (output_yw + 100) / 100) * thrust;
    
    // distribute the boom velocity to each boom motor
    float va = ((output_ac + 100) / 100) * v_ac;
    float vb = ((output_bd + 100) / 100) * v_bd;
    
    float vc = (abs((output_ac - 100) / 100)) * v_ac;
    float vd = (abs((output_bd - 100) / 100)) * v_bd;
    */
    //
    //////////////////////////////////////////////////////
    
    float v_ac = thrust;
    float v_bd = thrust;

    va = MIN_SIGNAL + (v_ac + output_ac);
    vc = MIN_SIGNAL + (v_ac - output_ac);
    vb = MIN_SIGNAL + (v_bd + output_bd);
    vd = MIN_SIGNAL + (v_bd - output_bd);

    va = va <= MIN_THRUST ? MIN_SIGNAL : va;
    vc = vc <= MIN_THRUST ? MIN_SIGNAL : vc;
    vb = vb <= MIN_THRUST ? MIN_SIGNAL : vb;
    vd = vd <= MIN_THRUST ? MIN_SIGNAL : vd;
    
    va = va > MAX_THRUST ? MAX_THRUST : va;
    vc = vc > MAX_THRUST ? MAX_THRUST : vc;
    vb = vb > MAX_THRUST ? MAX_THRUST : vb;
    vd = vd > MAX_THRUST ? MAX_THRUST : vd;  
  }

  esc_a.writeMicroseconds(va);
  esc_c.writeMicroseconds(vc);
  //esc_b.writeMicroseconds(b);
  //esc_d.writeMicroseconds(d);

  if (millis() - mpu_debug_info_hz > DELAY)
  {
    mpu_debug_info_hz = millis();
    
#ifdef DEBUG    
    
    //log_pid_tuning(kp,ki,kd);
    //log_data(va, vc);
    // log_graphing_data(va,vc);
    // log_data(input_values);
    //plot(va,vc);
    log_data2(va, vc);
        
    //print_mpu_readings(mode,fifoBuffer);
#endif


    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    
  }
  
}
//////////////////////////////////////////////////////////////////////

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
  
  process = &process_pilot;

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

    readCsvToVector(input_values);
  }
  else
  {
    // mpu was not read
    if ( millis() - last_mpu_read > DELAY )
    {
      // no sucessful mpu reads for awhile
      // something is wrong
    }
  }
}



