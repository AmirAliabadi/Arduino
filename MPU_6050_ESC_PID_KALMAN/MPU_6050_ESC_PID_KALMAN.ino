#include <Servo.h>
#include <PID_v1.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <Kalman.h>

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
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// 798	-738	1310	34	31	31
// 952	-584	1464	34	31	31
#define MPU6050_ACCEL_OFFSET_X 952
#define MPU6050_ACCEL_OFFSET_Y -584
#define MPU6050_ACCEL_OFFSET_Z 1464
#define MPU6050_GYRO_OFFSET_X  34
#define MPU6050_GYRO_OFFSET_Y  31
#define MPU6050_GYRO_OFFSET_Z  31

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
//
////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////
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
// ESC Settings
#define ESC_ARM_DELAY 3000
#define MAX_SIGNAL 2000		// Simulate throttle at full
#define MAX_THRUST 1350   // safety setting while testing.
#define MIN_SIGNAL 1110		// Minimum ESC signal to ARM less than or equal to this should turn off motor completely
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

//Specify the links and initial tuning parameters
double input_ypr[3] = {0.0, 0.0, 0.0};
PID yw_pid(&input_ypr[YW], &output_yw, &setpoint_yw, 2.0, .1, .75, DIRECT);
PID ac_pid(&input_ypr[AC], &output_ac, &setpoint_ac, 2.0, .1, .75, REVERSE);
PID bd_pid(&input_ypr[BD], &output_bd, &setpoint_bd, 2.0, .1, .75, REVERSE);
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// MISC items
long last_mpu_read;
long mpu_debug_info_hz;
boolean dmp_ready = false;
boolean dmp_stable = false;
boolean esc_ready = false;
boolean pid_ready = false;

float calib_y, calib_p, calib_r;
float calib_yi, calib_pi, calib_ri;
int calib_index;
float thrust = 0.0;

#define NEUTRAL_THRUST 130
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
  
  ac_pid.SetTunings(read_kp(), read_ki(), read_kd());
  yw_pid.SetTunings(read_kp(), read_ki(), read_kd());
  bd_pid.SetTunings(read_kp(), read_ki(), read_kd());
  
  if(thrust >= NEUTRAL_THRUST) {
    yw_pid.Compute();
    ac_pid.Compute();
    bd_pid.Compute();
  }

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

  float a = (MIN_SIGNAL + thrust) + (output_ac / 2.0);
  float c = (MIN_SIGNAL + thrust) - (output_ac / 2.0);
  float b = (MIN_SIGNAL + thrust) + (output_bd / 2.0);
  float d = (MIN_SIGNAL + thrust) - (output_bd / 2.0);

  a = a <= MIN_SIGNAL ? 0.0 : a;
  c = c <= MIN_SIGNAL ? 0.0 : c;
  b = b <= MIN_SIGNAL ? 0.0 : b;
  d = d <= MIN_SIGNAL ? 0.0 : d;

  a = a > MAX_THRUST ? MAX_THRUST : a;
  c = c > MAX_THRUST ? MAX_THRUST : c;
  b = b > MAX_THRUST ? MAX_THRUST : b;
  d = d > MAX_THRUST ? MAX_THRUST : d;  

  esc_a.writeMicroseconds(a);
  esc_c.writeMicroseconds(c);
  //esc_b.writeMicroseconds(b);
  //esc_d.writeMicroseconds(d);

#ifdef DEBUG
  if (millis() - mpu_debug_info_hz > DELAY)
  {
    mpu_debug_info_hz = millis();

    Serial.print(thrust, 4);
    Serial.print("\t");
    Serial.print(input_ypr[AC], 2);
    Serial.print("\t");
    Serial.print(output_ac, 2);
    Serial.print("\t");
    Serial.print(a, 4);
    Serial.print("\t");
    Serial.print(c, 4);
    Serial.print("\t");
    Serial.print(ac_pid.pterm);
    Serial.print("\t");
    Serial.print(ac_pid.iterm);
    Serial.print("\t");
    Serial.print(ac_pid.dterm);
    Serial.print("\n");

    //print_mpu_readings(mode,fifoBuffer);

    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
#endif
}
//////////////////////////////////////////////////////////////////////

void calibrate_mpu()
{
  init_pid();
  Serial.print(setpoint_ac);Serial.print("\t");
  Serial.print(setpoint_bd);Serial.print("\t");
  Serial.println(setpoint_yw);
  init_esc();
  process = &process_pilot;
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

  process = &calibrate_mpu;

}
//////////////////////////////////////////////////////////////////////

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop()
{
  if (!dmp_ready) return;

  if ( read_mpu() )
  {
    process();
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



