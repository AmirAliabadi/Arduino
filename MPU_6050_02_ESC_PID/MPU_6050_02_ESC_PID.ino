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
PID yw_pid(&input_ypr[YW], &output_yw, &setpoint_yw, 1.5, .1, .75, DIRECT);
PID ac_pid(&input_ypr[AC], &output_ac, &setpoint_ac, 1.5, .1, .75, REVERSE);
PID bd_pid(&input_ypr[BD], &output_bd, &setpoint_bd, 1.5, .1, .75, REVERSE);

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


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void init_i2c()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

void init_mpu()
{
  if (!dmp_ready)
  {
    Serial.println(F("Initializing MPU I2C connection..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(MPU6050_ACCEL_OFFSET_X);
    mpu.setYAccelOffset(MPU6050_ACCEL_OFFSET_Y);
    mpu.setZAccelOffset(MPU6050_ACCEL_OFFSET_Z);
    mpu.setXGyroOffset(MPU6050_GYRO_OFFSET_X);
    mpu.setYGyroOffset(MPU6050_GYRO_OFFSET_Y);
    mpu.setZGyroOffset(MPU6050_GYRO_OFFSET_Z);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();

      mpu_debug_info_hz = last_mpu_read = millis();

      dmp_ready = true;
    }
    else
    {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
  }
}

void init_pid()
{
  if ( !pid_ready )
  {
    //turn the PID on
    yw_pid.SetOutputLimits(-255.0, 255.0);
    ac_pid.SetOutputLimits(-255.0, 255.0);
    bd_pid.SetOutputLimits(-255.0, 255.0);

    setpoint_ac = 0.0f ;
    setpoint_bd = 0.0f ;
    setpoint_yw = 0.0f ;

    //turn the PID on
    yw_pid.SetMode(AUTOMATIC);
    ac_pid.SetMode(AUTOMATIC);
    bd_pid.SetMode(AUTOMATIC);

    pid_ready = true;
  }
}

void arm_esc()
{
  esc_a.writeMicroseconds(MIN_SIGNAL + read_throttle());
  //esc_b.writeMicroseconds(MIN_SIGNAL+ read_throttle());
  esc_c.writeMicroseconds(MIN_SIGNAL + read_throttle());
  //esc_d.writeMicroseconds(MIN_SIGNAL+ read_throttle());

  delay(ESC_ARM_DELAY);
}

void init_esc()
{
  if ( !esc_ready )
  {
    Serial.println(F("Attaching to motor pins"));
    esc_a.attach(MOTOR_PIN_A);
    //esc_b.attach(MOTOR_PIN_B);
    esc_c.attach(MOTOR_PIN_C);
    //esc_d.attach(MOTOR_PIN_D);

    arm_esc();

    esc_ready = true;
  }
}


void calibrate_mpu()
{
  if ( calib_index > 50 ) {
    calib_y = ypr[0];
    calib_p = ypr[1];
    calib_r = ypr[2];

    if (calib_index == 51) {
      calib_yi = calib_y;
      calib_pi = calib_p;
      calib_ri = calib_r;
    }

    if ( calib_index % 200 == 0 ) {
      Serial.print(calib_index); Serial.print("\t");
      Serial.print(calib_yi); Serial.print("\t"); Serial.print(calib_y); Serial.print("\t");
      Serial.print(calib_pi); Serial.print("\t"); Serial.print(calib_p); Serial.print("\t");
      Serial.print(calib_ri); Serial.print("\t"); Serial.println(calib_r);
    }
  }

  if ( calib_index++ >= 2000 ) {
    ypr_offset[0] = calib_y;
    ypr_offset[1] = calib_p;
    ypr_offset[2] = calib_r;

    dmp_stable = true;

    Serial.println("calibration done.");

    init_pid();
    init_esc();

    process = &process_pilot;
  }
}

bool read_mpu()
{
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();

#ifdef DEBUG
    Serial.println(F("FIFO overflow!"));
#endif
    return false;

  } // otherwise, check for DMP data ready interrupt (this should happen frequently)
  else if (mpuIntStatus & 0x02)
  {
    last_mpu_read = millis();

    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    ypr[YW] = (ypr[YW]) * 180.0 / M_PI ;
    ypr[AC] = (ypr[AC]) * 180.0 / M_PI ;
    ypr[BD] = (ypr[BD]) * 180.0 / M_PI ;
 
    //ypr[AC] = (float)(digitalSmooth( (int)(ypr[AC]*1000.0), sensSmoothArray1 )/1000.0);
    //ypr[BD] = (float)(digitalSmooth( (int)(ypr[BD]*1000.0), sensSmoothArray1 )/1000.0);

    if ( dmp_stable ) {
      ypr[YW] -= ypr_offset[YW];
      ypr[AC] -= ypr_offset[AC];
      ypr[BD] -= ypr_offset[BD];
      
      //ypr[YW] = (float)((int)((((ypr[YW]) * 180.0 / M_PI)*10.0)+.5))/10.0 ;
      //ypr[AC] = (float)((int)((((ypr[AC]) * 180.0 / M_PI)*10.0)+.5))/10.0 ;
      //ypr[BD] = (float)((int)((((ypr[BD]) * 180.0 / M_PI)*10.0)+.5))/10.0 ;
    
    }

    //if (abs(ypr[YW] - ypr_last[YW]) > 30) ypr[YW] = ypr_last[YW];
    //if (abs(ypr[BD] - ypr_last[BD]) > 30) ypr[BD] = ypr_last[BD];
    //if (abs(ypr[AC] - ypr_last[AC]) > 30) ypr[AC] = ypr_last[AC];

    ypr_last[YW] = ypr[YW];
    ypr_last[AC] = ypr[AC];
    ypr_last[BD] = ypr[BD];

    // Update the PID input values
    input_ypr[YW] = ((int)((ypr[YW] * 1.0) + 0.5))/1.0;
    input_ypr[AC] = ((int)((ypr[AC] * 1.0) + 0.5))/1.0;
    input_ypr[BD] = ((int)((ypr[BD] * 1.0) + 0.5))/1.0;
    //input_ypr[YW] = (double)((int)(ypr[YW]+.5));
    //input_ypr[AC] = (double)((int)(ypr[AC]+.5));
    //input_ypr[BD] = (double)((int)(ypr[BD]+.5));

    return true;
  }
  else
  {
    // MPU was not ready
  }

  return false;
}

//////////////////////////////////////////////////////////////////////
// POT Inputs
//////////////////////////////////////////////////////////////////////
float read_throttle()
{
  if( thrust < NEUTRAL_THRUST ) thrust += .10;
  if( thrust >= NEUTRAL_THRUST ) thrust = NEUTRAL_THRUST;
  return thrust;

  return map(analogRead(THROTTLE_PIN), 0.0, 668.0, 0.0, 234.0);
}

double read_kp()
{
  return 2.0;
  double foo = map(analogRead(Kp_PIN), 0.0, 668.0, 0.0, 10000.0);

  foo = foo / 4000.0;
  if (millis() - mpu_debug_info_hz > DELAY)
  {
    Serial.print("Kp:"); Serial.print(foo,2);
  }
  return foo;
}
double read_ki()
{
  return 0.02;
  double foo = map(analogRead(Ki_PIN), 0.0, 668.0, 0.0, 10000.0);
  
  foo = foo / 2000.0;
  if (millis() - mpu_debug_info_hz > DELAY)
  {
    Serial.print(" Ki:"); Serial.print(foo,2);
  }
  return foo;
}
double read_kd()
{
  return 0.75;
  double foo = map(analogRead(Kd_PIN), 0.0, 668.0, 0.0, 10000.0);
  
  foo = foo / 8000.0;
  if (millis() - mpu_debug_info_hz > DELAY)
  {
    Serial.print(" Kd:"); Serial.print(foo,2); Serial.println("");
  }
  return foo;
}
///////////////////////////////////////////////////////////////////////

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
    if ( millis() - last_mpu_read > DELAY )
    {
      // no sucessful mpu reads for awhile
      // something is wrong
    }
  }
}



