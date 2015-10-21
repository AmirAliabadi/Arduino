#ifndef QUADARDU
#define QUADARDU

#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>

#define DEBUG
#define MPU6050_ACCEL_OFFSET_X -6640
#define MPU6050_ACCEL_OFFSET_Y -1344
#define MPU6050_ACCEL_OFFSET_Z 832
#define MPU6050_GYRO_OFFSET_X  38
#define MPU6050_GYRO_OFFSET_Y  -52
#define MPU6050_GYRO_OFFSET_Z  -16

// board1 :   722  -654   1305  48    -16   -32
// board2 : -6640  -1344  832   38    -52   16



/*  Arduino Pin configuration
 *
 */

#define ESC_A 9
#define ESC_B 5
#define ESC_C 6
#define ESC_D 10

#define RC_1 13
#define RC_2 12
#define RC_3 11
#define RC_4 10
#define RC_5 8
#define RC_PWR A0


/* ESC configuration
 *
 */

#define ESC_MIN 1100 //22
#define ESC_MAX 1800 //115
#define ESC_TAKEOFF_OFFSET 1140 //30
#define ESC_ARM_DELAY 5000

/* RC configuration
 *
 */

#define RC_HIGH_CH1 1000
#define RC_LOW_CH1 2000
#define RC_HIGH_CH2 1000
#define RC_LOW_CH2 2000
#define RC_HIGH_CH3 1000
#define RC_LOW_CH3 2000
#define RC_HIGH_CH4 1000
#define RC_LOW_CH4 2000
#define RC_HIGH_CH5 1000
#define RC_LOW_CH5 2000
#define RC_ROUNDING_BASE 50

/*  PID configuration
 *
 */

#define PITCH_P_VAL .01
#define PITCH_I_VAL 0
#define PITCH_D_VAL .001

#define ROLL_P_VAL .01
#define ROLL_I_VAL 0 //5
#define ROLL_D_VAL .001

#define YAW_P_VAL 0
#define YAW_I_VAL 0 //5
#define YAW_D_VAL 0


/* Flight parameters
 *
 */

#define PITCH_MIN -30
#define PITCH_MAX 30
#define ROLL_MIN -30
#define ROLL_MAX 30
#define YAW_MIN -180
#define YAW_MAX 180
#define PID_PITCH_INFLUENCE 20
#define PID_ROLL_INFLUENCE 20
#define PID_YAW_INFLUENCE 20


/*  MPU variables
 *
 */

MPU6050 mpu;                           // mpu interface object


uint8_t mpuIntStatus;                  // mpu statusbyte
uint8_t devStatus;                     // device status
uint16_t packetSize;                   // estimated packet size
uint16_t fifoCount;                    // fifo buffer size
uint8_t fifoBuffer[64];                // fifo buffer

Quaternion q;                          // quaternion for mpu output
VectorFloat gravity;                   // gravity vector for ypr
float yrp[3] = {0.0f, 0.0f, 0.0f};     // yaw pitch roll values
float yrpLast[3] = {0.0f, 0.0f, 0.0f};

volatile bool mpuInterrupt = false;    //interrupt flag

/* Interrupt lock
 *
 */

boolean interruptLock = false;

/*  RC variables
 *
 */

float ch1, ch2, ch3, ch4, ch5;         // RC channel inputs

unsigned long rcLastChange1 = micros();
unsigned long rcLastChange2 = micros();
unsigned long rcLastChange3 = micros();
unsigned long rcLastChange4 = micros();
unsigned long rcLastChange5 = micros();

/*  Motor controll variables
 *
 */

int velocity;                          // global velocity

float bal_ac, bal_bd;                 // motor balances can vary between -100 & 100
float bal_axes;                       // throttle balance between axes -100:ac , +100:bd

int va, vb, vc, vd;                    //velocities
int v_ac, v_bd;                        // velocity of axes

Servo a, b, c, d;

/*  PID variables
 *
 */

#define YAW  0
#define PITCH 2
#define ROLL 1

PID pitchReg(&yrp[PITCH], &bal_ac, &ch2, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, REVERSE);
PID rollReg(&yrp[ROLL], &bal_bd, &ch1, ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL, REVERSE);
PID yawReg(&yrp[YAW], &bal_axes, &ch4, YAW_P_VAL, YAW_I_VAL, YAW_D_VAL, DIRECT);


/*  Filter variables
 *
 */

float ch1Last, ch2Last, ch4Last, velocityLast;

/*  Setup function
 *
 */

long last_log = 0;

float yaw_offset, pitch_offset, roll_offset;

void setup() {
    a.detach();
    c.detach();
    b.detach();
    d.detach();

    delay(10);
  
#ifdef DEBUG                        // Device tests go here
  Serial.begin(115200);                 // Serial only necessary if in DEBUG mode
  Serial.flush();
#endif

  //initRC();                            // Self explaining
  initMPU();
  initESCs();
  initBalancing();
  initRegulators();

  /*
    for(int i=0; i < 500; i++ ) {
      getYPR();
      delay(5);
    }

    yaw_offset = yrp[0];
    pitch_offset = yrp[2];
    roll_offset = yrp[1]; */


}

/* loop function
 *
 */

 boolean is_stable = 0;
 float last_yaw = 333;

void loop() {

  while (!mpuInterrupt) { // && fifoCount < packetSize){

    /* Do nothing while MPU is not working
     * This should be a VERY short period
     */

  }

  getYPR();

  if( is_stable ) {
    computePID();
    calculateVelocities();
    updateMotors();
  } else {
    if (millis() - last_log > 2000)
    {
      Serial.print(yrp[0]); Serial.print(F("\t"));
      Serial.println(last_yaw);
      last_log = millis();
      if( yrp[0] == last_yaw )
      {
        is_stable = 1;
        yaw_offset = yrp[0];
      } 
      else 
      {
        last_yaw = yrp[0];
      }
    }
  }
}

/*  computePID function
 *
 *  formats data for use with PIDLib
 *  and computes PID output
 */

void computePID() {

  acquireLock();

  /*
  ch1 = floor(ch1/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  ch2 = floor(ch2/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  ch4 = floor(ch4/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;

  ch2 = map(ch2, RC_LOW_CH2, RC_HIGH_CH2, PITCH_MIN, PITCH_MAX);
  ch1 = map(ch1, RC_LOW_CH1, RC_HIGH_CH1, ROLL_MIN, ROLL_MAX);
  ch4 = map(ch4, RC_LOW_CH4, RC_HIGH_CH4, YAW_MIN, YAW_MAX);

  if((ch2 < PITCH_MIN) || (ch2 > PITCH_MAX)) ch2 = ch2Last;
  if((ch1 < ROLL_MIN) || (ch1 > ROLL_MAX)) ch1 = ch1Last;
  if((ch4 < YAW_MIN) || (ch4 > YAW_MAX)) ch4 = ch4Last;

  ch1Last = ch1;
  ch2Last = ch2;
  ch4Last = ch4;
  */
  ch1Last = 0;
  ch2Last = 0;
  ch4Last = 0;

////
//
////

  pitchReg.Compute();
  rollReg.Compute();
  yawReg.Compute();

  releaseLock();

}

/*  getYPR function
 *
 *  gets data from MPU and
 *  computes pitch, roll, yaw on the MPU's DMP
 */

void getYPR() {

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {

    Serial.println(F("Fifo overflow!"));
    mpu.resetFIFO();

  } else if (mpuIntStatus & 0x02) {

    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);

    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(yrp, &q, &gravity);

    /////
    //
    yrp[0] = yrp[0] * 180 / M_PI;
    yrp[1] = yrp[1] * 180 / M_PI;
    yrp[2] = yrp[2] * 180 / M_PI;
    
    yrp[0] = (float)((int)(( (yrp[0] - yaw_offset) * 10.0) +.5))/10.0;
    yrp[1] = (float)((int)((yrp[1] * 10.0) +.5))/10.0;
    yrp[2] = (float)((int)((yrp[2] * 10.0) +.5))/10.0;    
    
    if (abs(yrp[0] - yrpLast[0]) > 30) yrp[0] = yrpLast[0];
    if (abs(yrp[1] - yrpLast[1]) > 30) yrp[1] = yrpLast[1];
    if (abs(yrp[2] - yrpLast[2]) > 30) yrp[2] = yrpLast[2];
    
    yrpLast[0] = yrp[0];
    yrpLast[1] = yrp[1];
    yrpLast[2] = yrp[2];
    //
    //////    

  }

}

/*  calculateVelocities function
 *
 *  calculates the velocities of every motor
 *  using the PID output
 */

void calculateVelocities() {

  acquireLock();

  //ch3 = floor(ch3/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  //velocity = map(ch3, RC_LOW_CH3, RC_HIGH_CH3, ESC_MIN, ESC_MAX);
  velocity = map(analogRead(0), 0, 688, ESC_MIN, ESC_MAX);

  releaseLock();

  if ((velocity < ESC_MIN) || (velocity > ESC_MAX)) velocity = velocityLast;

  velocityLast = velocity;

  v_ac = (abs(-100 + bal_axes) / 100) * velocity;
  v_bd = ((100 + bal_axes) / 100) * velocity;

  va = ((100 + bal_ac) / 100) * v_ac;
  vb = ((100 + bal_bd) / 100) * v_bd;

  vc = (abs((-100 + bal_ac) / 100)) * v_ac;
  vd = (abs((-100 + bal_bd) / 100)) * v_bd;

  va = constrain(va, ESC_MIN, ESC_MAX);
  vb = constrain(vb, ESC_MIN, ESC_MAX);
  vc = constrain(vc, ESC_MIN, ESC_MAX);
  vd = constrain(vd, ESC_MIN, ESC_MAX);

  if (velocity < ESC_TAKEOFF_OFFSET) {

    va = ESC_MIN;
    vb = ESC_MIN;
    vc = ESC_MIN;
    vd = ESC_MIN;

  }

  if (millis() - last_log > 50)
  {
    last_log = millis();

    Serial.print(velocity);  Serial.print(F("\t"));

    Serial.print(yrp[0]);    Serial.print(F("\t"));
    Serial.print(yrp[1]);    Serial.print(F("\t"));
    Serial.print(yrp[2]);    Serial.print(F("\t"));

    //    Serial.print(bal_axes);  Serial.print(F("\t"));
    Serial.print(bal_ac);    Serial.print(F("\t"));
    Serial.print(bal_bd);    Serial.print(F("\t"));

    Serial.print(va);       Serial.print(F("\t"));
    Serial.print(vc);       Serial.print(F("\t"));
    Serial.print(vb);       Serial.print(F("\t"));
    Serial.print(vd);

    Serial.println(F(""));

  }
}

inline void updateMotors() {

  //a.write(va);
  //c.write(vc);
  //b.write(vb);
  //d.write(vd);

  a.writeMicroseconds(va);
  c.writeMicroseconds(vc);
  b.writeMicroseconds(vb);
  d.writeMicroseconds(vd);

}

inline void arm() {

  //a.write(ESC_MIN);
  //b.write(ESC_MIN);
  //c.write(ESC_MIN);
  //d.write(ESC_MIN);

  a.writeMicroseconds(ESC_MIN);
  b.writeMicroseconds(ESC_MIN);
  c.writeMicroseconds(ESC_MIN);
  d.writeMicroseconds(ESC_MIN);

  delay(ESC_ARM_DELAY);

}

inline void dmpDataReady() {
  mpuInterrupt = true;
}

inline void initRC() {
  pinMode(RC_PWR, OUTPUT);
  digitalWrite(RC_PWR, HIGH);

  // FIVE FUCKING INTERRUPTS !!!
  PCintPort::attachInterrupt(RC_1, rcInterrupt1, CHANGE);
  PCintPort::attachInterrupt(RC_2, rcInterrupt2, CHANGE);
  PCintPort::attachInterrupt(RC_3, rcInterrupt3, CHANGE);
  PCintPort::attachInterrupt(RC_4, rcInterrupt4, CHANGE);
  PCintPort::attachInterrupt(RC_5, rcInterrupt5, CHANGE);

}

void initMPU() {
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {

    // Supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(MPU6050_ACCEL_OFFSET_X);
    mpu.setYAccelOffset(MPU6050_ACCEL_OFFSET_Y);
    mpu.setZAccelOffset(MPU6050_ACCEL_OFFSET_Z);
    mpu.setXGyroOffset(MPU6050_GYRO_OFFSET_X);
    mpu.setYGyroOffset(MPU6050_GYRO_OFFSET_Y);
    mpu.setZGyroOffset(MPU6050_GYRO_OFFSET_Z);

    delay(10);

    ///////////////////////////////////////////////////////////////////
    mpu.setDLPFMode(MPU6050_DLPF_BW_5); //MPU6050_DLPF_BW_98);

    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();

  }
}

inline void initESCs() {

  a.attach(ESC_A);
  b.attach(ESC_B);
  c.attach(ESC_C);
  d.attach(ESC_D);

  delay(100);

  arm();

}

inline void initBalancing() {

  bal_axes = 0;
  bal_ac = 0;
  bal_bd = 0;

}

inline void initRegulators() {

  pitchReg.SetMode(AUTOMATIC);
  pitchReg.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);

  rollReg.SetMode(AUTOMATIC);
  rollReg.SetOutputLimits(-PID_ROLL_INFLUENCE, PID_ROLL_INFLUENCE);

  yawReg.SetMode(AUTOMATIC);
  yawReg.SetOutputLimits(-PID_YAW_INFLUENCE, PID_YAW_INFLUENCE);

}

inline void rcInterrupt1() {
  if (!interruptLock) ch1 = micros() - rcLastChange1;
  rcLastChange1 = micros();
}

inline void rcInterrupt2() {
  if (!interruptLock) ch2 = micros() - rcLastChange2;
  rcLastChange2 = micros();
}

inline void rcInterrupt3() {
  if (!interruptLock) ch3 = micros() - rcLastChange3;
  rcLastChange3 = micros();
}

inline void rcInterrupt4() {
  if (!interruptLock) ch4 = micros() - rcLastChange4;
  rcLastChange4 = micros();
}

inline void rcInterrupt5() {
  if (!interruptLock) ch5 = micros() - rcLastChange5;
  rcLastChange5 = micros();
}

inline void acquireLock() {
  interruptLock = true;
}

inline void releaseLock() {
  interruptLock = false;
}

#endif


