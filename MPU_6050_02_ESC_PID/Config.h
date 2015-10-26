

#define LED_PIN 13            // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define LOG_FREQUENCY  50     // ms DEBUG Logging interval
#define BLINK_FREQUENCY 1000

#define INIT_CLEARED          0b00000000
#define INIT_ESC_ATTACHED     0b00000001
#define INIT_ESC_ARMED        0b00000010
#define INIT_THROTTLE_ACTIVE  0b00000100
#define INIT_MOTORS_ENABLED   0b00001000
#define INIT_MPU_ARMED        0b00010000
#define INIT_MPU_STABLE       0b00100000
#define INIT_PID_ON           0b01000000
#define INIT_THROTTLE_ZEROED  0b10000000


#define SETPOINT_UNCHANGED    0b000
#define SETPOINT_CHANGED_AC   0b001
#define SETPOINT_CHANGED_BD   0b010
#define SETPOINT_CHANGED_YW   0b100

#define MPU_3

#ifdef MPU_1
#define MPU6050_ACCEL_OFFSET_X 699
#define MPU6050_ACCEL_OFFSET_Y -669
#define MPU6050_ACCEL_OFFSET_Z 1301
#define MPU6050_GYRO_OFFSET_X  47
#define MPU6050_GYRO_OFFSET_Y  -16
#define MPU6050_GYRO_OFFSET_Z  -30
#endif

#ifdef MPU_2
#define MPU6050_ACCEL_OFFSET_X -6542
#define MPU6050_ACCEL_OFFSET_Y -1314
#define MPU6050_ACCEL_OFFSET_Z 840
#define MPU6050_GYRO_OFFSET_X  39
#define MPU6050_GYRO_OFFSET_Y  -49
#define MPU6050_GYRO_OFFSET_Z  15
#endif

#ifdef MPU_3
#define MPU6050_ACCEL_OFFSET_X -62
#define MPU6050_ACCEL_OFFSET_Y 366
#define MPU6050_ACCEL_OFFSET_Z 1416
#define MPU6050_GYRO_OFFSET_X  161
#define MPU6050_GYRO_OFFSET_Y  145
#define MPU6050_GYRO_OFFSET_Z  -19
#endif


// mpu1
// 699  -669  1301  47  -16 -30
// 757  -662  1301  47  -12 -30

// mpu2
// -6640  -1344  832   38    -52   16
// -6542  -1314  840   39    -49   15
// -6545  -1318  834   45    -48   14

// mpu3
// 46 365 1393  163 143 -19
// 47 364 1394  162 143 -20
// 40 376 1398  162 142 -19
// 41 377 1398  162 142 -19
/*
Sensor readings with offsets:  -3  -7  16380 0 0 1
Your offsets: -64 373 1417  161 145 -19

Sensor readings with offsets:  2 -1  16388 0 1 1
Your offsets: -62 366 1416  161 145 -19
*/


/*
 *             (y)
                A Pitch +
                |
        Roll -  |
     (x)D -----[Y]----- B Roll +
                |(z)
                |
                C Pitch -
*/
#define YW 0
#define YAW 0
#define AC 2
#define PITCH 2
#define BD 1
#define ROLL 1
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
#define MAX_ESC_SIGNAL    1900    // This is the max output that will be sent to ESC.  Limitiing for safetly reasons during debug.  Should be 2000 ideally.
#define MIN_ESC_SIGNAL    1100    // Minimum ESC signal to ARM less than or equal to this should turn off motor completely
#define MOTOR_PIN_A       9       // ESC signal wire conected to pin 9
#define MOTOR_PIN_B       5       // ESC signal wire conected to pin ??
#define MOTOR_PIN_C       6       // ESC signal wire conected to pin 6
#define MOTOR_PIN_D       10       // ESC signal wire conected to pin ??

// THROTTLE SETTINGS
#define MAX_INPUT_THRUST  800     // 
#define MIN_INPUT_THRUST  0       // 
