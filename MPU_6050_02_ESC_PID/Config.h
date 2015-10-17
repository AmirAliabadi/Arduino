

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define LOG_FREQUENCY  50 // ms DEBUG Logging interval
#define BLINK_FREQUENCY 1000

#define INIT_CLEARED          0b00000000
#define INIT_ESC_ATTACHED     0b00000001
#define INIT_ESC_ARMED        0b00000010
#define INIT_THROTTLE_ACTIVE  0b00000100
#define INIT_MOTORS_ENABLED   0b00001000
#define INIT_MPU_ARMED        0b00010000
#define INIT_MPU_STABLE       0b00100000
#define INIT_PID_ON           0b01000000


#define SETPOINT_UNCHANGED    0b000
#define SETPOINT_CHANGED_AC   0b001
#define SETPOINT_CHANGED_BD   0b010
#define SETPOINT_CHANGED_YW   0b100


/////////////////////////////////////////
// MPU
// 798  -738  1310  34  31  31
// 952  -584  1464  34  31  31
#define MPU6050_ACCEL_OFFSET_X 952
#define MPU6050_ACCEL_OFFSET_Y -584
#define MPU6050_ACCEL_OFFSET_Z 1464
#define MPU6050_GYRO_OFFSET_X  34
#define MPU6050_GYRO_OFFSET_Y  31
#define MPU6050_GYRO_OFFSET_Z  31

/*
                A
                |Pitch +
        Roll -  |
        D -----[Y]----- B Roll +
                |
                |Pitch -
                C
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
#define MOTOR_PIN_B       10       // ESC signal wire conected to pin ??
#define MOTOR_PIN_C       6       // ESC signal wire conected to pin 6
#define MOTOR_PIN_D       3       // ESC signal wire conected to pin ??

// THROTTLE SETTINGS
#define MAX_INPUT_THRUST  800     // 
#define MIN_INPUT_THRUST  0       // 
