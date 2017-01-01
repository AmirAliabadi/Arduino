

#define LED_PIN 13            // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define LOG_FREQUENCY  50     // ms DEBUG Logging interval
//#define BLINK_FREQUENCY 1000

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


/*
 *             (y)
                A Pitch +
                |
        Roll -  |
     (x)D -----[Y]----- B Roll +
                |(z)
                !
                C Pitch -

                USB
*/
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
#define Kp_PIN 0
#define Ki_PIN 1
#define Kd_PIN 2

////////////////////////////////////////////////////////////////
// ESC Settings
#define ESC_ARM_DELAY     3000
#define MAX_ESC_SIGNAL    2000    // This is the max output that will be sent to ESC.
#define MIN_ESC_CUTOFF    1200    // Minimum ESC signal to spin props
#define MIN_ESC_SIGNAL    1000    // Minimum ESC signal, attach esc with this singal, should be not prop spinning.
#define MOTOR_PIN_A       6 // 3       // setup with 500hz PWM pins
#define MOTOR_PIN_B       9       
#define MOTOR_PIN_C       11       
#define MOTOR_PIN_D       10      
/*
 * If you use the default values set by the Arduino Diecimila's bootloader, these are your PWM frequencies:
 * Arduino Pins 5 and 6: 1kHz
 * Arduino Pins 9, 10, 11, and 3: 500Hz
 * 
 * pin pairs:
 * 3 - 11 : default PWM frequency: 490.20 Hz
 * 9 - 10 : default PWM frequency: 490.20 Hz
 * 5 - 6  : default PWM frequency: 976.56 Hz
 */


// THROTTLE SETTINGS
#define MAX_INPUT_THRUST  1000    // 
#define MIN_INPUT_THRUST  0       // 



// led blink patterns
long last_blink = 0;
int blink_point = 0;
int blink_pattern[4] = {1000,1000,1000,1000};

#define blink_reset blink_point = 0; last_blink = millis(); digitalWrite(LED_PIN, LOW); 
#define blink_pattern_1 blink_reset blink_pattern[0] = 1000; blink_pattern[1] = 1000;  blink_pattern[2] = 1000;  blink_pattern[3] = 1000;
#define blink_pattern_2 blink_reset blink_pattern[0] = 1000; blink_pattern[1] = 200;   blink_pattern[2] = 1000;  blink_pattern[3] = 200;
#define blink_pattern_3 blink_reset blink_pattern[0] = 1000; blink_pattern[1] = 200;   blink_pattern[2] = 200;   blink_pattern[3] = 1000;
#define blink_pattern_4 blink_reset blink_pattern[0] = 1000; blink_pattern[1] = 200;   blink_pattern[2] = 200;   blink_pattern[3] = 200;
#define blink_pattern_5 blink_reset blink_pattern[0] = 100;  blink_pattern[1] = 100;   blink_pattern[2] = 100;   blink_pattern[3] = 100;

///
// PPM INPUTS
#define THROTTLE_CHANNEL 3
#define PID_SELECT_CHANNEL 5
#define PID_TUNE_CHANNEL 6
