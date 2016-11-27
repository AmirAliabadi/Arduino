
#define MOTOR_PIN_A       3       
#define MOTOR_PIN_B       9       
#define MOTOR_PIN_C       11       
#define MOTOR_PIN_D       10 

//#define USE_SERVO
//#define USE_ESC
//#define USE_ANALOGWRITE
//#define PHASE_CORRECT_PWM
//#define USE_BITBANG
#define USE_BITBANG_2

//unsigned long tick;
//unsigned long last_tick = 0;
unsigned long last_pwm_pulse = 0;

int pwm_output = 0;
int pwm_00_percent = 0;
int pwm_10_percent = 0;
int pwm_20_percent = 0;
int pwm_increment = 0;
int top_hold = 100;


long throttle = 0;
void setup() {
  Serial.begin(115200); 
  while (!Serial);
  
  pwm_00_percent = get_pwm_00_percent();  
  pwm_10_percent = get_pwm_10_percent();
  pwm_20_percent = get_pwm_20_percent();
  pwm_increment = get_pwm_increment();
  
  pwm_setup();

}

long read_throttle()
{
  long foo = analogRead(0);
  foo = map(foo, 0, 1000, pwm_10_percent, pwm_20_percent);
  Serial.println(foo);
  return foo; 
}

void loop() 
{  
  pwm_output = read_throttle();  
  read_mpu();
  read_battery();
  update_pids();
  process();

  do_it();
}

void read_mpu()
{
  delay(2); 
}
void read_battery()
{
  //delay(1);
}
void update_pids()
{
  //delay(1);
}
void process()
{
  //delay(1);
}

