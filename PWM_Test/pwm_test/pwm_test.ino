
#define MOTOR_PIN_A       3       
#define MOTOR_PIN_B       9       
#define MOTOR_PIN_C       10       
#define MOTOR_PIN_D       11 

//#define USE_SERVO
//#define USE_ESC
//#define USE_ANALOGWRITE
//#define PHASE_CORRECT_PWM
//#define USE_BITBANG
#define USE_BITBANG_2

//unsigned long tick;
//unsigned long last_tick = 0;
unsigned long last_pwm_pulse = 0;

//int pwm_output = 0;
int pwm_output_a = 0;
int pwm_output_b = 0;
int pwm_output_c = 0;
int pwm_output_d = 0;


int pwm_00_percent = 0;
int pwm_10_percent = 0;
int pwm_20_percent = 0;
int pwm_increment = 0;

long throttle = 0;
void setup() {
  Serial.begin(115200); 
  while (!Serial);

  pinMode(6, OUTPUT);
  
  pwm_00_percent = get_pwm_00_percent();  
  pwm_10_percent = get_pwm_10_percent();
  pwm_20_percent = get_pwm_20_percent();
  pwm_increment = get_pwm_increment();
  
  pwm_setup();

}

long read_throttle()
{
  long foo = analogRead(0);
  foo = map(constrain(foo,0,1000), 0, 1000, pwm_10_percent, pwm_20_percent);
  return foo; 
}

void loop() 
{  
  int t = read_throttle();
  pwm_output_a = t;  
  pwm_output_b = t;
  pwm_output_c = t;
  pwm_output_d = t;      

  digitalWrite(6, HIGH);
  read_mpu();
  read_battery();
  update_pids();
  process();
  digitalWrite(6, LOW);

Serial.println(t);
  do_it();
}

void read_mpu()
{
  delayMicroseconds(800); 
}
void read_battery()
{
}
void update_pids()
{
  delayMicroseconds(100); 
}
void process()
{
  delayMicroseconds(100); 
}

