
#define MOTOR_PIN_A       3       
#define MOTOR_PIN_B       9       
#define MOTOR_PIN_C       11       
#define MOTOR_PIN_D       10 

//#define USE_SERVO
//#define USE_ESC
//#define USE_ANALOGWRITE
#define PHASE_CORRECT_PWM
//#define USE_BITBANG
//#define USE_BITBANG_2

unsigned long tick;
unsigned long last_tick = 0;
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

  throttle = read_throttle();
return;
  pwm_output = pwm_20_percent;
  unsigned long last_tck = millis();
  while( (millis() - last_tck) <= 10000 ){ do_it(); }

  pwm_output = pwm_00_percent;
  last_tck = millis();
  while( (millis() - last_tck) <= 5000){ do_it(); }

  pwm_output = pwm_10_percent;

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
read_throttle();
return;
  
  tick = millis();

  if( (tick - last_tick) > 100 )
  {
    last_tick = tick;
    
    pwm_output = pwm_output + pwm_increment;
  }

  do_it();

  if( pwm_output >= pwm_20_percent ) 
  { 
    unsigned long last_tck = millis();      
    while( (millis() - last_tck) <= 5000){ do_it(); }
    pwm_increment = pwm_increment * -1;
  }
  if( pwm_output <= pwm_10_percent ) 
  { 
    unsigned long last_tck = millis();      
    while( (millis() - last_tck) <= 5000){ do_it(); }      
    pwm_increment = pwm_increment * -1; 
  }

}

