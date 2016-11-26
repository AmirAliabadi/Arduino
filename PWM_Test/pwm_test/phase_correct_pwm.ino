#ifdef PHASE_CORRECT_PWM

#define PWM_INCREMENT 1
#define PWM_FERQUENCY 255
#define PWM_00_PERCENT .30 * PWM_FERQUENCY
#define PWM_10_PERCENT .32 * PWM_FERQUENCY
#define PWM_20_PERCENT .65 * PWM_FERQUENCY

int get_pwm_increment()
{
  return PWM_INCREMENT;
}

int get_pwm_00_percent()
{
  return PWM_00_PERCENT; 
}


int get_pwm_10_percent()
{
  return PWM_10_PERCENT; 
}

int get_pwm_20_percent()
{
  return PWM_20_PERCENT; 
}


void pwm_setup()
{
  Serial.println("pwm Mode");

  pinMode(MOTOR_PIN_A, OUTPUT);
  pinMode(MOTOR_PIN_B, OUTPUT);
  pinMode(MOTOR_PIN_C, OUTPUT);
  pinMode(MOTOR_PIN_D, OUTPUT);

  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS22);  
}

void do_it()
{
  OCR2A = pwm_output;
  OCR2B = pwm_output;
}

#endif

