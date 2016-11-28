#ifdef USE_ANALOGWRITE

#define PWM_INCREMENT 1
#define PWM_FERQUENCY 255
#define PWM_00_PERCENT .35 * PWM_FERQUENCY
#define PWM_10_PERCENT .35 * PWM_FERQUENCY 
#define PWM_20_PERCENT .90 * PWM_FERQUENCY


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
  Serial.println("analogWrite Mode");
  analogWrite(MOTOR_PIN_A, 1);
  analogWrite(MOTOR_PIN_B, 1);
  analogWrite(MOTOR_PIN_C, 1);
  analogWrite(MOTOR_PIN_D, 1);
}

void do_it()
{
  analogWrite(MOTOR_PIN_A, pwm_output);
  analogWrite(MOTOR_PIN_B, pwm_output);
  analogWrite(MOTOR_PIN_C, pwm_output);
  analogWrite(MOTOR_PIN_D, pwm_output);     
}

#endif

