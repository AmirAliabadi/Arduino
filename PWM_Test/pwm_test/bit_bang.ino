#ifdef USE_BITBANG

#define PWM_INCREMENT 10
#define PWM_FERQUENCY 2800
#define PWM_00_PERCENT .30 * PWM_FERQUENCY
#define PWM_10_PERCENT .32 * PWM_FERQUENCY
#define PWM_20_PERCENT .65 * PWM_FERQUENCY

// .35, .45, .65 - works
// .30, .33, .65 - works and 10 percetn keeps motor running

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
}

unsigned long micro_tickets = 0;
void do_it()
{
  while( (micro_tickets = micros()) - last_pwm_pulse <= PWM_FERQUENCY ) ;  // wait until next rising pulse
  last_pwm_pulse= micro_tickets;
  
  digitalWrite(MOTOR_PIN_A, HIGH);
  digitalWrite(MOTOR_PIN_B, HIGH);
  digitalWrite(MOTOR_PIN_C, HIGH);
  digitalWrite(MOTOR_PIN_D, HIGH);   

/*
  while( (micros() - last_pwm_pulse) < pwm_output_a ) ;
  digitalWrite(MOTOR_PIN_A, LOW); 
  digitalWrite(MOTOR_PIN_B, LOW); 
  digitalWrite(MOTOR_PIN_C, LOW); 
  digitalWrite(MOTOR_PIN_D, LOW); 
*/

  int motors = 0x00001111;
  while( motors | 0x00000000 )
  {
    if((micros() - last_pwm_pulse) >= pwm_output_a) { digitalWrite(MOTOR_PIN_A, LOW); motors &= 0x00000111; }
    if((micros() - last_pwm_pulse) >= pwm_output_b) { digitalWrite(MOTOR_PIN_B, LOW); motors &= 0x00001011; }
    if((micros() - last_pwm_pulse) >= pwm_output_c) { digitalWrite(MOTOR_PIN_C, LOW); motors &= 0x00001101; }
    if((micros() - last_pwm_pulse) >= pwm_output_d) { digitalWrite(MOTOR_PIN_D, LOW); motors &= 0x00001110; }
  }
}

#endif

