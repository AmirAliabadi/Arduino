
void init_esc()
{
  DDRD |= B00001000;                                           //Configure digital poort 3 as output
  DDRB |= B00001110;                                           //Configure digital poort 9, 10, 11 as output.

  system_check |= INIT_ESC_ATTACHED;
  
  arm_esc();
}

void arm_esc()
{
  system_check |= INIT_ESC_ARMED;
}

void disarm_esc()
{
    system_check &= ~(INIT_ESC_ARMED);
}

/*
void update_motors()
{
// TRIAL AND Error has yielded 120 - 240 ESC signal when 
// using analogWrite() and the yellow unknown esc's from amazon
#define PWM_00_PERCENT (MIN_ESC_SIGNAL / 10 + 20) // 120
#define PWM_20_PERCENT (MAX_ESC_SIGNAL / 10 + 40) // 240

  va = map(va, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_00_PERCENT, PWM_20_PERCENT);
  vb = map(vb, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_00_PERCENT, PWM_20_PERCENT);
  vc = map(vc, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_00_PERCENT, PWM_20_PERCENT);
  vd = map(vd, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_00_PERCENT, PWM_20_PERCENT);

  analogWrite(MOTOR_PIN_A , va);
  analogWrite(MOTOR_PIN_B , vb);
  analogWrite(MOTOR_PIN_C , vc);
  analogWrite(MOTOR_PIN_D , vd);
}
*/


unsigned long last_pwm_pulse = 0;
unsigned long esc_pwm_timmer = 0;
unsigned long micro_tickets = 0;
unsigned long timer_channel_1 = 0;
unsigned long timer_channel_2 = 0; 
unsigned long timer_channel_3 = 0;
unsigned long timer_channel_4 = 0; 
int motors = 0x0;
void update_motors()
{
  #define PWM_FERQUENCY 5550 //5550 works with mpu dmpEnabled()
  #define PWM_00_PERCENT MIN_ESC_SIGNAL 
  #define PWM_20_PERCENT MAX_ESC_SIGNAL

  va = map(va, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_00_PERCENT, PWM_20_PERCENT);
  vb = map(vb, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_00_PERCENT, PWM_20_PERCENT);
  vc = map(vc, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_00_PERCENT, PWM_20_PERCENT);
  vd = map(vd, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_00_PERCENT, PWM_20_PERCENT);

  while( (micros() - last_pwm_pulse) < PWM_FERQUENCY );
  last_pwm_pulse= micros(); 
  
  PORTD |= B00001000;                                        //Set digital port 3 high
  PORTB |= B00001110;                                        //Set digital port 9,10,11 high

  timer_channel_1 = last_pwm_pulse + va;
  timer_channel_2 = last_pwm_pulse + vb;
  timer_channel_3 = last_pwm_pulse + vc;
  timer_channel_4 = last_pwm_pulse + vd;
  motors = 0x00001111;
  while( motors )
  {
      esc_pwm_timmer = micros();
      if((motors & 0x00001000) && (timer_channel_1 <= esc_pwm_timmer)){ PORTD &= B11110111; motors &= 0x00000111; }
      if((motors & 0x00000100) && (timer_channel_2 <= esc_pwm_timmer)){ PORTB &= B11110111; motors &= 0x00001011; }
      if((motors & 0x00000010) && (timer_channel_3 <= esc_pwm_timmer)){ PORTB &= B11111011; motors &= 0x00001101; }
      if((motors & 0x00000001) && (timer_channel_4 <= esc_pwm_timmer)){ PORTB &= B11111101; motors &= 0x00001110; }      
  }
}

