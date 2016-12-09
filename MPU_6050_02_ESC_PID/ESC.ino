
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
  #define PWM_00_PERCENT 125 // (MIN_ESC_SIGNAL / 10 + 20) // 120
  #define PWM_20_PERCENT 245 // (MAX_ESC_SIGNAL / 10 + 40) // 240

  // map input throttle of 0-1000 to 125 to 245
  va = map(va, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_00_PERCENT, PWM_20_PERCENT);
  vb = map(vb, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_00_PERCENT, PWM_20_PERCENT);
  vc = map(vc, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_00_PERCENT, PWM_20_PERCENT);
  vd = map(vd, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_00_PERCENT, PWM_20_PERCENT);
  
  analogWrite(MOTOR_PIN_A , va ) ;
  analogWrite(MOTOR_PIN_B , vb ) ;
  analogWrite(MOTOR_PIN_C , vc ) ;
  analogWrite(MOTOR_PIN_D , vd ) ; 

  // map 125 to 245 to 1000 to 2000, this makes things look like the 1-2ms duty cycle
  // this is only needed for debuging / comparing analogWrite() to bit banging to Servo.h to ESC.h 
  va = map(va, PWM_00_PERCENT, PWM_20_PERCENT, va == 0 ? MIN_ESC_SIGNAL : MIN_ESC_CUTOFF, MAX_ESC_SIGNAL);
  vb = map(vb, PWM_00_PERCENT, PWM_20_PERCENT, vb == 0 ? MIN_ESC_SIGNAL : MIN_ESC_CUTOFF, MAX_ESC_SIGNAL);
  vc = map(vc, PWM_00_PERCENT, PWM_20_PERCENT, vc == 0 ? MIN_ESC_SIGNAL : MIN_ESC_CUTOFF, MAX_ESC_SIGNAL);
  vd = map(vd, PWM_00_PERCENT, PWM_20_PERCENT, vd == 0 ? MIN_ESC_SIGNAL : MIN_ESC_CUTOFF, MAX_ESC_SIGNAL);

}
*/

unsigned long last_pwm_pulse = 0;
unsigned long esc_pwm_timmer = 0;
unsigned long timer_channel_a = 0;
unsigned long timer_channel_b = 0; 
unsigned long timer_channel_c = 0;
unsigned long timer_channel_d = 0; 

int motors = 0x00000000;
void update_motors()
{
  #define PWM_FERQUENCY 5550 //5550 works with mpu dmpEnabled()

  while( (micros() - last_pwm_pulse) < PWM_FERQUENCY );
  last_pwm_pulse= micros(); 
  
  PORTD |= B00001000; // Set digital port 3 high
  PORTB |= B00001110; // Set digital port 9,10,11 high
  
  va = map(va, MIN_INPUT_THRUST, MAX_INPUT_THRUST, va == 0 ? MIN_ESC_SIGNAL : MIN_ESC_CUTOFF, MAX_ESC_SIGNAL);
  vb = map(vb, MIN_INPUT_THRUST, MAX_INPUT_THRUST, vb == 0 ? MIN_ESC_SIGNAL : MIN_ESC_CUTOFF, MAX_ESC_SIGNAL);
  vc = map(vc, MIN_INPUT_THRUST, MAX_INPUT_THRUST, vc == 0 ? MIN_ESC_SIGNAL : MIN_ESC_CUTOFF, MAX_ESC_SIGNAL);
  vd = map(vd, MIN_INPUT_THRUST, MAX_INPUT_THRUST, vd == 0 ? MIN_ESC_SIGNAL : MIN_ESC_CUTOFF, MAX_ESC_SIGNAL);

  timer_channel_a = last_pwm_pulse + va ;
  timer_channel_b = last_pwm_pulse + vb ;
  timer_channel_c = last_pwm_pulse + vc ;
  timer_channel_d = last_pwm_pulse + vd ;

  motors = 0x00001111;
  while( motors )
  {
      esc_pwm_timmer = micros();
      if((motors & 0x00001000) && (timer_channel_a <= esc_pwm_timmer)){ PORTD &= B11110111; motors &= 0x00000111; }
      if((motors & 0x00000100) && (timer_channel_b <= esc_pwm_timmer)){ PORTB &= B11110111; motors &= 0x00001011; }
      if((motors & 0x00000010) && (timer_channel_c <= esc_pwm_timmer)){ PORTB &= B11111011; motors &= 0x00001101; }
      if((motors & 0x00000001) && (timer_channel_d <= esc_pwm_timmer)){ PORTB &= B11111101; motors &= 0x00001110; }      
  }
}

