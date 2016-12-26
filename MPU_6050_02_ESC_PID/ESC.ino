
void arm_esc()
{
  system_check |= INIT_ESC_ARMED;
}

void disarm_esc()
{
  system_check &= ~(INIT_ESC_ARMED);
}

//----------------
/*
void init_esc()
{
  system_check |= INIT_ESC_ATTACHED;
  arm_esc();
}


void update_motors()
{
  // Trial and Error has yielded 120 - 240 ESC signal when 
  // using analogWrite() and the yellow unknown esc's from amazon
  #define PWM_00_PERCENT 125 
  #define PWM_10_PERCENT 155 
  #define PWM_20_PERCENT 250 

  if( system_check & INIT_ESC_ARMED ) {
    // map input throttle of 0-1000 to 125 to 250
    analogWrite(MOTOR_PIN_A , map(va, MIN_INPUT_THRUST, MAX_INPUT_THRUST, va == 0 ? PWM_00_PERCENT : PWM_10_PERCENT, PWM_20_PERCENT) ) ;
    analogWrite(MOTOR_PIN_B , map(vb, MIN_INPUT_THRUST, MAX_INPUT_THRUST, vb == 0 ? PWM_00_PERCENT : PWM_10_PERCENT, PWM_20_PERCENT) ) ;
    analogWrite(MOTOR_PIN_C , map(vc, MIN_INPUT_THRUST, MAX_INPUT_THRUST, vc == 0 ? PWM_00_PERCENT : PWM_10_PERCENT, PWM_20_PERCENT) ) ;
    analogWrite(MOTOR_PIN_D , map(vd, MIN_INPUT_THRUST, MAX_INPUT_THRUST, vd == 0 ? PWM_00_PERCENT : PWM_10_PERCENT, PWM_20_PERCENT) ) ; 
  } else {
    analogWrite(MOTOR_PIN_A , 0 );
    analogWrite(MOTOR_PIN_B , 0 );
    analogWrite(MOTOR_PIN_C , 0 );
    analogWrite(MOTOR_PIN_D , 0 );
  }
}
*/



unsigned long last_pwm_pulse = 0;
unsigned long esc_pwm_timmer = 0;
unsigned long timer_channel_a = 0;
unsigned long timer_channel_b = 0; 
unsigned long timer_channel_c = 0;
unsigned long timer_channel_d = 0; 
int motors = 0x00000000;

void init_esc()
{
  DDRD |= B00001000;                                           //Configure digital poort 3 as output
  DDRB |= B00001110;                                           //Configure digital poort 9, 10, 11 as output.

  system_check |= INIT_ESC_ATTACHED;
  
  arm_esc();
}

void update_motors()
{
  #define PWM_FERQUENCY 5550 // 5550 //5550 works with mpu dmpEnabled()
  // 5450 : 170-190hz 
  // 5000 jumps arounds when throttle at over 60%

  while( (micros() - last_pwm_pulse) < PWM_FERQUENCY );
  last_pwm_pulse= micros(); 
  
  PORTD |= B00001000; // Set digital port 3 high
  PORTB |= B00001110; // Set digital port 9,10,11 high

  if( system_check & INIT_ESC_ARMED ) {

    timer_channel_a = last_pwm_pulse + map(va, MIN_INPUT_THRUST, MAX_INPUT_THRUST, va == 0 ? MIN_ESC_SIGNAL : MIN_ESC_CUTOFF, MAX_ESC_SIGNAL); ;
    timer_channel_b = last_pwm_pulse + map(vb, MIN_INPUT_THRUST, MAX_INPUT_THRUST, vb == 0 ? MIN_ESC_SIGNAL : MIN_ESC_CUTOFF, MAX_ESC_SIGNAL); ;
    timer_channel_c = last_pwm_pulse + map(vc, MIN_INPUT_THRUST, MAX_INPUT_THRUST, vc == 0 ? MIN_ESC_SIGNAL : MIN_ESC_CUTOFF, MAX_ESC_SIGNAL); ;
    timer_channel_d = last_pwm_pulse + map(vd, MIN_INPUT_THRUST, MAX_INPUT_THRUST, vd == 0 ? MIN_ESC_SIGNAL : MIN_ESC_CUTOFF, MAX_ESC_SIGNAL); ;
  
    motors = 0x00001111;
    while( motors ) 
    {
        esc_pwm_timmer = micros();
        if( (motors & 0x00001000) && ( esc_pwm_timmer > timer_channel_a )){ PORTD &= B11110111; motors &= 0x00000111; }
        if( (motors & 0x00000100) && ( esc_pwm_timmer > timer_channel_b )){ PORTB &= B11110111; motors &= 0x00001011; }
        if( (motors & 0x00000010) && ( esc_pwm_timmer > timer_channel_c )){ PORTB &= B11111011; motors &= 0x00001101; }
        if( (motors & 0x00000001) && ( esc_pwm_timmer > timer_channel_d )){ PORTB &= B11111101; motors &= 0x00001110; }      
    }
    
  } else {
    PORTD &= B11110111;
    PORTB &= B11110111;
    PORTB &= B11111011;
    PORTB &= B11111101;
  }
}

