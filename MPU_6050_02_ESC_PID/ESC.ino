
/*
   *  ESC respond to PWM signals
   *  typically 
   *    5% duty cycle = 0 throttle
   *    10% duty cycle = 100 throttle
   *    
   *    a 50hz signal
   *    each rising edge will be 1/50 = 20ms 
   *    0 throttle = 5% of 20ms = 1ms
   *    100 throt = 10% of 200m = 2ms
   *    
   *    PWM signals for 50hz
   *      .........    
   *      |       |
   *    --'       '----- 
   *      20ms (50hz)
   *         
   *       _
   *      | |  5% duty cycle = Min ESC Signal
   *   ---` '-----------
   *       1ms
   *       
   *       __
   *      |  |  10% duty cycle = Max ESC Signal (Full throttle)
   *   ---`  '-----------
   *       2ms
   *
   * a 200hz signal
   *  each period will by 5ms
   *    0 throttle = 5% of 5ms = 250 microseconds
   *    100 throt = 10% of 5ms = 500 microseconds
   *
   * a 400hz signal
   *  each rising edge will at 2.5 ms intervals
   *    0 throttle = 5% of 2.5ms = 125 microseconds
   *    100 throt = 10% of 2.5ms = 250 microseconds
   *    
   *    My Loop cycle time is 1.22ms
   *    this is the time is take to do all reads and calculations
   *    This is ~ 890hz 
   *    
   */
 


void init_esc()
{
/*
  esc_a.attach(MOTOR_PIN_A);
  esc_c.attach(MOTOR_PIN_C);
  esc_b.attach(MOTOR_PIN_B);
  esc_d.attach(MOTOR_PIN_D);
*/  

  DDRD |= B00001000;                                           //Configure digital poort 3 as output
  DDRB |= B00001110;                                           //Configure digital poort 9, 10, 11 as output.

  system_check |= INIT_ESC_ATTACHED;
  
  arm_esc();
}

void arm_esc()
{
/*  
  esc_a.arm();
  esc_c.arm();
  esc_b.arm();
  esc_d.arm();
*/
  
  system_check |= INIT_ESC_ARMED;
}

void disarm_esc()
{
  /*
#ifdef ARMED_PULSE_WIDTH
    esc_a.disarm();
    esc_c.disarm();
    esc_b.disarm();
    esc_d.disarm();
#endif    
*/
    system_check &= ~(INIT_ESC_ARMED);
}
/*
void update_motors()
{
#define PWM_FERQUENCY 255 //2800
#define PWM_00_PERCENT 120 //0.05 * PWM_FERQUENCY
#define PWM_10_PERCENT 120 //* PWM_FERQUENCY
#define PWM_20_PERCENT 240 //0.65 * PWM_FERQUENCY

  va = map(va, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_10_PERCENT, PWM_20_PERCENT);
  vb = map(vb, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_10_PERCENT, PWM_20_PERCENT);
  vc = map(vc, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_10_PERCENT, PWM_20_PERCENT);
  vd = map(vd, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_10_PERCENT, PWM_20_PERCENT);

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
#define PWM_FERQUENCY 5550 //2800
#define PWM_00_PERCENT 1000 //.10 * PWM_FERQUENCY
#define PWM_20_PERCENT 2000 //.80 * PWM_FERQUENCY

  va = map(va, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_00_PERCENT, PWM_20_PERCENT);
  vb = map(vb, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_00_PERCENT, PWM_20_PERCENT);
  vc = map(vc, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_00_PERCENT, PWM_20_PERCENT);
  vd = map(vd, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_00_PERCENT, PWM_20_PERCENT);

  while( (micros() - last_pwm_pulse) < PWM_FERQUENCY );
  last_pwm_pulse= micros(); 
  
  PORTD |= B00001000;                                        //Set digital port 3 high
  PORTB |= B00001110;                                        //Set digital port 9,10,11 high

  timer_channel_1 = last_pwm_pulse + va;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = last_pwm_pulse + vb;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = last_pwm_pulse + vc;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = last_pwm_pulse + vd;                                     //Calculate the time of the faling edge of the esc-4 pulse.
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

