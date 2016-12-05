
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
#define PWM_00_PERCENT 0.05 * PWM_FERQUENCY
#define PWM_10_PERCENT 0.20 * PWM_FERQUENCY
#define PWM_20_PERCENT 0.65 * PWM_FERQUENCY

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
unsigned long micro_tickets = 0;
void update_motors()
{
#define PWM_FERQUENCY 2500 //2800
#define PWM_00_PERCENT .10 * PWM_FERQUENCY
//#define PWM_10_PERCENT .65 * PWM_FERQUENCY
#define PWM_20_PERCENT .20 * PWM_FERQUENCY

  va = map(va, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_00_PERCENT, PWM_20_PERCENT);
  vb = map(vb, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_00_PERCENT, PWM_20_PERCENT);
  vc = map(vc, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_00_PERCENT, PWM_20_PERCENT);
  vd = map(vd, MIN_INPUT_THRUST, MAX_INPUT_THRUST, PWM_00_PERCENT, PWM_20_PERCENT);

  if( (micro_tickets = micros()) - last_pwm_pulse <= PWM_FERQUENCY ) 
  {
    //return;  // wait until next rising pulse
  }
  last_pwm_pulse= micro_tickets; 
  
  PORTD |= B00001000;                                        //Set digital port 3 high
  PORTB |= B00001110;                                        //Set digital port 9,10,11 high

  int motors =    0x00001111;
  while( motors | 0x00000000 )
  {
      if((motors & 0x00001000) && (micros() - last_pwm_pulse) >= va) { PORTD &= B11110111; motors &= 0x00000111; }
      if((motors & 0x00000100) && (micros() - last_pwm_pulse) >= vb) { PORTB &= B11110111; motors &= 0x00001011; }
      if((motors & 0x00000010) && (micros() - last_pwm_pulse) >= vc) { PORTB &= B11111011; motors &= 0x00001101; }
      if((motors & 0x00000001) && (micros() - last_pwm_pulse) >= vd) { PORTB &= B11111101; motors &= 0x00001110; }
  }
}

