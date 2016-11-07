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
  esc_a.attach(MOTOR_PIN_A);
  esc_c.attach(MOTOR_PIN_C);
  esc_b.attach(MOTOR_PIN_B);
  esc_d.attach(MOTOR_PIN_D);
  
  system_check |= INIT_ESC_ATTACHED;
  
  arm_esc();
}

void arm_esc()
{
  esc_a.arm();
  esc_c.arm();
  esc_b.arm();
  esc_d.arm();
  
  system_check |= INIT_ESC_ARMED;
}

void disarm_esc()
{
#ifdef ARMED_PULSE_WIDTH
    esc_a.disarm();
    esc_c.disarm();
    esc_b.disarm();
    esc_d.disarm();
#endif    

    system_check &= ~(INIT_ESC_ARMED);
}

