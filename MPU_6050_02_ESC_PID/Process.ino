
void wait_for_stable_process()
{
  if (millis() % 2000 == 0 ) //0 - last_log > 2000)
  {
    process = &check_if_stable_process;  
  }
}

void check_if_stable_process()
{
  static float last_yw = -333.0;

  Serial.print(F("#SR "));
  Serial.println(abs(ypr[YW] - last_yw),2);

  float yw_reading = (float)((int)(ypr[YW]*10.0 + .5))/10.0;
  
  if(last_yw == yw_reading) {

    Serial.print(F("#stb: "));
    Serial.print(yw_offset);
    Serial.print(F(" "));
    Serial.print(ac_offset);
    Serial.print(F(" "));
    Serial.println(bd_offset);

    yw_offset = yw_reading;
    ac_offset = 0.0; //ypr[AC];
    bd_offset = 0.0; //ypr[BD];

    ypr_last[YW] = 0.0;
    ypr_last[AC] = 0.0;
    ypr_last[BD] = 0.0;    

    system_check |= INIT_MPU_STABLE;

    do_blink = &do_blink_arm_esc_process;
    process = &arm_esc_process;
    
  } else {
    last_yw = yw_reading;
    process = &wait_for_stable_process;
    
  }
}

void arm_esc_process()
{
    init_esc();
    if( system_check & (INIT_ESC_ATTACHED | INIT_ESC_ARMED) ) {
      do_blink = &do_blink_attitude_process;
      process = &attitude_process; //wait_for_stable_process; //attitude_process;
    }
}

//////////////////////////////////////////////////////////////////////
// main autopilot routine
void attitude_process()
{
  if( system_check & INIT_ESC_ARMED )
  {
    if(abs(ypr[AC]) > 45.0 || abs(ypr[BD]) > 45.0) 
    {
      disarm_esc();
      do_blink = &do_blink_disarm_error;
      process = &do_log;
      return;
    }
  }

  // Update the Stable PID input values
  // Angle reading
  input_ypr[YW] = ypr[YW];
  input_ypr[BD] = ypr[BD];
  input_ypr[AC] = ypr[AC];

#ifdef CASCADE_PIDS    
  // acceleration rate reading
  input_gyro[YW] = gyro.z*-1.0;
  input_gyro[BD] = gyro.y*-1.0;  
  input_gyro[AC] = gyro.x;  
#endif  

  if(INPUT_THRUST > MIN_INPUT_THRUST) {

    if( !(system_check & INIT_PID_ON) ) init_pid();        

    pid_stable[YW].Compute();  
    pid_stable[BD].Compute();  
    pid_stable[AC].Compute();      

#ifdef CASCADE_PIDS
    pid_rate[YW].Compute();
    pid_rate[BD].Compute();       
    pid_rate[AC].Compute();     
#endif

    //////////////////////////////
    // Motor Mix Algorithm      //
    //////////////////////////////
    // compute the boom thrust  //
#ifdef CASCADE_PIDS    
    v_ac = INPUT_THRUST - output_rate[YW];
    v_bd = INPUT_THRUST + output_rate[YW];
#else
    v_ac = INPUT_THRUST - output_ypr[YW]; 
    v_bd = INPUT_THRUST + output_ypr[YW]; 
#endif

    // compute motor speeds
#ifdef CASCADE_PIDS
    va = MIN_ESC_CUTOFF + (v_ac - output_rate[AC]); 
    vc = MIN_ESC_CUTOFF + (v_ac + output_rate[AC]); 
    vb = MIN_ESC_CUTOFF + (v_bd - output_rate[BD]); 
    vd = MIN_ESC_CUTOFF + (v_bd + output_rate[BD]); 
#else
    va = MIN_ESC_CUTOFF + (v_ac - output_ypr[AC]); 
    vc = MIN_ESC_CUTOFF + (v_ac + output_ypr[AC]); 
    vb = MIN_ESC_CUTOFF + (v_bd - output_ypr[BD]); 
    vd = MIN_ESC_CUTOFF + (v_bd + output_ypr[BD]); 
#endif
    //
    ////////////////////////////////

    va = constrain(va, MIN_ESC_CUTOFF, MAX_ESC_SIGNAL);
    vc = constrain(vc, MIN_ESC_CUTOFF, MAX_ESC_SIGNAL);
    vb = constrain(vb, MIN_ESC_CUTOFF, MAX_ESC_SIGNAL);
    vd = constrain(vd, MIN_ESC_CUTOFF, MAX_ESC_SIGNAL);    
  }
  else 
  {
    va = vb = vc = vd = MIN_ESC_SIGNAL;
    
    pid_reset(); //(MANUAL);
  }

  /*
   *  ESC respond to PWM signals
   *  typically 
   *    5% duty cycle = 0 throttle
   *    10% duty cycle = 100 throttle
   *    
   *    a 50hz signal
   *    each period will be 1/50 = 20ms long
   *    0 throttle = 5% of 20ms = 1ms
   *    100 throt = 10% of 200m = 2ms
   *    
   *    PWM signals for 50hz
   *      _________
   *      |       |
   *    --'       '----- 
   *         20ms (50hz)
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
   *  each period will be 2.5 ms
   *    0 throttle = 5% of 2.5ms = 125 microseconds
   *    100 throt = 10% of 2.5ms = 250 microseconds
   *    
   *    My Loop cycle time is 1.22ms
   *    this is the time is take to do all reads and calculations
   *    This is ~ 890hz 
   *    
   */

  esc_a.writeMicroseconds(va);
  esc_c.writeMicroseconds(vc);
  esc_b.writeMicroseconds(vb);
  esc_d.writeMicroseconds(vd);
 
}

//////////////////////////////////////////////////////////////////////

void do_log() {
  if (millis() - last_log > LOG_FREQUENCY)
  {
    last_log = millis();
    log_data();
  }
}


