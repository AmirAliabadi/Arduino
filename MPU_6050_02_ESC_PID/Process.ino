
void wait_for_stable_process()
{
  if (millis() - stable_check_loop > 2000)
  {
    stable_check_loop = millis();
    process = &check_if_stable_process;  
  }
}

void check_if_stable_process()
{
  static float last_yw = -333.0;

  Serial.print(F("#SR "));
  Serial.println(abs(ypr[YAW] - last_yw),2);

  float yw_reading = (float)((int)(ypr[YAW]*10.0 + .5))/10.0;
  
  if(last_yw == yw_reading) 
  {
    yw_offset = yw_reading;
    ac_offset = ypr[AC];
    bd_offset = ypr[BD];

    Serial.print(F("#stb: "));
    Serial.print(yw_offset);
    Serial.print(F(" "));
    Serial.print(ac_offset);
    Serial.print(F(" "));
    Serial.println(bd_offset);

    ac_offset = 0;
    bd_offset = 0;

    system_check |= INIT_MPU_STABLE;

    blink_pattern_3
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
      blink_pattern_1      
      process = &attitude_process; 
    }
}

//////////////////////////////////////////////////////////////////////
// main autopilot routine
void attitude_process()
{
//  if( system_check & INIT_ESC_ARMED )
//  {
//    if(abs(ypr[AC]) > 45.0 || abs(ypr[BD]) > 45.0) 
//    {
//      blink_pattern_5      
//      disarm_esc();
//      process = &do_log;
//      return;
//    }
//  }

  // Update the Stable PID input values
  // Angle reading
  current_attitude[YAW] = ypr[YAW];
  current_attitude[BD]  = ypr[BD];  
  current_attitude[AC]  = ypr[AC];  

#ifdef CASCADE_PIDS    
  // acceleration rate reading
  current_rate[YAW] = gyro.z*-1.0;
  current_rate[BD]  = gyro.y*-1.0;    
  current_rate[AC]  = gyro.x;    
#endif  

  if(INPUT_THRUST > MIN_INPUT_THRUST) {

    if( !(system_check & INIT_PID_ON) ) {
            init_pid();  
        }
        
//    if( INPUT_THRUST > 350 ) {
    pid_attitude[YAW].Compute();  
    pid_attitude[BD].Compute();     
    pid_attitude[AC].Compute();       

#ifdef CASCADE_PIDS
    pid_rate[YAW].Compute();
    pid_rate[BD].Compute();         
    pid_rate[AC].Compute();         
#endif
//}

    //////////////////////////////
    // Motor Mix Algorithm      //
    //////////////////////////////
    // compute the boom thrust  //
#ifdef CASCADE_PIDS    
    v_ac = MIN_ESC_CUTOFF + (INPUT_THRUST  - rate_correction[YAW]);
    v_bd = MIN_ESC_CUTOFF + (INPUT_THRUST  + rate_correction[YAW]);
#else
    v_ac = MIN_ESC_CUTOFF + (INPUT_THRUST  - attitude_correction[YAW]); 
    v_bd = MIN_ESC_CUTOFF + (INPUT_THRUST  + attitude_correction[YAW]); 
#endif

    // compute motor speeds
#ifdef CASCADE_PIDS
    va = (v_ac - rate_correction[AC]); 
    vc = (v_ac + rate_correction[AC]); 
    vb = (v_bd - rate_correction[BD]); 
    vd = (v_bd + rate_correction[BD]); 
#else
    va = (v_ac - attitude_correction[AC]); 
    vc = (v_ac + attitude_correction[AC]); 
    vb = (v_bd - attitude_correction[BD]); 
    vd = (v_bd + attitude_correction[BD]); 
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

    yw_offset = (float)((int)(ypr[YAW]*10.0 + .5))/10.0;
    
    pid_reset(); //(MANUAL);
  }

digitalWrite(6,HIGH);
  //update_motors();
  update_motors_analogWrite(); 
digitalWrite(6,LOW);
  
/*

  esc_a.writeMicroseconds(va);
  esc_c.writeMicroseconds(vc);
  esc_b.writeMicroseconds(vb);
  esc_d.writeMicroseconds(vd);  
*/
}

//////////////////////////////////////////////////////////////////////

void do_log() {
  if (millis() - last_log > LOG_FREQUENCY)
  {
    last_log = millis();
    log_data();
  }
}


