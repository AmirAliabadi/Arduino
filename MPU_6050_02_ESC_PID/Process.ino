void process_off()
{
  #ifdef DEBUG      
  if (millis() - last_log > LOG_FREQUENCY)
  {
    last_log = millis();
    Serial.print(F("#processing is off: "));    
//    log_data();
  }
  #endif    
}

void wait_for_stable()
{
  if (millis() - last_log > 2000)
  {
    last_log = millis();
    process = &check_if_stable;  
  }
}

void check_if_stable()
{
  static float last_yw = -333.0;

#ifdef DEBUG    
  Serial.print(F("#waiting 4 SR "));
  Serial.println(abs(ypr[YW] - last_yw),2);
#endif

  float yw_reading = (int)(ypr[YW]*10.0 + .5)/10.0;

  if(last_yw == yw_reading) {
    system_check |= INIT_MPU_STABLE;
    
    yw_offset = yw_reading;
    ac_offset = ypr[AC];
    bd_offset = ypr[BD];
    
    ypr[YW] = 0;
    ypr_last[YW] = 0;
    
    process = &attitude_process;
  } else {
    last_yw = yw_reading;
    process = &wait_for_stable;
    
  }
}

void arm_esc_process()
{
    read_throttle();
    init_esc();
      
    if( system_check & (INIT_ESC_ATTACHED | INIT_ESC_ARMED) ) 
    {
      process = &wait_for_stable;    
    }
}

//////////////////////////////////////////////////////////////////////
// main autopilot routine
void attitude_process()
{
  uint16_t v_ac = 0;
  uint16_t v_bd = 0;

  if( system_check & INIT_ESC_ARMED )
  {
    if(abs(ypr[AC]) > 45.0 || abs(ypr[BD]) > 45.0) 
    {
      disarm_esc();
      
#ifdef DEBUG       
      Serial.print(F("#esc off: "));
//      log_data();     
#endif      

      process = &process_off;
      return;
    }
  }
  else 
  {

#ifdef DEBUG         
      if (millis() - last_log > LOG_FREQUENCY)
      {   
        last_log = millis();       
             
        Serial.print(F("#esc not ready : "));
//        log_data(); 
      }     
#endif      

      return;
  }

  if(INPUT_THRUST > MIN_INPUT_THRUST + 5.0) {

    if( !(system_check & INIT_PID_ON) ) init_pid();        

    update_pid_settings();

    // Update the Stable PID input values
    input_ypr[YW] = ypr[YW];
    input_ypr[AC] = ypr[AC];
    input_ypr[BD] = ypr[BD];

    input_gyro[YW] = gyro.z*-1.0;//[YW];
    input_gyro[AC] = gyro.x;//[AC];
    input_gyro[BD] = gyro.y*-1.0;//[BD];

    yw_pid.Compute();
    ac_pid.Compute(); bd_pid.Compute(); 
    ac_rat.Compute(); bd_rat.Compute();   

    //////////////////////////////////////////////////////
    // compute the boom velocity
    /*
    float v_ac = (abs(output_ypr[YW] - 100) / 100) * thrust;
    float v_bd = (   (output_ypr[YW] + 100) / 100) * thrust;
    
    // distribute the boom velocity to each boom motor
    float va = ((output_ypr[AC] + 100) / 100) * v_ac;
    float vb = ((output_ypr[BD] + 100) / 100) * v_bd;
    
    float vc = (abs((output_ypr[AC] - 100) / 100)) * v_ac;
    float vd = (abs((output_ypr[BD] - 100) / 100)) * v_bd;
    */
    //
    //////////////////////////////////////////////////////

/*
 * 
 * gyro = angle/s
 * o[P] = angle error (setpoint - ypr{PITCH])
 * r[P] = what to apply to motors
gyro    o[P]  r[P]    va      vc
0.0     0.0   0.0     15.00   15.00 
0.0     0.0   0.0     15.00   15.00 
0.0     0.0   -1.0    14.99   15.01 
2.0     0.0   1.0     15.01   14.99 
42.0    4.5   23.5    15.23   14.76 
138.0   27.0  89.0    15.89   14.11 
235.0   45.0  190.0   16.90   13.10 
220.0   45.0  175.0   16.75   13.25 

4.5 / 1 / REVERSE / REVERSE
 * 
 */
    

    //////////////////////////////
    // Motor Mix Alorithm       //
    //////////////////////////////
    // compute the boom thrust  //
    v_ac = INPUT_THRUST - output_ypr[YW];
    v_bd = INPUT_THRUST + output_ypr[YW];

    // compute motor speeds
    va = MIN_ESC_SIGNAL + (v_ac - output_rate[AC]); // output_ypr
    vc = MIN_ESC_SIGNAL + (v_ac + output_rate[AC]); // output_ypr
    vb = MIN_ESC_SIGNAL + (v_bd - output_rate[BD]); // output_ypr
    vd = MIN_ESC_SIGNAL + (v_bd + output_rate[BD]); // output_ypr
    //
    ////////////////////////////////
  }
  else 
  {
    va = vb = vc = vd = MIN_ESC_SIGNAL;
    
    pid_off();
  }

  va = constrain(va, MIN_ESC_SIGNAL, MAX_ESC_SIGNAL);
  vc = constrain(vc, MIN_ESC_SIGNAL, MAX_ESC_SIGNAL);
  vb = constrain(vb, MIN_ESC_SIGNAL, MAX_ESC_SIGNAL);
  vd = constrain(vd, MIN_ESC_SIGNAL, MAX_ESC_SIGNAL);

  esc_a.writeMicroseconds(va);
  esc_c.writeMicroseconds(vc);
  esc_b.writeMicroseconds(vb);
  esc_d.writeMicroseconds(vd);


#ifdef DEBUG   
  if (millis() - last_log > LOG_FREQUENCY)
  {
    last_log = millis();
//    log_data();
    if( serial_data_mode == 0 ) SerialSend_AC();
    else if( serial_data_mode == 1 ) SerialSend_BD();
    else SerialSend_YAW();
  }
#endif    
    //SerialSend();
  
}
//////////////////////////////////////////////////////////////////////
