
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

