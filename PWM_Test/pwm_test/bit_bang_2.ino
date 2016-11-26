#ifdef USE_BITBANG_2

#define PWM_INCREMENT 10
#define PWM_FERQUENCY 2550
#define PWM_00_PERCENT .30 * PWM_FERQUENCY
#define PWM_10_PERCENT .32 * PWM_FERQUENCY
#define PWM_20_PERCENT .65 * PWM_FERQUENCY

int get_pwm_increment()
{
  return PWM_INCREMENT;
}

int get_pwm_00_percent()
{
  return PWM_00_PERCENT; 
}


int get_pwm_10_percent()
{
  return PWM_10_PERCENT; 
}

int get_pwm_20_percent()
{
  return PWM_20_PERCENT; 
}


void pwm_setup()
{
  Serial.println("pwm Mode");

  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.

  //       76543210  
  DDRD |= B00001000;                                           //Configure digital poort 4, 5, 6 and 7 as output.
  
  //         111100
  //         321098
  DDRB |= B00001110;                                           //Configure digital poort 12 and 13 as output.
  
}

unsigned long micro_tickets = 0;
void do_it()
{
  while( (micro_tickets = micros()) - last_pwm_pulse <= PWM_FERQUENCY ) ;  // wait until next rising pulse
  last_pwm_pulse= micro_tickets;

  //        76543210
  PORTD |= B00001000;                                        //Set digital poort 3 high

  //          111100
  //          321098
  PORTB |= B00001110;                                        //Set digital poort 9,10,11 high
  
  while( (micros() - last_pwm_pulse) < pwm_output ) ;

  //        76543210
  PORTD &= B11110111;                                        //Set digital poort 3 high

  //          111100
  //          321098
  PORTB &= B11110001;                                        //Set digital poort 9,10,11 high

  
}

#endif

