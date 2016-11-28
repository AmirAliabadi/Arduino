#ifdef USE_BITBANG_2


#define PWM_INCREMENT 10
#define PWM_FERQUENCY 2800
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

/*
Arduino Digital 
Pin   Port Pin
0     PD0
1     PD1
2     PD2
3     PD3
4     PD4
5     PD5
6     PD6
7     PD7

8     PB0
9     PB1
10    PB2
11    PB3
12    PB4
13    PB5  
*/
  
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.

  DDRD |= B00001000;                                           //Configure digital poort 4, 5, 6 and 7 as output.
  DDRB |= B00001110;                                           //Configure digital poort 12 and 13 as output.

  //PORTD |= B00001000;                                        //Set digital port 3 high
  //PORTB |= B00001110;                                        //Set digital port 9,10,11 high

}

unsigned long micro_tickets = 0;
void do_it()
{  
  while( (micro_tickets = micros()) - last_pwm_pulse <= PWM_FERQUENCY ) ;  // wait until next rising pulse
  last_pwm_pulse= micro_tickets;
  
  PORTD |= B00001000;                                        //Set digital port 3 high
  PORTB |= B00001110;                                        //Set digital port 9,10,11 high

  int motors =    0x00001111;
  while( motors | 0x00000000 )
  {
      if((micros() - last_pwm_pulse) >= pwm_output_a) { PORTD &= B11110111; motors &= 0x00000111; }
      if((micros() - last_pwm_pulse) >= pwm_output_b) { PORTB &= B11110111; motors &= 0x00001011; }
      if((micros() - last_pwm_pulse) >= pwm_output_c) { PORTB &= B11111011; motors &= 0x00001101; }
      if((micros() - last_pwm_pulse) >= pwm_output_d) { PORTB &= B11111101; motors &= 0x00001110; }
  }
}

#endif

