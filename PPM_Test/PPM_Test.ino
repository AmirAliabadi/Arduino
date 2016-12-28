
int throttle = 0;
unsigned long last_log = 0;

void setup() {
  Serial.begin(115200); 
  while (!Serial);

  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);

  throttle = 0;

  attachInterrupt(digitalPinToInterrupt(3), ppmRising, RISING);
  
}

volatile unsigned long last_ppm_clock = 99999;
volatile unsigned long current_ppm_clock = 0;
volatile unsigned long ppm_dt = 0;
volatile boolean ppm_read = true;
volatile boolean ppm_sync = false;
volatile unsigned short ppm_channel = 0;
volatile unsigned long ppm_channels[7] = {4000,1500,1500,1500,1500,1500,1500};

void ppmRising() {
//  noInterrupts();
  ppm_read = false;
    {
      current_ppm_clock = micros();
      ppm_dt = current_ppm_clock - last_ppm_clock;
      if( ppm_dt >= 4000 ) {
        ppm_sync = true;
        ppm_channel = 0;
        ppm_channels[ppm_channel]=ppm_dt;         
      }
      else {
        if( ppm_sync ) {
          ppm_channel++;
          if( ppm_channel > 6 ) ppm_sync = false;
          else ppm_channels[ppm_channel]=ppm_dt; 
        }
      }
      last_ppm_clock = current_ppm_clock;   
    }
  ppm_read = true;
//  interrupts();    
}

unsigned long foo ;
unsigned long micros_count = 0;
void loop() 
{        
  read_throttle();   
  if( millis() - last_log  > 5 ) 
  {
    Serial.println( throttle );     
    last_log = millis();
  }

  if( micros() - micros_count >= foo ) {
      digitalWrite( 6, !digitalRead(6) );
      micros_count = micros();
  }
}

void read_throttle() {
  if( ppm_read ) {
    cli();
    foo = ppm_channels[3] ;    
    sei();
    
    if( foo < 1490 && throttle > 0 ) {
      if( throttle > 1 ) throttle -= 1;
      else throttle = 0;
    } 

    if( foo < 1200 && throttle > 0 ) {
      if( throttle > 50 ) throttle -= 50;
      else throttle = 0;
    }     
    
    if ( foo > 1510 && throttle < 1000 ) {
     throttle += 1 ;
    }
  }
}
