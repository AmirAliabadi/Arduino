
int throttle = 0;
unsigned long last_log = 0;

void setup() {
  Serial.begin(115200); 
  while (!Serial);

  pinMode(6, OUTPUT);

  throttle = 0;

  attachInterrupt(digitalPinToInterrupt(3), ppmRising, RISING);
  
}

volatile unsigned long last_ppm_clock = 99999;
volatile unsigned long current_ppm_clock = 0;
volatile unsigned long ppm_dt = 0;
volatile boolean ppm_read = true;
volatile unsigned short ppm_channel = 0;
volatile unsigned long ppm_channels[6] = {1500,1500,1500,1500,1500,1500};

void ppmRising() {
//  noInterrupts();
  ppm_read = false;
    {
      current_ppm_clock = micros();
      ppm_dt = current_ppm_clock - last_ppm_clock;
      if( ppm_dt >= 4000 ) {
        ppm_channel = 0;
      }
      else {
        ppm_channels[ppm_channel]=ppm_dt; 
        ppm_channel++;
        if( ppm_channel > 6 ) ppm_channel = 0;
      }
      last_ppm_clock = current_ppm_clock;   
    }
  ppm_read = true;
//  interrupts();    
}

void loop() 
{        
  if( millis() - last_log  > 3 ) 
  {
    read_throttle(); 
    Serial.println( throttle );     
    last_log = millis();
  }
}

void read_throttle() {
  if( ppm_read ) {
    cli();
    unsigned long foo = ppm_channels[2] ;    
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
