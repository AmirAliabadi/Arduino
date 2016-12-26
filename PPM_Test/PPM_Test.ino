
int throttle = 0;
unsigned long last_log = 0;

void setup() {
  Serial.begin(115200); 
  while (!Serial);

  pinMode(6, OUTPUT);

  throttle = 0;

  attachInterrupt(digitalPinToInterrupt(3), ppmRising, RISING);
  
}

unsigned long last_ppm_clock = 99999;
unsigned long current_ppm_clock = 0;
unsigned long ppm_dt = 0;
volatile boolean ppm_read = true;
volatile unsigned int ppm_channel = 0;
volatile unsigned int ppm_channels[6];

void ppmRising() {
  noInterrupts();
    ppm_read = false;
    {
      current_ppm_clock = micros();
      ppm_dt = current_ppm_clock - last_ppm_clock;
      if( ppm_dt >= 4000 ) {
        ppm_channel = 0;
      }
      else {
        ppm_channels[ppm_channel++] = ppm_dt ;
      }
      last_ppm_clock = current_ppm_clock;   
    }
    ppm_read = true;
  interrupts();    
}

void loop() 
{        
  if( millis() - last_log  > 500 ) 
  {
    read_throttle();
    Serial.println( throttle );  
    last_log = millis();
  }
}

void read_throttle() {
  if( ppm_read ) {}
  if( ppm_channels[2] == 123 ) {}
  return ;
  
  if( ppm_read ) {
    
    if( ppm_channels[2] < 1490 && throttle > 0 ) {
     throttle -= 1;
    } 
    
    if ( ppm_channels[2] > 1510 && throttle < 1000 ) {
     throttle += 1 ;
    }
  }
}
