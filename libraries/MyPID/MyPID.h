#ifndef PID_h
#define PID_h

#include <stdlib.h>
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
#endif
#include "pins_arduino.h"

class MyPID {
	public:
	
	MyPID(float p, float i, float d, float ref, float in, float maxCtrl, float minCtrl) {
		Kp = p;
		Ki = i;
		Kd = d;
		prevTime = micros();
		prevRef = ref;
		prevInput = in;
		
		minLimit = minCtrl;
		maxLimit = maxCtrl;
		
		iTerm = 0;
		
		sum_errors = 0;
		last_output = 0;		
	}
	
	void resetITerm() {
		iTerm = 0;
		prevTime = micros();
	}

	float calculate(float ref, float input, bool debug) {
		// Calculate sampling time
		long dt = (micros() - prevTime); // Convert to seconds
		if( dt > 0 ) {
		float dt_float = (float)dt ;//* 0.001 ;
		
		float error = ref - input;
		
		pTerm  =  Kp * (ref - input);
		sum_errors = sum_errors + error;
		
		if( debug ) {
			// # -0.50 7300 -316.62 0.03 -29.65
/* 			Serial.print("# ");
			Serial.print(error);
			Serial.print(" ");
			Serial.print(dt);
			Serial.print(" ");
			Serial.print(sum_errors);
			Serial.print(" ");
			Serial.print(Ki);
			Serial.print(" "); */
		}			
		
		iTerm +=  Ki/100000.0 * (error * dt);
			
		if( debug ) {
			Serial.print(iTerm);
			Serial.print("\t");
		}
		
		dTerm  = -Kd * (input - prevInput)/dt_float; // dError/dt = - dInput/dt	

		if( iTerm >  75.0 ) iTerm =  75.0;
		if( iTerm < -75.0 ) iTerm = -75.0;
		
		// Calculate control
		float output = pTerm + iTerm + dTerm;
		
		if( debug ) { Serial.println(output); }
		
		// Anti-windup
		if (output > maxLimit) {
			iTerm -= output - maxLimit;
			output = maxLimit;
		} else if ( output < minLimit ){
			iTerm += minLimit - output;
			output = minLimit;
		} else {
			//output is output
		}
		
		// Update state
		prevTime = micros();
		prevRef = ref;
		prevInput = input;
		last_output = output;
		}
		
		return last_output;
	}

	void setControlCoeffs(float* pidVector) {
		Kp = pidVector[0];
		Ki = pidVector[1];
		Kd = pidVector[2];
	}
	
	private:
	long prevTime;
	float prevRef;
	float prevInput;
	float pTerm;
	float iTerm;
	float dTerm;
	float sum_errors;
	float minLimit;
	float maxLimit;
	float Kp, Ki, Kd;
	float last_output;
};

#endif