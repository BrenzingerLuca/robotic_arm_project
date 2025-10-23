#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>
#include <iostream>
#include "ServoClass.hpp"


//Constructor
ServoMotor::ServoMotor(int pwm_pin, int start_position, int min_ms, int max_ms, int offset)
	: min_ms(min_ms), max_ms(max_ms), offset(offset)
{
	pin = pwm_pin;		
	//Initialisation of the softPWM Pin with starting value (=0) and max value (200ms)
	wiringPiSetupGpio();
	softPwmCreate(pwm_pin,  0, 200);
	servoWriteMS(start_position);		
	//Print command to check the initialisation
	std::cout <<"Servo initialized on pin " << pwm_pin
		  <<" with minimal ms" << this->min_ms
		  <<" and maximal ms" << this->max_ms << std::endl;  
}

//Destructor
ServoMotor::~ServoMotor(){
}

void ServoMotor::servoWriteMS(int ms){     //specific the unit for pulse(5-25ms) with specific duration output by servo pin: 0.1ms
    if(ms > this->max_ms)
        ms = this->max_ms;
    if(ms < this->min_ms)
        ms = this->min_ms;
    softPwmWrite(this->pin,ms);
}

int ServoMotor::read_minMS(){
    return min_ms;
}

int ServoMotor::read_maxMS(){
    return max_ms;
}

//Standard mapping function later used to map potentiometer values onto degree values
int ServoMotor::ADC_to_MS(int adc_value){
    return min_ms + (adc_value * (max_ms - min_ms)) / 255;
}


