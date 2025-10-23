
#ifndef SERVOMOTOR_HPP
#define SERVOMOTOR_HPP


class ServoMotor{
	
	private: 
		//Attributes
		int pin;
		int offset;
		int min_ms;
		int max_ms;
		
	public:
		//Constructor & Destructor
		ServoMotor(int pwm_pin, int start_position, int min_ms = 5, int max_ms = 25, int offset = 3);
		~ServoMotor();
		
		//Methods
		void servoWriteMS(int ms);
		int ADC_to_MS(int adc_value);
		int read_minMS();
		int read_maxMS();
		
};

#endif		
				
