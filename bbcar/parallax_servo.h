#ifndef PARALLAX_SERVO_H
#define PARALLAX_SERVO_H
#include "mbed.h"
#include "PwmIn.h"

#define CENTER_BASE 1500
#define unitsFC 360                          // Units in a full circle
#define dutyScale 1000                       // Scale duty cycle to 1/1000ths
#define dcMin 29                             // Minimum duty cycle
#define dcMax 971                            // Maximum duty cycle
#define q2min unitsFC/4                      // For checking if in 1st quadrant
#define q3max q2min * 3                      // For checking if in 4th quadrant

class parallax_servo {
    public:
        parallax_servo(PwmOut& pin_control, PwmIn& pin_feedback);

        void set_speed( double value );
        void set_factor( double value );
        void control();

        //feedback servo function
        void feedback360();                         // Position monitoring

    // private:

        PwmOut *pwmOut; // pwm output pin pointer
        PwmIn  *pwmIn;  //servo feedback signal

        double factor;	// determine turning rate, 0 means this wheel don't move
        double target_pwm_value; // desire steady state pwm output value
        double current_pwm_value;  // current pwm output value
	    double ramping_factor; // car pwm value change speed each iteration

        //feedback 360 servo 
        volatile int angle, targetAngle=0;              // Global shared variables
        volatile float tCycle;
        volatile int theta;
        volatile int thetaP;
        int dc=0;
        volatile int turns = 0;
};

#endif
