#include "parallax_servo.h"

parallax_servo::parallax_servo (PwmOut& pin_control, PwmIn& pin_feedback) {
    pin_control.period(0.02);
    pwmOut = &pin_control;
    factor = 1;
    target_pwm_value = 0;
    current_pwm_value = 0;
    ramping_factor = 0;
    pwmIn = &pin_feedback;
}

// set new target pwm value (speed = pwm value)
void parallax_servo::set_speed( double value ) {
    target_pwm_value = value * factor;
    // you can determine ramping factor formula by yourself
    ramping_factor = abs(target_pwm_value - current_pwm_value) / 50;
    if (target_pwm_value > 200) target_pwm_value = 200;
    else if (target_pwm_value < -200) target_pwm_value = -200;
}

void parallax_servo::set_factor( double value ){
    factor = value;
}

// control servo input pwm, also ramping is mainly done in here
void parallax_servo::control(){
    if (current_pwm_value > target_pwm_value) {
        if (current_pwm_value < target_pwm_value + ramping_factor)
            current_pwm_value = target_pwm_value;
        else current_pwm_value -= ramping_factor;
    }
    else if (current_pwm_value < target_pwm_value) {
        if (current_pwm_value > target_pwm_value - ramping_factor)
            current_pwm_value = target_pwm_value;
        else current_pwm_value += ramping_factor;
    }
    pwmOut->write((CENTER_BASE + current_pwm_value) / 20000);
}

// detect the current agle of  feedback 360 servo
void parallax_servo::feedback360()                          // Position monitoring
{                           
    tCycle = pwmIn->period();

    int dc = dutyScale * pwmIn->dutycycle();
    theta = (unitsFC - 1) -                   // Calculate angle
            ((dc - dcMin) * unitsFC) 
                  / (dcMax - dcMin + 1);
    if(theta < 0)                             // Keep theta valid
        theta = 0; 
    else if(theta > (unitsFC - 1)) 
        theta = unitsFC - 1;

            // If transition from quadrant 4 to  
          // quadrant 1, increase turns count. 
    if((theta < q2min) && (thetaP > q3max))
        turns++;
          // If transition from quadrant 1 to  
          // quadrant 4, decrease turns count. 
    else if((thetaP < q2min) && (theta > q3max))
        turns --;

            // Construct the angle measurement from the turns count and
          // current theta value.
    if(turns >= 0)
        angle = (turns * unitsFC) + theta;
    else if(turns <  0)
        angle = ((turns + 1) * unitsFC) - (unitsFC - theta);

      thetaP = theta;                               // Theta previous for next rep
}


