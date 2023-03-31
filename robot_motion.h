/*
 * File:   main.c
 * Author: Nathan Ludlow, Solomon Olmstead, Bryant Jepsen, Dallin Davis
 *
 * Created on March 2023, 3:33 PM
 */

/*
 * Usage:
 * 
 * move_linear_at_velocity(double velocity_m_s)
 * @Param velocity_m_s The speed in m/s that the base will move at. Positive is 
 * forwards, negative is backwards.
 * @Return void
 * @Definition Computes the speed the wheels should turn at and then starts them
 * moving at the correct voltage for the given speed.
 * 
 * 
 * move_linear_to_position(double velocity_m_s, double distance_m, bool wait)
 * @Param velocity_m_s The absolute value of the speed in m/s that the base will
 *  move at. This value should always be positive by convention.
 * @Param distance_m The total distance the base should travel in a linear 
 * motion from its current position. Positive is forwards, negative is 
 * backwards.
 * @Param wait Determines if the function should be blocking or non-blocking
 * @Return void
 * @Definition Moves the base from its current position to the set position
 * 
 * 
 * pivot_at_angular_velocity(double angular_velocity_deg_s)
 * @Param angular_velocity_deg_s The speed in degs/s that the base will pivot 
 * at. Positive is clockwise, negative is counterclockwise.
 * @Return void
 * @Definition Rotates the base about the center of the wheels at the specified
 * angular velocity. 
 * 
 * 
 * pivot_to_angle(double angular_velocity_deg_s, double angle_deg, bool wait)
 * @Param angular_velocity_deg_s The abolute value of the angular speed in 
 * degs/s that the base will pivot at. This value should always be positive 
 * by convention.
 * @Param angle_deg The total angle the base should pivot from its current 
 * position. Positive is clockwise, negative is counterclockwise.
 * @Param wait Determines if the function should be blocking or non-blocking
 * @Return void
 * @Definition Rotates the base from the current angle to the set angle by
 * pivoting about the center of the wheels
 * 
 * 
 * move_curved_at_angular_velocity(double angular_velocity_deg_s, 
 *                                 double pivot_point)
 * @Param angular_velocity_deg_s The angular velocity the base will rotate at in
 * degs/sec
 * @Param pivot_point The point the robot will rotate about in m from the center
 * of the base to along the axis of the wheels. Positive is a rotation point to
 * the right, and negative is to the left.
 * @Return void
 * @Definition Computes the speed the wheels should turn at and then starts them
 * moving at the correct voltage for the given speed. This will move the base in
 * a curved path about a point of rotation outside the center of the wheels. 
 * 
 * 
 * wait(double seconds)
 * @Param seconds The amount of time for the function to wait in seconds
 * @Return void
 * @Definition A blocking function to wait for the specified number of seconds
 * 
 */

#ifndef ROBOT_MOTION_H
#define	ROBOT_MOTION_H

#include "configuration.h"
    
enum DIRECTION {forward, backward};
#define _WHEEL_WIDTH 256 // mm distance between wheels
#define _WHEEL_RADIUS 44.45
#define _FUDGE_FACTOR_LEFT 1.10294
#define _FUDGE_FACTOR_RIGHT 1.12294
#define _FUDGE_FACTOR_TURNING 1.0
#define _MOTOR_MAX_RAD_S 15.7 // post gearbox speed 15.7 942.478
#define _MOTOR_MAX_REV_S 150
#define _PWM_PERIOD (2656/postscaling)// With 1:16 postscaling and FRCDIV
#define _TIMER_PERIOD (0.000064*postscaling) // Gives us a max count of 4*postscaling seconds max

void config_timer_1_interupt(void) {
    _T1IP = 2; // Priority
    timer_1_interrupt_flag = 0;
    timer_1_interrupt_enable = 0;
}

void config_timer_1(void) {
    T1CON = 0;
    T1CONbits.TCKPS = 0b11; // 1:256
    T1CONbits.TCS = 0;
    // Max count of 60 seconds with 1:16 postscaling and 1:256 prescaling
    timer_1_period = 0; 
    timer_1 = 0;
    timer_1_enable = 0;
}
    
void config_motors(void) {
    OC2CON1 = 0;
    OC2CON2 = 0;
    OC2CON1bits.OCTSEL = 0b111;
    OC2CON1bits.OCM = 0b110;
    OC2CON2bits.OCTRIG = 0;
    OC2CON2bits.SYNCSEL = 0b11111;
    right_motor_pwm_period = _PWM_PERIOD; // 1.5 kHz
    right_motor_pwm_duty_cycle = 0;
    
    OC3CON1 = 0;
    OC3CON2 = 0;
    OC3CON1bits.OCTSEL = 0b111;
    OC3CON1bits.OCM = 0b110;
    OC3CON2bits.OCTRIG = 0;
    OC3CON2bits.SYNCSEL = 0b11111;
    left_motor_pwm_period = _PWM_PERIOD; // 1.5 kHz
    left_motor_pwm_duty_cycle = 0;
    
    left_motor_in_1 = 0;
    left_motor_in_2 = 0;
    right_motor_in_3 = 0;
    right_motor_in_4 = 0;
    
    config_timer_1();
    config_timer_1_interupt();
}

#define _RAD_TO_DEG 57.2958
static double degs_to_rads(double degs) {
    return degs/_RAD_TO_DEG;
}

void set_left_motor_direction(enum DIRECTION direction) {
    if (direction == forward) {
        left_motor_in_1 = 0;
        left_motor_in_2 = 1;
    }
    else {
        left_motor_in_1 = 1;
        left_motor_in_2 = 0;
    }
}

void set_right_motor_direction(enum DIRECTION direction) {
    if (direction == forward) {
        right_motor_in_3 = 0;
        right_motor_in_4 = 1;
    }
    else {
        right_motor_in_3 = 1;
        right_motor_in_4 = 0;
    }
}

void move_linear_at_velocity(double velocity_m_s) {   
    velocity_m_s *= 1000;
    // Set speeds and directions of left wheel
    double left_wheel_velocity = (velocity_m_s)/(_WHEEL_RADIUS);
    if (left_wheel_velocity > 0) {
        set_left_motor_direction(forward);
    }
    else{
        set_left_motor_direction(backward);
    }
    int left_pwm_value = (int)fabs((left_wheel_velocity/_MOTOR_MAX_RAD_S) * 
                                    _PWM_PERIOD * _FUDGE_FACTOR_LEFT);
    if (left_pwm_value > _PWM_PERIOD) {
        left_pwm_value = _PWM_PERIOD;
    }
    left_motor_pwm_duty_cycle = left_pwm_value;
    
    // Set speeds and directions of right wheel
    double right_wheel_velocity = (velocity_m_s)/(_WHEEL_RADIUS);
    if (right_wheel_velocity > 0) {
        set_right_motor_direction(forward);
    }
    else{
        set_right_motor_direction(backward);
    }
    int right_pwm_value = (int)fabs((right_wheel_velocity/_MOTOR_MAX_RAD_S) * 
                                     _PWM_PERIOD * _FUDGE_FACTOR_RIGHT);
    if (right_pwm_value > _PWM_PERIOD) {
        right_pwm_value = _PWM_PERIOD;
    }
    right_motor_pwm_duty_cycle = right_pwm_value;
}

// FIXME Also potentially tune the fudge factor for friction generated while 
// turning
void pivot_at_angular_velocity(double angular_velocity_deg_s) {
    double angular_velocity_rad_s = degs_to_rads(angular_velocity_deg_s);
    // Declare radius of rotation and set to 0 to always turn about center
    double R = 0; 
    // Set speeds and directions of motors
    double left_wheel_velocity = ((angular_velocity_rad_s * (R 
                                   + (_WHEEL_WIDTH/2))))/(_WHEEL_RADIUS);
    if (left_wheel_velocity > 0) {
        set_left_motor_direction(forward);
    }
    else{
        set_left_motor_direction(backward);
    }
    int left_pwm_value = (int)fabs((left_wheel_velocity/_MOTOR_MAX_RAD_S) 
                                    * _PWM_PERIOD * _FUDGE_FACTOR_LEFT
                                    * _FUDGE_FACTOR_TURNING);
    if (left_pwm_value > _PWM_PERIOD) {
        left_pwm_value = _PWM_PERIOD;
    }
    left_motor_pwm_duty_cycle = left_pwm_value;
    
    double right_wheel_velocity = ((angular_velocity_rad_s * (R 
                                    - (_WHEEL_WIDTH/2))))/(_WHEEL_RADIUS);
    if (right_wheel_velocity > 0) {
        set_right_motor_direction(forward);
    }
    else{
        set_right_motor_direction(backward);
    }
    int right_pwm_value = (int)fabs((right_wheel_velocity/_MOTOR_MAX_RAD_S) 
                                     * _PWM_PERIOD * _FUDGE_FACTOR_RIGHT 
                                     * _FUDGE_FACTOR_TURNING);
    if (right_pwm_value > _PWM_PERIOD) {
        right_pwm_value = _PWM_PERIOD;
    }
    right_motor_pwm_duty_cycle = right_pwm_value;
}

// TODO: Check that this function works and that positive R is to the right of 
// the base, also check what happens when angular velocity and R are opposite
void move_curved_at_angular_velocity(double angular_velocity_deg_s, 
                                     double pivot_point) {
    // FIXME: fabs() may not be needed
    double angular_velocity_rad_s = degs_to_rads(angular_velocity_deg_s); 
    double R = pivot_point; // Define radius of rotation
    // Set speeds and directions of motors
    double left_wheel_velocity = ((angular_velocity_rad_s * (R 
                                   + (_WHEEL_WIDTH/2))))/(_WHEEL_RADIUS);
    if (left_wheel_velocity > 0) {
        set_left_motor_direction(forward);
    }
    else{
        set_left_motor_direction(backward);
    }
    int left_pwm_value = (int)fabs((left_wheel_velocity/_MOTOR_MAX_RAD_S) 
                                    * _PWM_PERIOD * _FUDGE_FACTOR_LEFT);
    if (left_pwm_value > _PWM_PERIOD) {
        left_pwm_value = _PWM_PERIOD;
    }
    left_motor_pwm_duty_cycle = left_pwm_value;
    
    double right_wheel_velocity = ((angular_velocity_rad_s * (R 
                                    - (_WHEEL_WIDTH/2))))/(_WHEEL_RADIUS);
    if (right_wheel_velocity > 0) {
        set_right_motor_direction(forward);
    }
    else{
        set_right_motor_direction(backward);
    }
    int right_pwm_value = (int)fabs((right_wheel_velocity/_MOTOR_MAX_RAD_S) 
                                     * _PWM_PERIOD * _FUDGE_FACTOR_RIGHT);
    if (right_pwm_value > _PWM_PERIOD) {
        right_pwm_value = _PWM_PERIOD;
    }
    right_motor_pwm_duty_cycle = right_pwm_value;
}

void turn_motors_at_speed(double angular_velocity_left_rev_s, 
                          double angular_velocity_right_rev_s) { 
    
    if (angular_velocity_left_rev_s > 0) {
        set_left_motor_direction(forward);
    }
    else{
        set_left_motor_direction(backward);
    }
    int left_pwm_value = (int)fabs((angular_velocity_left_rev_s/_MOTOR_MAX_REV_S) * 
                                    _PWM_PERIOD * _FUDGE_FACTOR_LEFT);
    if (left_pwm_value > _PWM_PERIOD) {
        left_pwm_value = _PWM_PERIOD;
    }
    left_motor_pwm_duty_cycle = left_pwm_value;
    
    
    if (angular_velocity_right_rev_s > 0) {
        set_right_motor_direction(forward);
    }
    else{
        set_right_motor_direction(backward);
    }
    int right_pwm_value = (int)fabs((angular_velocity_right_rev_s/_MOTOR_MAX_REV_S) * 
                                    _PWM_PERIOD * _FUDGE_FACTOR_LEFT);
    if (right_pwm_value > _PWM_PERIOD) {
        right_pwm_value = _PWM_PERIOD;
    }
    right_motor_pwm_duty_cycle = right_pwm_value;
}

enum POSITION_MODE {none, linear_motion, angular_turn, wait_time};
enum POSITION_MODE current_mode = none;
int timer_counts_until_stop = 0;

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {   
    if (current_mode == linear_motion) {
        move_linear_at_velocity(0);
        timer_1_enable = 0;
        timer_1 = 0;
        timer_1_interrupt_enable = 0;
        current_mode = none;
    }
    else if (current_mode == angular_turn) {
        pivot_at_angular_velocity(0);
        timer_1_enable = 0;
        timer_1 = 0;
        timer_1_interrupt_enable = 0;
        current_mode = none;
    }
    else if (current_mode == wait_time) {
        timer_1_enable = 0;
        timer_1 = 0;
        timer_1_interrupt_enable = 0;
        current_mode = none;
    }
    timer_1_interrupt_flag = 0; // set flag low
}

// FIXME: Use trapezoidal speed curve on starting side (not ending side due to)
// gearbox inertia and friction to compute a more accurate time
void move_linear_to_position(double velocity_m_s, double distance_m, 
                             bool wait) {
    velocity_m_s *= 1000;
    distance_m *= 1000;
    double seconds_to_wait = fabs(distance_m/velocity_m_s);
    timer_1_period = (int)round(seconds_to_wait/_TIMER_PERIOD);
    current_mode = linear_motion;
    timer_1_enable = 1;
    timer_1 = 0;
    timer_1_interrupt_enable = 1;
    timer_1_interrupt_flag = 0;
    move_linear_at_velocity(velocity_m_s);
    
    if (wait) {
        while (current_mode != none) {
            Nop();
        }
    }
}

// FIXME: Use trapezoidal speed curve on starting side (not ending side due to)
// gearbox inertia and friction to compute a more accurate time. 
void pivot_to_angle(double angular_velocity_deg_s, double angle_deg, 
                    bool wait) {
    double seconds_to_wait = fabs(angle_deg/angular_velocity_deg_s);
    // Multiply seconds into timer counts
    timer_1_period = (int)round(seconds_to_wait/_TIMER_PERIOD);
    current_mode = angular_turn;
    timer_1_enable = 1;
    timer_1 = 0;
    timer_1_interrupt_enable = 1;
    timer_1_interrupt_flag = 0;
    pivot_at_angular_velocity(angular_velocity_deg_s);
    
    if (wait) {
        while (current_mode != none) {
            Nop();
        }
    }
}

void wait(double seconds) {
    double seconds_to_wait = seconds;
    // Multiply seconds into timer counts
    timer_1_period = (int)round(seconds_to_wait/_TIMER_PERIOD);
    current_mode = wait_time;
    timer_1_enable = 1;
    timer_1 = 0;
    timer_1_interrupt_enable = 1;
    timer_1_interrupt_flag = 0;

    while (current_mode != none) {
        Nop();
    }
}

#endif	/* ROBOT_MOTION_H */

