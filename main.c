/*
 * File:   main.c
 * Author: Nathan Ludlow, Solomon Olmstead
 *
 * Created on March 2023
 */

/*
 * Main function for the Mars rover team project for team CAnemone. 
 */

#include "xc.h"
#include "configuration.h"
#include "robot_motion.h"
#include "line_follow.h"
#include "detect_task.h"
#include "sample_collection_return.h"
#include "canyon_navigation.h"
#include "read_qrd.h"
#include "read_ir_range_sensors.h"
#include "set_laser.h"
#include "satillite_transmission.h"
#include "eqipment_servicing.h"
#include "leave_lander.h"

// 8 MHz oscilator with postscaling
#pragma config FNOSC = FRCDIV // 8 MHz
// Disable pin 8 clock output
#pragma config OSCIOFNC = OFF
// Setup for pins 9 and 10 by disabling secondary oscillator peripheral
#pragma config SOSCSRC = DIG
// Disable watchdog
#pragma config FWDTEN = OFF // 0
#pragma config WINDIS = OFF // 0

int main(void) {
    
    set_postscaling();
    initialize_registers();
    config_motors();
    move_linear_at_velocity(0);
    config_qrds();
    config_ir_range_finders();
    configure_servo();
    set_door_servo(50);
    config_laser();
    // Set initial task here!
    enum task_type current_task = STARTUP;
    // Wait for 2 seconds before starting to allow the base to turn on 
    // properly and allow the user to move away from the base after turning on
    wait(2); 
    
    unsigned int interations_count = 0;
    while(1){    
        switch(current_task) {
            case (TEST) :
                // Put code in here that you want to test!
                break;
            case(IDLE):
                break;
            case (STARTUP):
                leave_lander();
                current_task = LINE_FOLLOW;
                break;
            case (LINE_FOLLOW):
                line_follow();
                current_task = detect_task();
                break;  
            case (SAMPLE_COLLECTION):
                collect_sample();
                current_task = LINE_FOLLOW;
                reset_line_follow_errors();
                break;
            case (SAMPLE_RETURN):
                interations_count++;
                if (interations_count >= 450) {
                    return_sample();
                    reset_line_follow_errors();
                    interations_count = 0;
                    current_task = LINE_FOLLOW;
                }
                else {
                    line_follow();
                }
                break;
            case (CANYON_NAVIGATION):
                current_task = navigate_canyon();
                reset_line_follow_errors();
                break;
            case(EQUIPMENT_SERVICING):
                service_equipment();
                current_task = LINE_FOLLOW;
                reset_line_follow_errors();
                break;
            case(DATA_TRANSMISSION):
                if (interations_count == 0) {
                    move_linear_at_velocity(-0.50);
                    wait(0.25);
                    pivot_to_angle(-140, -104, true);  // 90 deg turn clockwise
                }
                interations_count++;
                if (interations_count >= 2300) {
                    move_linear_at_velocity(0);
                    transmit_to_satilite();
                    interations_count = 0;
                    current_task = IDLE;
                }
                else if (interations_count >= 700) {
                    move_linear_at_velocity(0.40);
                }
                else {
                    line_follow();
                }
                break;
            default:
                current_task = IDLE;
        }
    }
    
    move_linear_at_velocity(0); 
    set_door_servo(50);
    
    return 0;
}


