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

// 8 MHz oscilator with postscaling
#pragma config FNOSC = FRCDIV
// Disable pin 8 clock output
#pragma config OSCIOFNC = OFF
// Setup for pins 9 and 10
#pragma config SOSCSRC = DIG

int main(void) {
    
    set_postscaling();
    
    initialize_registers();
    config_motors();
    config_qrds();
    config_ir_range_finders();
    // Set initial task here!
    enum task_type current_task = LINE_FOLLOW;
    // Wait for 2 seconds before starting to allow the base to turn on 
    // properly and allow the user to move  away from the base after turning on
    wait(2); 
    move_linear_at_velocity(0); 
    
    while(1){    
        update_distance_traveled();
        switch(current_task) {
            case (TEST) :
                // Put code in here that you want to test!
                break;
            case(IDLE):
                break;
            case (STARTUP):
                // Code to move to line then turn to start motion
                move_linear_to_position(0.5, 0.4, true); // Two tiles
                pivot_to_angle(180, -105, true);  // 90 deg turn counterclockwise
                current_task = LINE_FOLLOW;
                break;
            case (LINE_FOLLOW):
                line_follow();
                current_task = detect_task();
                break;  
            case (SAMPLE_COLLECTION):
//                collect_sample();
                current_task = LINE_FOLLOW;
                break;
            case (SAMPLE_RETURN):
                move_linear_at_velocity(0);
                wait(2);
//                return_sample();
                current_task = LINE_FOLLOW;
                break;
            case (CANYON_NAVIGATION):
                current_task = navigate_canyon();
                break;
            case(EQUIPMENT_SERVICING):
                break;
            case(DATA_TRANSMISSION):
                break;
            default:
                current_task = IDLE;
        }
    }
    
    return 0;
}


