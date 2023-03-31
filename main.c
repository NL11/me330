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
#include "lander.h"

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
    enum task_type current_task = LEAVE_LANDER;
    // Wait for 2 seconds before starting to allow the base to turn on 
    // properly and allow the user to move away from the base after turning on
    wait(2); 
    
    while(1){    
        switch(current_task) {
            case (TEST) :
                // Put code in here that you want to test!
                break;
            case(IDLE):
                break;
            case (LEAVE_LANDER):
                current_task = leave_lander();
                break;
            case (LINE_FOLLOW):
                line_follow();
                current_task = detect_task();
                break;  
            case (SAMPLE_COLLECTION):
                current_task = collect_sample();
                break;
            case (SAMPLE_RETURN):
                current_task = return_sample();
                break;
            case (CANYON_NAVIGATION):
                current_task = navigate_canyon();
                break;
            case(EQUIPMENT_SERVICING):
                current_task = service_equipment();
                break;
            case(RETURN_TO_LANDER):
                current_task = return_to_lander();
                break;
            case(DATA_TRANSMISSION):
                current_task = transmit_to_satilite();
                break;
            default:
                current_task = IDLE;
        }
    }
    
    move_linear_at_velocity(0); 
    set_door_servo(50);
    
    return 0;
}


