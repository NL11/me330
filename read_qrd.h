/*
 * File:   main.c
 * Author: Nathan Ludlow, Solomon Olmstead, Bryant Jepsen, Dallin Davis
 *
 * Created on March 2023
 */

// File that includes code to read the current state of a QRD sensor

#ifndef READ_QRD_H
#define	READ_QRD_H

#include "configuration.h"
    
// Used for line following
#define ALPHA 0.2

// Used for task detection and ball color detection
#define QRD_THRESHOLD 3000
#define BALL_QRD_THRESHOLD 3500
#define black true
#define white false
#define RIGHT_QRD_VALUE_OFFSET -485

void config_qrds(void) {
    // Line QRDS
    _TRISB14 = 1;    // pin 17 AN10
    _ANSB14 = 1;     // pin 17 AN10
    
    _TRISB15 = 1;    // pin 18 AN9
    _ANSB15 = 1;     // pin 18 AN9
    
    _TRISB13 = 1;    // pin 16 AN11
    _ANSB13 = 1;     // pin 16 AN11
   
    // Task QRD
    _TRISA3 = 1;    // pin 8 AN14
    _ANSA3 = 1;     // pin 8 AN14
    
    // Ball QRD
    _TRISB4 = 1;    // pin 8 AN14
    _ANSB4 = 1;     // pin 8 AN14
    
    enable_ad_auto_sample = 0;    // Disable A/D module during configuration
    
    // AD1CON1
    _MODE12 = 1;  // 12-bit resolution
    _FORM = 0;    // unsigned integer output
    _SSRC = 7;    // auto convert
    _ASAM = 1;    // auto sample

    // AD1CON2
    _PVCFG = 0;   // use VDD as positive reference
    _NVCFG = 0;   // use VSS as negative reference
    _BUFREGEN = 1;// store results in buffer corresponding to channel number
    _CSCNA = 1;   // scanning mode
    _SMPI = 4;    // begin new sampling sequence after every sample
    // number of pins you are sampling since this determines that the buffer 
    // Resets its scan after N-1 number of pins scanned
    _ALTS = 0;    // sample MUXA only

    // AD1CON3
    _ADRC = 0;    // use system clock
    _SAMC = 0;    // sample every A/D period change to be N-1 where N is the 
    _ADCS = 0x3F; // TAD = 64*TCY

    // AD1CSS -- Choose which channel/pin to scan
    _CSS9 = 1; // AN9
    _CSS10 = 1; // AN10
    _CSS11 = 1; // AN11
    _CSS14 = 1; // AN14
    _CSS15 = 1; // AN15

    enable_ad_auto_sample = 1;    // enable module
}

static int averaged_right_qrd_value = 0;
// return value between 0 and 4096 of filtered sensor reading
int read_right_qrd(void) {
    averaged_right_qrd_value = averaged_right_qrd_value*ALPHA + right_qrd*(1.0-ALPHA);
    return averaged_right_qrd_value + RIGHT_QRD_VALUE_OFFSET;
}

static int averaged_center_qrd_value = 0;
// return value between 0 and 4096 of filtered sensor reading
int read_center_qrd(void) {
    averaged_center_qrd_value = averaged_center_qrd_value*ALPHA + center_qrd*(1.0-ALPHA);
    return averaged_center_qrd_value;
}

static int averaged_left_qrd_value = 0;
// return value between 0 and 4096 of filtered sensor reading
int read_left_qrd(void) {
    averaged_left_qrd_value = averaged_left_qrd_value*ALPHA + left_qrd*(1-ALPHA);
    return averaged_left_qrd_value;
}

bool read_task_qrd(void) {
    if (task_qrd >= QRD_THRESHOLD) {
        return black;
    }
    return white;
}

bool read_ball_qrd(void) {
    if (ball_qrd >= BALL_QRD_THRESHOLD) {
        return black;
    }
    return white;
}

#endif	/* READ_QRD_H */

