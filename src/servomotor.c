/**************************************************************************//**
 *
 * @file rotary-encoder.c
 *
 * @author (Jimmy Hua)
 * @author (Abdelrahman Elhaj)
 *
 * @brief Code to control a servomotor.
 *
 ******************************************************************************/

/*
 * ComboLock GroupLab assignment and starter code (c) 2022-24 Christopher A. Bohn
 * ComboLock solution (c) the above-named students
 */

#include <CowPi.h>
#include "servomotor.h"
#include "interrupt_support.h"

#define SERVO_PIN           (22)
#define PULSE_INCREMENT_uS  (500) // 500 or 100 us?
#define SIGNAL_PERIOD_uS    (20000) // every 20ms mili (20,000μs micro)

static int volatile pulse_width_us;

static void handle_timer_interrupt();

void initialize_servo() {
    cowpi_set_output_pins(1 << SERVO_PIN);
    center_servo();
    register_periodic_timer_ISR(0, PULSE_INCREMENT_uS, handle_timer_interrupt);
}

char *test_servo(char *buffer) {
    //  Add code to call the center_servo(), rotate_full_clockwise(), and rotate_full_counterclockwise() functions as necessary for Requirements 2-3
    //  Add code to populate buffer with the strings required by Requirement 4
    //  Compile and upload your code, and confirm that the correct strings are displayed
    if (cowpi_left_button_is_pressed){  // When (and only when) the left pushbutton is pressed, the servo moves to the center position
        center_servo();    
        sprintf(buffer, "SERVO: center"); 
    //  look for wait time?
    } else if (cowpi_left_button_is_pressed == false && cowpi_left_switch_is_in_left_position ){   // When the left switch is in the left position (and the pushbutton is not pressed), the servo deflects fully clockwise
        rotate_full_clockwise();    
        sprintf(buffer, "SERVO: left");
    } else if (cowpi_left_button_is_pressed == false && cowpi_left_switch_is_in_right_position ){  //  When the left switch is in the right position (and the pushbutton is not pressed), the servo deflects fully clockwise
        rotate_full_counterclockwise();     
        sprintf(buffer, "SERVO: right");
    }
    return buffer;
}

void center_servo() {   // 1500μs is 1.5 ms  the central position
    pulse_width_us = 1500;
}

void rotate_full_clockwise() {  //   500μs pulse directs the servomotor to rotate fully clockwise,
    pulse_width_us = 500;
}

void rotate_full_counterclockwise() {   //  2500μs pulse directs the servomotor to rotate fully counter clockwise
    pulse_width_us = 2500;
}

// ( ) Add code so that when the time until the next falling edge is 0μs:
// •Finish the pulse by setting output pin 22 to 0.
// ( ) Add code to update the time remaining until the next rising edge and the time until
// the next falling edge, to reflect the time that has elapsed between timer interrupts.
static void handle_timer_interrupt() {
// Add variables to track the time until the next rising edge of the pulse and the next falling edge of the pulse
    static int32_t rising_edge = 0;
    static int32_t falling_edge = 0;

// ( ) Add code so that when the time until the next rising edge is 0μs:
        // •Start the pulse by setting output pin 22 to 1.
    if (rising_edge <= 0) {     // Start the pulse
        cowpi_set_output_pins(1<<SERVO_PIN);
// Update your variable that tracks the time until the next rising edge, to the time
        // until the next pulse should start.
        rising_edge = SIGNAL_PERIOD_uS; //  signal period is how often the pulse is sent
// •Update your variable that tracks the time until the next falling edge, to the time
        // until this pulse should finish.
        falling_edge = pulse_width_us;

    }
    if (falling_edge <= 0) {    // End the pulse
        cowpi_set_output_pins(1<<SERVO_PIN);

    }

    rising_edge  += PULSE_INCREMENT_uS ;
    falling_edge += PULSE_INCREMENT_uS ;
}
