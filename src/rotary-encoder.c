/**************************************************************************//**
 *
 * @file rotary-encoder.c
 *
 * @author (Jimmy Hua)
 * @author (Abdelrahman Elhaj)
 *
 * @brief Code to determine the direction that a rotary encoder is turning.
 *
 ******************************************************************************/

/*
 * ComboLock GroupLab assignment and starter code (c) 2022-24 Christopher A. Bohn
 * ComboLock solution (c) the above-named students
 */

#include <CowPi.h>
#include "interrupt_support.h"
#include "rotary-encoder.h"

#define A_WIPER_PIN         (16)
#define B_WIPER_PIN         (17)

typedef enum {
    HIGH_HIGH, HIGH_LOW, LOW_LOW, LOW_HIGH, UNKNOWN
} rotation_state_t;

// typedef enum {
//     counterclockwise, clockwise 
// } directions;

static rotation_state_t volatile state;
static direction_t volatile direction = STATIONARY;
static int volatile clockwise_count = 0;
static int volatile counterclockwise_count = 0;


static void handle_quadrature_interrupt();

void initialize_rotary_encoder() { 
    //Register interrupt service for pins A & B 
    cowpi_set_pullup_input_pins((1 << A_WIPER_PIN) | (1 << B_WIPER_PIN));
    register_pin_ISR((1 << A_WIPER_PIN) | (1 << B_WIPER_PIN), handle_quadrature_interrupt);
    state = UNKNOWN; //initial state set to unknown 

}

uint8_t get_quadrature() {
    cowpi_ioport_t volatile *ioport = (cowpi_ioport_t *) (0xD0000000);;    
    //set up pins stats read for both A & B 
    
    uint32_t a_signal = ((ioport->input) & (1 << A_WIPER_PIN));
    uint32_t b_signal = ((ioport->input) & (1 << B_WIPER_PIN));
    // uint8_t signala = cowpi_register_pin_ISR
    //Signal shift by 1 bit 
    uint32_t BA = (b_signal >> A_WIPER_PIN) & (a_signal >> A_WIPER_PIN);
    return BA;
    // return (b_signal << 1) | a_signal;
}

char *count_rotations(char *buffer) {
    //  construct rotation counts to a string
    sprintf(buffer, "CW:%d CCW:%d", clockwise_count, counterclockwise_count);
    //  buffer function returns count rotation 
    return buffer;
}

direction_t get_direction() {
    //store current direction into variable 
    direction_t current_direction = direction;
    //reset direction 
    direction = STATIONARY;
    return current_direction;
}

static void handle_quadrature_interrupt() {
    static rotation_state_t last_state = UNKNOWN;
    uint8_t quadrature = get_quadrature();
    //Recognizing clockwise indicators
    // switch(quadrature) {
    //     case 0b00: state = LOW_LOW; break;
    //     case 0b10: state = LOW_HIGH; break;
    //     case 0b01: state = HIGH_LOW; break;
    //     case 0b11: state = HIGH_HIGH; break;
    //     default  : state = UNKNOWN; //state case to be determined 
    // }
    //Recognizing clockwise indicator 
    if (last_state != UNKNOWN && state != UNKNOWN) {
        if (state == HIGH_HIGH){
            if (quadrature == 0b01){
                state = LOW_HIGH;
                last_state = HIGH_HIGH;
                direction = COUNTERCLOCKWISE; 
            }else if (quadrature == 0b10){
                state = HIGH_LOW;
                last_state = HIGH_HIGH;
                direction = CLOCKWISE; 
            }
        }else if(state == HIGH_LOW){
            if(quadrature == 0b11){
                state = HIGH_HIGH;
                last_state = HIGH_LOW;
                direction = COUNTERCLOCKWISE; 
            } else if(quadrature == 0b00){
                state = LOW_LOW;
                last_state = HIGH_LOW;
                direction = CLOCKWISE; 
                clockwise_count++;

            }
        }else if(state == LOW_LOW){
            if(quadrature == 0b01){
                state = LOW_HIGH;
                last_state = LOW_LOW;
                direction = CLOCKWISE; 
            } else if(quadrature == 0b10){
                state = HIGH_LOW;
                last_state = LOW_LOW;
                direction = COUNTERCLOCKWISE; 

            }
        }else if(state == LOW_HIGH){
            if(quadrature == 0b11){
                state = HIGH_HIGH;
                last_state = LOW_HIGH;
                direction = CLOCKWISE;
            } else if(quadrature == 0b00){
                state = LOW_LOW;
                last_state = LOW_HIGH;
                direction = COUNTERCLOCKWISE; 
                counterclockwise_count++;
            }
        }
    }
}
