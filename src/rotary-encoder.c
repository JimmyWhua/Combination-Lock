/**************************************************************************/ /**
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

#define A_WIPER_PIN (16)
#define B_WIPER_PIN (17)

typedef enum {
    LOW_LOW, LOW_HIGH , HIGH_LOW, HIGH_HIGH , UNKNOWN
} rotation_state_t;

static volatile rotation_state_t last_state = UNKNOWN;
static volatile direction_t direction = STATIONARY;
static volatile int clockwise_count = 0;
static volatile int counterclockwise_count = 0;

static void handle_quadrature_interrupt();

uint8_t get_quadrature() {
    cowpi_ioport_t volatile *ioport = (cowpi_ioport_t *)(0xD0000000);
    uint8_t BA = (uint8_t)((ioport->input >> A_WIPER_PIN) & 0x03);
    return BA;
    //uint32_t a_signal = ((ioport->input) & (1 << A_WIPER_PIN));
    //uint32_t b_signal = ((ioport->input) & (1 << B_WIPER_PIN));
}

void initialize_rotary_encoder() {
    uint8_t quadrature = get_quadrature();
    switch (quadrature) {
        case 0b00: last_state = LOW_LOW;   break;
        case 0b01: last_state = LOW_HIGH;  break;
        case 0b10: last_state = HIGH_LOW;  break;
        case 0b11: last_state = HIGH_HIGH; break;
        default:   last_state = UNKNOWN;   break;
    }

    direction = STATIONARY;
    clockwise_count = 0;
    counterclockwise_count = 0;

    cowpi_set_pullup_input_pins((1 << A_WIPER_PIN) | (1 << B_WIPER_PIN));
    register_pin_ISR((1 << A_WIPER_PIN) | (1 << B_WIPER_PIN), handle_quadrature_interrupt);
}

char *count_rotations(char *buffer) {
    sprintf(buffer, "CW: %d | CCW: %d", clockwise_count, counterclockwise_count);
    return buffer;
}

direction_t get_direction() {
    direction_t state = direction;
    direction = STATIONARY;
    return state;
}

static void handle_quadrature_interrupt() {
    uint8_t quadrature = get_quadrature();
    rotation_state_t state;

    switch (quadrature) {
        case 0b00: state = LOW_LOW;   break;
        case 0b01: state = LOW_HIGH;  break;
        case 0b10: state = HIGH_LOW;  break;
        case 0b11: state = HIGH_HIGH; break;
        default:   state = UNKNOWN;   break;
    }

    if (state == UNKNOWN || last_state == UNKNOWN) {
        last_state = state;
        return;
    }
    if (state == last_state) {
        return;
    }

    switch (last_state) {
        case HIGH_HIGH:
            if (state == HIGH_LOW && quadrature == 0b10) {
                direction = CLOCKWISE; 
            }
            else if (state == LOW_HIGH && quadrature == 0b01) {
                direction = COUNTERCLOCKWISE;
            } else {
                direction = STATIONARY;
            }
            break;

        case HIGH_LOW:
            if (state == LOW_LOW && quadrature == 0b00 && direction == CLOCKWISE) {
                clockwise_count++;
                direction = STATIONARY; 
            }
            break;

        case LOW_HIGH:
            if (state == LOW_LOW && quadrature == 0b00 && direction == COUNTERCLOCKWISE) {
                counterclockwise_count++;
                direction = STATIONARY; 
            }
            break;

        case LOW_LOW:
            direction = STATIONARY;
            break;

        default:
            direction = STATIONARY;
            break;
    }

    last_state = state;
}
