/**************************************************************************//**
 *
 * @file lock-controller.c
 *
 * @author (Jimmy Hua)
 * @author (Abdelrahman Elhaj)
 *
 * @brief Code to implement the "combination lock" mode.
 *
 ******************************************************************************/

/*
 * ComboLock GroupLab assignment and starter code (c) 2022-24 Christopher A. Bohn
 * ComboLock solution (c) the above-named students
 */

#include <CowPi.h>
#include "display.h"
#include "lock-controller.h"
#include "rotary-encoder.h"
#include "servomotor.h"

static uint8_t combination[3] __attribute__((section (".uninitialized_ram.")));
static uint8_t unlockcombo[3];  // Combo entered by user
static int lockstate = 0;       // 0 = LOCKED, 1 = UNLOCKED, 2 = ALARMED
static int comboposition = 0;   // Current entry position
static int numattempts = 0;     // Incorrect attempts counter    “bad try 1” or “bad try 2” 
//When the system is ALARMED, it shall display alert! and both LEDs shall blink on-and-off every quarter-second

uint8_t const *get_combination() {
    return combination;
}

void force_combination_reset() {
    combination[0] = 5;
    combination[1] = 10;
    combination[2] = 15;
}

void initialize_lock_controller() {
    lockstate = 0;  
    if (combination[0] > 15 || combination[1] > 15 || combination[2] > 15){
        force_combination_reset;
    }
    for (size_t i = 0; i < 3; i++){
        unlockcombo[i]= 00; 
    }
    ;
}

void control_lock() {



//  When the system is ALARMED, it shall display alert! and both LEDs shall blink on-and-off every quarter-second
    numattempts++;
    if (numattempts >= 3) {
        lockstate = 2; // ALARMED
        sprintf(buffer ,"alert!");
        while (lockstate == 2) { // Blink LEDs
            cowpi_illuminate_left_led();
            cowpi_illuminate_left_led();
            for (size_t i = 0; i < 250000; i++);   // busy wait 1/4 second
        }
        }
    ;
}
