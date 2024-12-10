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
static int comboposition[3];   // Current entry position
static int numattempts;     // Incorrect attempts counter    “bad try 1” or “bad try 2” 
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
    clears();
    lockstate = 0;
    char buff[22];
    // sprintf(buff, "BA : %d",BA);
    // display_string(5,buff);
    if(unlockcombo[0] == 0 && unlockcombo[1] == 0 && unlockcombo[2] == 0){  //  Req 8 
        sprintf(buff, "UnloComb:   -  -  " );
        display_string(2,buff);
    }else if(combination[0] > 0 && combination[1] == 0 && combination[2] == 0 ){
        sprintf(buff, "UnloComb: %02d-  -  ", unlockcombo[0]);
        display_string(2,buff);
    }else if(combination[0] > 0 && combination[1] > 0 && combination[2] == 0 ){
        sprintf(buff, "UnloComb: %02d-%02d-  ", unlockcombo[0], unlockcombo[1]);
        display_string(2,buff);
    }else if(combination[0] > 0 && combination[1] > 0 && combination[2] > 0 ){
        sprintf(buff, "UnloComb: %02d-%02d-%02d", unlockcombo[0], unlockcombo[1], unlockcombo[2]);
        display_string(2,buff);
    }

    sprintf(buff, "Key: %02d-%02d-%02d", combination[0], combination[1], combination[2]);
    display_string(1,buff);

    if (combination[0] > 15 || combination[1] > 15 || combination[2] > 15 ){
        // force_combination_reset();
    }
    ;
}
//  make method to clear all entrys
void clears(){
    numattempts = 0;
    for (size_t i = 0; i < 3; i++){
        unlockcombo[i] = 00;
        comboposition[i] = 00;
    }
    
}
void control_lock() {
    char buff[22];
    // sprintf(buff, "BA : %d",BA);
    // display_string(5,buff);

    direction_t direction = get_direction();
    if (lockstate == 0){ // Start of Requirement 8 or 9
    //  Req 9. When system is LOCKED - left LED is on - the right LED is off
        cowpi_illuminate_left_led();
        cowpi_deluminate_right_led();
        rotate_full_clockwise();    //Req 10
        // Req 11
        
        

        /* code */
    }
//  When the system is ALARMED, it shall display alert! and both LEDs shall blink on-and-off every quarter-second
    numattempts++;
    if (numattempts >= 3) {     // Requirement 19
        lockstate = 2; // ALARMED
        sprintf(buff ,"alert!");
        display_string(5,buff);
        
        while (lockstate == 2) { // Blink LEDs while locked --> needs to be reset to continue
            cowpi_illuminate_left_led();
            cowpi_illuminate_right_led();
            for (size_t i = 0; i < 250000; i++);   // busy wait 1/4 second is 250k us
            cowpi_deluminate_left_led();
            cowpi_deluminate_right_led();
            for (size_t i = 0; i < 250000; i++);   // busy wait 1/4 second is 250k us
        }
    }else {     // Requirement 18
        lockstate = 0;
        sprintf(buff, "bad try %d", numattempts);
        display_string(5,buff);
        for (size_t i = 0; i < 2; i++){     //  both LEDs shall blink twice
            cowpi_illuminate_left_led();
            cowpi_illuminate_right_led();
            for (size_t i = 0; i < 250000; i++);   // busy wait 1/4 second is 250k us
            cowpi_deluminate_left_led();
            cowpi_deluminate_right_led();
            for (size_t i = 0; i < 250000; i++);   // busy wait 1/4 second is 250k us
        }

    }

}

//make another function for req 12
