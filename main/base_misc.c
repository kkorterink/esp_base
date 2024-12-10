// /home/kl/esp/projects/get_started/base/main/base_misc.c

#include "base_misc.h"

#define MAX_EDELAYS     16

typedef struct {
    uint32_t    Time;
    void        *pContext;
    wait_cmd_f  WaitCb;
} Edelay_t;

static Edelay_t EDelay[MAX_EDELAYS];

/*-----------------------------------------------------------------------------
NAME:           ewait_timer_task
DESCRIPTION:    This is called every 10ms from RTOS free running timer
                Run through the list of all timers, if one has expired then check
                if a  callback has been registered for that timer and if so exe it
INPUTS:         none
OUTPUTS:        none
RETURN:         none
-----------------------------------------------------------------------------*/
void ewait_timer_task() {
    
    // temp function pointer, the purpose of this is to allow
    // the original pointer to be cleared so it can be re-used
    // when the temp function pointer is executed, i.e. the 
    // callback function can reload the same callback function
    void(*pcb)(void *pCtx);
    
    // update timers and execute callback if timer has expired
    for (uint32_t i = 0; i < MAX_EDELAYS; i++) {
        if (EDelay[i].Time == 0) {              // true then timer expired
            if (EDelay[i].WaitCb) {             // check if callback has been registered
                pcb = EDelay[i].WaitCb;         // copy callback to allow to be removed from timer list
                EDelay[i].WaitCb = 0;           // remove the timer from the list
                (*pcb)(EDelay[i].pContext);     // execute callback function
            }
        } else {
            EDelay[i].Time--;
        }
    }
}


/*-----------------------------------------------------------------------------
NAME:          ewait_add_timer
DESCRIPTION:   add timer
INPUTS:        TimeUnit10ms: time in units of 10ms
               WaitHdl: callback
               *pContext: pointer to context parameters
OUTPUTS:       none
-----------------------------------------------------------------------------*/
void ewait_add_timer(uint32_t TimeUnit10ms, wait_cmd_f WaitHdl, void *pContext) {
    uint8_t i;
    
    // if timer is already active then just reload timer value
    for (i = 0; i < MAX_EDELAYS; i++) {
        if (EDelay[i].WaitCb == WaitHdl) {
            EDelay[i].Time     = TimeUnit10ms;
            EDelay[i].pContext = pContext;
            return;            
        }
    }

    // find empty entry EDelay[] and add timer
    for (i = 0; i < MAX_EDELAYS; i++) {
        if (EDelay[i].WaitCb == 0) {
            EDelay[i].WaitCb   = WaitHdl;
            EDelay[i].Time     = TimeUnit10ms;
            EDelay[i].pContext = pContext;
            break;
        }
    }
}

/*-----------------------------------------------------------------------------
NAME:           ewait_remove_timer
DESCRIPTION:    
INPUTS:         none
OUTPUTS:        none
-----------------------------------------------------------------------------*/
void ewait_remove_timer(wait_cmd_f WaitHdl) {
    
    // remove  wait_cmd_cb from list
    for (uint8_t i = 0; i < MAX_EDELAYS; i++) {
        if (EDelay[i].WaitCb == WaitHdl) {
            EDelay[i].WaitCb   = 0;
            EDelay[i].Time     = 0;
            EDelay[i].pContext = 0;
            return;            
        }
    }
}



// /*-----------------------------------------------------------------------------
// NAME:           ewait_timer_busy
// DESCRIPTION:    Check if specific system timer is in use. If the timer list is 
//                 loaded with the required callback then return TRUE
// INPUTS:         timer callback
// OUTPUTS:        timer busy: TRUE else FALSE
// -----------------------------------------------------------------------------*/
// Bool ewait_timer_busy(wait_cmd_f WaitHdl) {
//     U8 i;
    
//     // check if wait_cmd_cb is in list
//     for (i = 0; i < MAX_EDELAYS; i++) {
//         if (EDelay[i].WaitCb == WaitHdl) {
//             return(TRUE);
//         }
//     }

//     return(FALSE);
// }

// /*-----------------------------------------------------------------------------
// NAME:           ewait_timer_expired
// DESCRIPTION:    Check if specific system timer is in use. If the timer list is 
//                 loaded with the required callback then return TRUE
// INPUTS:         timer callback
// OUTPUTS:        timer expired: TRUE else FALSE
// -----------------------------------------------------------------------------*/
// Bool ewait_timer_expired(wait_cmd_f WaitHdl) {
//     U8 i;
    
//     // check if wait_cmd_cb is in list
//     for (i = 0; i < MAX_EDELAYS; i++) {
//         if (EDelay[i].WaitCb == WaitHdl) {
//             return(FALSE);
//         }
//     }

//     return(TRUE);
// }


/*-----------------------------------------------------------------------------
NAME:           ewait_clear_all_timers
DESCRIPTION:    clear timer list
INPUTS:         none
OUTPUTS:        none
-----------------------------------------------------------------------------*/
void ewait_clear_all_timers() {
    for (uint32_t i = 0; i < MAX_EDELAYS; i++) {
        EDelay[i].WaitCb   = 0;
        EDelay[i].Time     = 0;
        EDelay[i].pContext = 0;
    }
}

/*-----------------------------------------------------------------------------
NAME:           read_chars
DESCRIPTION:    read 1, 2 or 3 ASCII numbers from pSrcBuf and convert to bin value
                leading spaces and '=' sign are ignored
INPUTS:         retval: pointer to bin return value
                num:    1, 2, or 3 number of chars to convert
RETURN:         Error: 0
                OK: updated pouinter to *pSrcBuf
-----------------------------------------------------------------------------*/
char* read_chars(char *pSrcBuf, uint8_t *pRetval, uint8_t Num) {
    char c;

    *pRetval = 0;
    do {
        c = *pSrcBuf++;
        if (!c) {
            return(0);
        }
    } while(c == SPACE || c == '=');

    if ( (c < '0') || (c > '9') ) {
        return(0);
    }

    *pRetval = c & 0x0f;                     // units
    while (--Num) {
        c = *pSrcBuf++;
        if (!c) {
            return (0);
        }

        if ( (c < '0') || (c > '9') ) {
            return (0);
        }

        *pRetval = (*pRetval * 10) + (c & 0x0f);
    }

    return(pSrcBuf);
}


