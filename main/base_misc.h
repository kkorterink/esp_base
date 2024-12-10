// /home/kl/esp/projects/get_started/base/main/base_misc.h

#ifndef __BASE_MISC_H__
#define __BASE_MISC_H__

#include <stdint.h>
#include <stdbool.h>

#define SPACE       0x20

typedef void(*wait_cmd_f)(void *pContext);

char* read_chars(char *pSrcBuf, uint8_t *pRetval, uint8_t Num);
void ewait_add_timer(uint32_t TimeUnit10ms, wait_cmd_f WaitHdl, void *pContext);
void ewait_remove_timer(wait_cmd_f WaitHdl);
void ewait_clear_all_timers(void);
void ewait_timer_task(void);

#endif /* __BASE_MISC_H__ */
