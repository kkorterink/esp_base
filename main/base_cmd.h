// /home/kl/esp/projects/get_started/base/main/base_cmd.h

#ifndef __BASE_CMD_H__
#define __BASE_CMD_H__

#include "esp_netif_types.h"

#define T_10MS(t)       t       // For readability purposes only
#define MAX_AP          20

void wifi_scan_and_print(void);

void cmd_ip(char *pBuf);
void cmd_test_x(char *pBuf);
void cmd_test_y(char *pBuf);
void cmd_hostname(char *pBuf);
void cmd_wifi_start(char *pBuf);
void cmd_wifi_end(char *pBuf);
void cmd_wifi_reset(char *pBuf);
void cmd_wifi_scan(char *pBuf);

bool cmd_interpreter(char *pBuf);

#endif /* __BASE_CMD_H__ */


