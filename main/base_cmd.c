
// /home/kl/esp/projects/get_started/base/main/base_cmd.c

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "base_misc.h"
#include "base_cmd.h"

// #include "esp_event.h"
// #include "esp_app_trace.h"

// static const char   *TAG = "base_cmd";

typedef void (*cmd_handler)(char *pBuf);

typedef struct led_flash_times {
    uint32_t    led_time_on;
    uint32_t    led_time_off;
    bool        led_on;
} led_flash_times_t;




// callback for greenled_flash() function, called from timer task
void greenled_flash_cb(void *pContext) {
    led_flash_times_t *led_times = (led_flash_times_t*)pContext;

    if (led_times->led_on == true) {
        
        // we get here when the on-time has expired, so led must be switched off
        gpio_set_level(GPIO_NUM_4, 1);    // GPIO_NUM_4 is green led, off is 1
        led_times->led_on = false;
        
        // led_time_off specifies the time it should be switched off
        ewait_add_timer(led_times->led_time_off,  greenled_flash_cb, (void*)led_times);
    } else {

        // we get here when the off-time has expired, so led must be switched on
        gpio_set_level(GPIO_NUM_4, 0);    // GPIO_NUM_4 is green led, on is 0
        led_times->led_on = true;

        // led_time_on specifies the time it should be switched on
        ewait_add_timer(led_times->led_time_on,  greenled_flash_cb, (void*)led_times);
    }
}


// Flash the green led, time_on and time_off in units of 10ms
void greenled_flash(uint32_t time_on, uint32_t time_off) {
    static led_flash_times_t led_flash;

    led_flash.led_time_on = T_10MS(time_on);
    led_flash.led_time_off = T_10MS(time_off);
    led_flash.led_on = false;
    if (time_on == 0 || time_off == 0) {
        ewait_remove_timer(greenled_flash_cb);
        gpio_set_level(GPIO_NUM_4, 1);    // GPIO_NUM_4 is green led, off is 1
    } else {
        
        // switch the led on and start the timer for the period it should be on
        gpio_set_level(GPIO_NUM_4, 0);    // GPIO_NUM_4 is green led, on is 0
        ewait_add_timer(time_on, greenled_flash_cb, (void*)&led_flash);
    }
}




// lt, list tasks
void cmd_list_tasks(char *pBuf) {
    TaskStatus_t *task_list = NULL;
    UBaseType_t task_list_size;
    uint32_t task_run_time;

    // allocate memory
    task_list_size = uxTaskGetNumberOfTasks();
    task_list = malloc(sizeof(TaskStatus_t) * task_list_size);
    if (task_list == NULL) {
        printf("err ESP_ERR_NO_MEM\r\n");
        return;
    }

    printf("tasks running: %d\r\n", task_list_size);

    // get task states
    // CONFIG_FREERTOS_USE_TRACE_FACILITY must be enabled!
    // This to allow configUSE_TRACE_FACILITY to be defined in FreeRTOSConfig.h
    // configUSE_TRACE_FACILITY is needed by RTOS to include uxTaskGetSystemState()
    task_list_size = uxTaskGetSystemState(task_list, task_list_size, &task_run_time);
    if (task_list_size == 0) {
        printf("err ESP_ERR_INVALID_SIZE\r\n");
        return;
    }

    // usStackHighWaterMark: The minimum amount of stack space that has
    // remained for the task since the task was created.  The closer this 
    // value is to zero the closer the task has come to overflowing its stack.
    char *names[] = {"running", "ready", "blocked", "suspended", "deleted", "invalid"};
    printf("        name  prior stack      state\r\n");
    printf("------------------------------------\r\n");
    for (uint32_t i = 0; i < task_list_size; i++) {
        int k = task_list[i].eCurrentState;

        // uint k = task_list[i].eCurrentState;
        printf("%12s%5d%8d%11s\r\n", task_list[i].pcTaskName, 
                                     task_list[i].uxCurrentPriority,
                                     (int)task_list[i].usStackHighWaterMark,
                                     names[k]);
    }
}


// a, version prompt
void cmd_prompt(char *pBuf) {
    printf("IDF Version: %s\r\n", esp_get_idf_version());
}


// restart, reboot device
void cmd_restart(char *pBuf) {
    printf("rebooting...\r\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    fflush(stdout);
    esp_restart();
}


void wifi_scan_and_print() {
    // configure and run the scan process in blocking mode
    wifi_scan_config_t scan_config = {
        .ssid = 0,
        .bssid = 0,
        .channel = 0,
        .show_hidden = true
    };

    printf("Start scanning...");
    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));
    printf(" completed!\r\n");

    // get the list of APs found in the last scan
    uint16_t ap_num = MAX_AP;
    wifi_ap_record_t ap_records[MAX_AP];
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_num, ap_records));

    // print the list 
    char *names[] = {"OPEN", "WEP", "WPA PSK", "WPA2 PSK", "WPA WPA2 PSK", 
                     "WPA2 ENTERPRISE", "WPA3 PSK", "WPA2 WPA3 PSK", "WAPI_PSK", "MAX"};
    printf("Found %d access points:\r\n", ap_num);
    // printf("               SSID              | Channel | RSSI |   Auth Mode \r\n");
    printf("BSSID                              SSID              | Channel | RSSI |   Auth Mode \r\n");
    printf("------------------------------------------------------------------------------------\r\n");
    for(int i = 0; i < ap_num; i++) {
        int k = ap_records[i].authmode;
        printf("%02X:%02X:%02X:%02X:%02X:%02X   %32s | %7d | %4d | %12s\n", 
            ap_records[i].bssid[0], ap_records[i].bssid[1], ap_records[i].bssid[2],
            ap_records[i].bssid[3], ap_records[i].bssid[4], ap_records[i].bssid[5],
            (char *)ap_records[i].ssid, ap_records[i].primary, ap_records[i].rssi, names[k]);
    }

    printf("\r\n");
}


// callback for cmd_greenled_on()
void cmd_greenled_on_cb(void *pContext) {
    gpio_set_level(GPIO_NUM_4, 0);  // led on
}

// gon=tt, switch green test LED on for time tt[10ms]
void cmd_greenled_on(char *pBufT10ms) {
    uint8_t reg;

    if (0 == read_chars(pBufT10ms, &reg, 2) ) {    // read 2 bytes
        goto err;
    }

    if (reg == 0)  {
        goto err;
    }

    gpio_set_level(GPIO_NUM_4, 1);  // led off
    ewait_add_timer(reg, cmd_greenled_on_cb, 0);
    printf("ok\r\n");
    return;
err:
    printf("err, format: gon=<tt> where <tt> != 0, \r\n");
}


// callback for cmd_greenled_flash()
// This is called every time a led ON or OFF timer expires,
// then from the callback a new timer is created
void cmd_greenled_flash_cb(void *pContext) {
    led_flash_times_t* ft = (led_flash_times_t*)pContext;

    // update either the on or off state and create new timer
    if (ft->led_on == false) {
        gpio_set_level(GPIO_NUM_4, 0);  // led on
        ft->led_on = true;
        ewait_add_timer(ft->led_time_on, cmd_greenled_flash_cb, (void*)ft);
    } else {
        gpio_set_level(GPIO_NUM_4, 1);  // led off
        ft->led_on = false;
        ewait_add_timer(ft->led_time_off, cmd_greenled_flash_cb, (void*)ft);
    }
}


// gflash=tt tt, flash the green test LED for on off time tt[10ms]
void cmd_greenled_flash(char *pBufT10ms) {
    uint8_t reg;
    char*   preg;
    static  led_flash_times_t ft;

    // get the ON timr bytes
    preg = read_chars(pBufT10ms, &reg, 2);     // rean ON time, 2 bytes
    if (preg == 0) {
        goto err;
    }
    
    if (reg == 0)  {
        if (ft.led_time_on) {
            ewait_remove_timer(cmd_greenled_flash_cb);
            printf("flash timers stopped\r\n");
            return;
        } else {
            goto err;
        }
    }
    ft.led_time_on = reg;

    // get the OFF time buyes
    if (0 == read_chars(preg, &reg, 2)) {      // read OFF time, 2 bytes
        goto err;
    }

    if (reg == 0)  {
        goto err;
    }

    ft.led_time_off = reg;
    ft.led_on = false;
    gpio_set_level(GPIO_NUM_4, 1);  // led off
    ewait_add_timer(ft.led_time_off, cmd_greenled_flash_cb, (void*)&ft);
    printf("ok\r\n");
    return;
err:
    printf("err, use: gflash=<t1 t2> on off, unit: 10ms, <t1>=00 to stop\r\n");
}





// char cmd_select(const char *pComName, char *pBuf, uint8_t Len, void (*cmd_handler)(char *pBuf)) {
char cmd_select(const char *pComName, char *pBuf, uint8_t Len, cmd_handler CmdHdl) {
    char    cmd, c1, c2;
    uint8_t i;

    cmd = true;
    for (i = 0; i < Len; i++) {
        c1 = pComName[i];
        c2 = pBuf[i];
        if (!c1 || !c2 ) {
            cmd = false;            // empty string, nothing to do
            break;
        }

        if (c1 != c2) {             // compare stored cmd chars with received cmd chars
            cmd = false;
        }
    }

    if (cmd == true) {

        // only accept as valid command if cmd==true and next char is one of following:
        // 0
        // '='
        // SPACE
        // between '0'-'9'
        if ( !pBuf[i] || (pBuf[i] == '=') || (pBuf[i] == SPACE)
                                    || (pBuf[i] >= '0' && pBuf[i] <= '9') ) {

            CmdHdl(&pBuf[i]);               // exe command, use typedef
            return(true);
        }

    }

    return(false);
}


// h, print command syntax
void cmd_help(char *pBuf) {
    printf("gflash=<tt tt>: flash green led for <t1 t2> (<t1>=on <t2>=off, units 10ms, <t1=00> stop)\r\n");
    printf("gon=<tt>:       green led on for <tt> (tt units of 10ms)\r\n");
    printf("a:              idf version\r\n");
    printf("x:              x-test function\r\n");
    printf("y:              y-test function\r\n");
    printf("lt:             list tasks running\r\n");
    printf("ip:             if wifi running return ip, gw and mask\r\n");
    printf("hname:          if wifi running return hostname\r\n");
    printf("wscan:          wifi scan\r\n");
    printf("wstart:         wifi start\r\n");
    printf("wend:           wifi end\r\n");
    printf("wreset:         does commands wend and wstart\r\n");
    printf("restart:        boot from start\r\n");
}


// command interpreter with command callback functions
bool cmd_interpreter(char *pBuf) {
    if ( cmd_select("gflash",   pBuf, 6, cmd_greenled_flash) )  return(true);
    if ( cmd_select("gon",      pBuf, 3, cmd_greenled_on) )     return(true);
    if ( cmd_select("h",        pBuf, 1, cmd_help) )            return(true);
    if ( cmd_select("a",        pBuf, 1, cmd_prompt) )          return(true);
    if ( cmd_select("x",        pBuf, 1, cmd_test_x) )          return(true);
    if ( cmd_select("y",        pBuf, 1, cmd_test_y) )          return(true);
    if ( cmd_select("lt",       pBuf, 2, cmd_list_tasks) )      return(true);
    if ( cmd_select("ip",       pBuf, 2, cmd_ip) )              return(true);
    if ( cmd_select("hname",    pBuf, 5, cmd_hostname) )        return(true);
    if ( cmd_select("wstart",   pBuf, 6, cmd_wifi_start) )      return(true);
    if ( cmd_select("wscan",    pBuf, 5, cmd_wifi_scan) )       return(true);
    if ( cmd_select("wend",     pBuf, 4, cmd_wifi_end) )        return(true);
    if ( cmd_select("wreset",   pBuf, 6, cmd_wifi_reset) )      return(true);
    if ( cmd_select("restart",  pBuf, 7, cmd_restart) )         return(true);
   return(false);  // false: did not find valid command
}






/*
    On the target side, the special vprintf-like function esp_apptrace_vprintf needs to be installed. 
    It sends log data to the host. Example code is provided in system/app_trace_to_host.

    Follow instructions in items 2-5 in Application Specific Tracing.
    https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/app_trace.html#application-specific-tracing
    
    To print out collected log records, run the following command in terminal: 
    $IDF_PATH/tools/esp_app_trace/logtrace_proc.py /path/to/trace/file /path/to/program/elf/file.
*/

/*
    // Route LOGx() to the host
    esp_log_set_vprintf(esp_apptrace_vprintf);  // output to JTAG
    int samples_collected = adc1_sample_and_show(TEST_SAMPLING_PERIOD);
    
    // Route LOGx() back to UART
    esp_log_set_vprintf(vprintf);               // output to UART
    
    // Flush collected data to the host
    esp_apptrace_flush(ESP_APPTRACE_DEST_TRAX, 100000);
    ESP_LOGI(TAG, "Collected %d samples in %d ms.\n", samples_collected, TEST_SAMPLING_PERIOD);
*/






