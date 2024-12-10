
// /home/kl/esp/projects/get_started/base/main/base_main.c

#include <stdio.h>
#include <string.h>

#include <sys/param.h>
#include <stdlib.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
// #include "freertos/mpu_wrappers.h"
// #include "freertos/timers.h"
// #include "freertos/semphr.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_log_buffer.h"
// /home/kl/esp/esp-idf/components/log/include/esp_log_buffer.h

#include "spi_flash_mmap.h"

#include "esp_wifi.h"
#include "esp_event.h"

#include "nvs_flash.h"
#include "sdkconfig.h"

#include "esp_http_client.h"

#include "base_cmd.h"
#include "base_misc.h"

// build_system/cmake/idf_as_lib/stubs/spi_flash/

#define RX_BUF_LEN          64
#define CR                  0x0d
#define LF                  0x0a
#define BUF_SIZE            (1024)

#define BIT_TIMER_10MS              BIT0    // trigger flag for the 10ms wait timer expired
// #define BIT_WIFI_EVENT_STA_START    BIT1    // WIFI_EVENT_STA_START
// #define BIT_WIFI_EVENT_SCAN_DONE    BIT2    // WIFI_EVENT_SCAN_DONE
#define BIT_TEST                    BIT3    // test only

#define STA_RETRY_CONNECT_MAX       4       // station connect retries
#define STA_REQUEST_DISCONNECT      0xff    // command to disconnect

// #define MAX_HTTP_RECV_BUFFER        512
#define MAX_HTTP_OUTPUT_BUFFER      2048


static int              retry_connect = 0;
static unsigned int     tmp_cnt = 0;
static unsigned int     cnt_1 = 0;
static unsigned int     cnt_2 = 0;

static const char   *TAG = "base_main";

// static const char   *ssid = "Youngs HH - admin";     // Beaufort West, Youngs Rooms
// static const char   *password = "HalfwayThere!";     // Beaufort West, Youngs Rooms
// static const char   *ssid = "HomeMark";          // Maria, Bloubergstrant
// static const char   *password = "Spider10";      // Maria, Bloubergstrant
static const char   *ssid = "Klaas Fibre";          // Max 32 chars (see wifi_config_t)
static const char   *password = "xxyd98942";        // Max 64 chars (see wifi_config_t)
static const char   *hostname = "klk_esp32";            // hostname
static const char   *http_endpoint = "httpbin.org";     // host connection, for now


static esp_netif_t                  *my_netif = NULL;
static wifi_config_t                wifi_config = { 0 };
static QueueHandle_t                uart0_queue;
static TaskHandle_t                 run_task_hdl = NULL;
static TimerHandle_t                led_timer_hdl = NULL;
static esp_event_handler_instance_t instance_any_id = NULL;

// flags used to determine the state of the wifi connection
typedef struct {
    uint8_t esp_wifi_init:1;
    uint8_t esp_netif_init:1;
    uint8_t esp_event_loop_created:1;
    uint8_t default_wifi_sta_created:1;
    uint8_t wifi_started:1;
    uint8_t wifi_connected:1;
    uint8_t wifi_got_ip:1;
    uint8_t b7:1;
} wifi_flags_t;

// types use to determine which wifi action to exe
typedef enum {
    WIFI_PHASE_IDLE,
    WIFI_PHASE_RESET_BEGIN,
    WIFI_PHASE_WAIT_TO_STOP,
    WIFI_PHASE_INIT,
    WIFI_PHASE_CONNECT,
    WIFI_PHASE_DISCONNECT,
    WIFI_PHASE_IP_CHANGE,
    WIFI_PHASE_DEINIT
} wifi_state_t ; 

static wifi_flags_t wifi_flag = { 0 }; 
static wifi_state_t wifi_state = WIFI_PHASE_IDLE;


// what is this?
// esp_netif_get_desc(event->netif)
// static void tx_rx_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
//     ip_event_tx_rx_t *event = (ip_event_tx_rx_t *)event_data;

//     if (event->dir == ESP_NETIF_TX) {
//         ESP_LOGI(TAG, "Got TX event: Interface \"%s\" data len: %d", 
//                                         esp_netif_get_desc(event->netif), event->len);
//     } else if (event->dir == ESP_NETIF_RX) {
//         ESP_LOGI(TAG, "Got RX event: Interface \"%s\" data len: %d", 
//                                         esp_netif_get_desc(event->netif), event->len);
//     } else {
//         ESP_LOGI(TAG, "Got Unknown event: Interface \"%s\"", );
//     }
// }

// esp_event_handler_register(IP_EVENT, IP_EVENT_TX_RX, &tx_rx_event_handler, NULL);


/* 
create                                  destroy                                     task        remark
-----------------------------------------------------------------------------------------------------
esp_wifi_init(&w)                       esp_wifi_deinit()                           "wifi"
esp_netif_init()                        (not possible)                              "tiT"       only once
esp_event_loop_create_default()         esp_event_loop_delete_default()             "sys_evt"
esp_event_handler_instance_register(..) esp_event_handler_instance_unregister(..)   -
esp_netif_create_default_wifi_sta()     esp_netif_destroy_default_wifi(..)          -

esp_netif_new(..)                       esp_netif_destroy(..)                       -
esp_wifi_start()                        esp_wifi_stop()                             -
esp_wifi_connect()                      esp_wifi_disconnect()                       -           WIFI_EVENT_STA_CONNECTED


event                           source of event
-------------------------------------------------------------------------------
WIFI_EVENT_STA_START            esp_wifi_start()
WIFI_EVENT_STA_STOP             esp_wifi_stop()
WIFI_EVENT_STA_CONNECTED        esp_wifi_connect()
WIFI_EVENT_STA_DISCONNECTED     - esp_wifi_connect() fails
                                - esp_wifi_disconnect()
                                - esp_wifi_stop()
                                - Wi-Fi connection is disrupted

wifi_config_t wifi_config = {
    .sta = {
        .ssid = EXAMPLE_ESP_WIFI_SSID,
        .password = EXAMPLE_ESP_WIFI_PASS,
            * ooh..
            * Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
            * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
            * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
            * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
        .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
        .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
    },
};
*/


    // isprint

    // struct esp_netif_config {
        // const esp_netif_inherent_config_t *base;
        // const esp_netif_driver_ifconfig_t *driver;
        // const esp_netif_netstack_config_t *stack;
    // };
    
    // esp_err_t esp_netif_get_hostname(esp_netif_t *esp_netif, const char **hostname)
    // esp_err_t esp_netif_set_hostname(esp_netif_t *esp_netif, const char *hostname)

    // double pointers
    // https://stackoverflow.com/questions/64171902/length-of-string-using-double-pointers-in-c

    // void find_length(char **k) {
    //     char *kk = *k; // temp.      // k is pointer to pointer to char
    //     while (*kk != '\0') {        // *k is a pointer to char
    //         printf("%c", *kk);       // **k is a char
    //         kk++;
    //     }    
    // }

    // ESP_LOGD(TAG, "%s esp_netif:%p", __func__, esp_netif);


esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
    static char *output_buffer = NULL;  // Buffer to store response of http request from event handler
    static int output_len = 0;          // Stores number of bytes read

    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", 
                                        evt->header_key, evt->header_value);
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER");
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            
            // Clean the buffer in case of a new request
            if (output_len == 0 && evt->user_data) {

                // we are just starting to copy the output data into the use
                memset(evt->user_data, 0, MAX_HTTP_OUTPUT_BUFFER);
            }
            // Check for chunked encoding is added as the URL for chunked 
            // encoding used in this example returns binary data.
            // However, event handler can also be used in case chunked encoding is used.
            if (!esp_http_client_is_chunked_response(evt->client)) {
                
                // If user_data buffer is configured, copy the response into the buffer
                int copy_len = 0;
                if (evt->user_data) {
                    
                    // The last byte in evt->user_data is kept for the NULL character 
                    // in case of out-of-bound access.
                    copy_len = MIN(evt->data_len, (MAX_HTTP_OUTPUT_BUFFER - output_len));
                    if (copy_len) {
                        memcpy(evt->user_data + output_len, evt->data, copy_len);
                    }
                } else {
                    int content_len = esp_http_client_get_content_length(evt->client);
                    if (output_buffer == NULL) {
                        
                        // We initialize output_buffer with 0 because it is used by strlen() 
                        // and similar functions therefore should be null terminated.
                        output_buffer = (char *) calloc(content_len + 1, sizeof(char));
                        output_len = 0;
                        if (output_buffer == NULL) {
                            ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                            return ESP_FAIL;
                        }
                    }
                    copy_len = MIN(evt->data_len, (content_len - output_len));
                    if (copy_len) {
                        memcpy(output_buffer + output_len, evt->data, copy_len);
                    }
                }
                output_len += copy_len;
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
            break;
        default:
            ESP_LOGI(TAG, "err http client event, id=%d line=%d", 
                                                (int)evt->event_id, __LINE__);
        }

    return ESP_OK;
}


// y, test function
void cmd_test_y(char *pBuf) {

    // Declare local_response_buffer with size (MAX_HTTP_OUTPUT_BUFFER + 1) to 
    // prevent out of bound access when it is used by functions like strlen(). 
    // The buffer should only be used upto size MAX_HTTP_OUTPUT_BUFFER
    char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER + 1] = {0};
    /**
     * NOTE: All the configuration parameters for http_client must be specified 
     * either in URL or as host and path parameters.
     * If host and path parameters are not set, query parameter will be ignored. 
     * In such cases, query parameter should be specified in URL.
     * If URL as well as host and path parameters are specified, values of host 
     * and path will be considered.
     */
    
    // CONFIG_EXAMPLE_HTTP_ENDPOINT="httpbin.org"
    // static const char   *http_endpoint = "httpbin.org";


    esp_http_client_config_t config = {
        .host = http_endpoint,                  // "httpbin.org"
        .path = "/get",
        .query = "esp",
        .event_handler = _http_event_handler,
        .user_data = local_response_buffer,     // Pass address of local buffer to the 'get' response
        .disable_auto_redirect = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    // GET
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %"PRId64,
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    }


    ESP_LOG_BUFFER_HEX(TAG, local_response_buffer, strlen(local_response_buffer));
    // todo test below
    // ESP_LOG_BUFFER_HEXDUMP(TAG, local_response_buffer, strlen(local_response_buffer), ESP_LOG_INFO);

    esp_http_client_cleanup(client);
    printf("ok y-cmd\r\n");
}

// x, test function
void cmd_test_x(char *pBuf) {
    // printf("cnt_1=%d, cnt_2=%d\r\n", cnt_1, cnt_2);
    printf("x-cmd\r\n");
}


void cmd_hostname(char *pBuf) {
    if (my_netif != NULL) {
        const char *hname = NULL;
        esp_netif_get_hostname(my_netif, &hname);   // note: bouble pointer!
        printf("hostname: %s\n\r", hname);
    } else {
        printf("err hostname, no wifi interface\n\r");
    }
}


// wscan, wifi scan
void cmd_wifi_scan(char *pBuf) {

    // network up then need to disconnect before doing wifi scan
    if (true == esp_netif_is_netif_up(my_netif)) {
        // network up and running then need to disconnect before calling wifi scan
        retry_connect = STA_REQUEST_DISCONNECT;
        ESP_ERROR_CHECK(esp_wifi_disconnect()); // triggers WIFI_EVENT_STA_DISCONNECTED
        wifi_scan_and_print();
        retry_connect = 0;
        // triggers WIFI_EVENT_STA_CONNECTED or WIFI_EVENT_STA_DISCONNECTED if connection fails
        ESP_ERROR_CHECK(esp_wifi_connect());
    } else {
        // network not up or running
        wifi_init_config_t w = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&w));                 // wifi task, buffers etc
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));  // STA or AP
        ESP_ERROR_CHECK(esp_wifi_start());                  // start wifi according mode
        wifi_scan_and_print();
        ESP_ERROR_CHECK(esp_wifi_stop());
        ESP_ERROR_CHECK(esp_wifi_deinit());
    }
}


// ip, list the ip, gw and mask address
void cmd_ip(char *pBuf) {
    esp_netif_ip_info_t ip;
    uint8_t mac[6] = { 0 };

    if (esp_netif_is_netif_up(my_netif)) {
        ESP_ERROR_CHECK(esp_netif_get_mac(my_netif, &mac[0]));
        esp_netif_get_ip_info(my_netif, &ip);
        printf("ip="IPSTR", gw="IPSTR", mask="IPSTR", mac=%02X:%02X:%02X:%02X:%02X:%02X\n\r", 
                    IP2STR(&ip.ip), IP2STR(&ip.gw), IP2STR(&ip.netmask), 
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else {
        printf("err, wifi interface down\n\r");
    }
}

// printf("ip="IPSTR", gw="IPSTR", mask="IPSTR"\n\r", 
//     IP2STR(&ip.ip), IP2STR(&ip.gw), IP2STR(&ip.netmask));



// wstart, wifi start
void cmd_wifi_start(char *pBuf) {
    wifi_state = WIFI_PHASE_INIT;
}

// we, wifi end
void cmd_wifi_end(char *pBuf) {
    wifi_state = WIFI_PHASE_DEINIT;
}

// RTOS timer callback
void rtos_timer_cb(TimerHandle_t t) {
    
    // notify the run task, timer controlled from run_task
    // notification to receive: ulTaskNotifyTake()
    xTaskNotify(run_task_hdl, BIT_TIMER_10MS, eSetBits);
}

// callback for wifi reset command "wreset"
void cmd_wifi_reset_cb(void* pContext) {
    if (wifi_flag.esp_wifi_init == false) {
        wifi_state = WIFI_PHASE_INIT;
    } else {
        printf("err, cannot restart wifi, already running\r\n");
    }
}

// if wifi running then stop and deinit, wait 1000ms, init and start the wifi
// command: "wreset"
void cmd_wifi_reset(char *pBuf) {
    if (wifi_state == WIFI_PHASE_IDLE) {
        if (wifi_flag.esp_wifi_init == true) {
            wifi_state = WIFI_PHASE_DEINIT;
            ewait_add_timer(T_10MS(100), cmd_wifi_reset_cb, NULL);  // 1000ms delay
        } else {
            printf("err, wifi not running\r\n");
        }
    } else {
        printf("err, wifi not in idle state\r\n");
    }
}


static void event_handler_any(void* arg, esp_event_base_t ev_base, int32_t ev_id, void* ev_data) {
    int line = 0;
    // WIFI_EVENT
    if (ev_base == WIFI_EVENT) {
        switch (ev_id) {
        case WIFI_EVENT_STA_START:      // source: esp_wifi_start()
            ESP_LOGI(TAG, "ev wifi started...");

            ESP_ERROR_CHECK(esp_wifi_connect());

            wifi_flag.wifi_started = true;
            // wifi_state = WIFI_PHASE_CONNECT;
            break;
        
        case WIFI_EVENT_STA_STOP:       // source: esp_wifi_stop()
            ESP_LOGI(TAG, "ev wifi stopped..");
            break;

        case WIFI_EVENT_STA_CONNECTED:  // source: esp_wifi_connect()
            // event when: esp_wifi_connect() connected to AP
            // If application depends on lwIP then have to wait for
            // IP addr, this happens when IP_EVENT_STA_GOT_IP received
            wifi_flag.wifi_connected = true;
            ESP_LOGI(TAG, "ev station connected..");
            break;

        case WIFI_EVENT_STA_DISCONNECTED:       // source: ??

            // retry_connect value      action
            // -----------------------------------------------
            // STA_RETRY_CONNECT_MAX    station try to connect
            // STA_REQUEST_DISCONNECT   request disconnect

            if (retry_connect < STA_RETRY_CONNECT_MAX) {
                esp_wifi_connect();
                retry_connect++;
                ESP_LOGI(TAG, "ev retry to connect.. (%d)", retry_connect);
            } else {
                wifi_flag.wifi_got_ip = false;
                if (retry_connect == STA_REQUEST_DISCONNECT) {
                    ESP_LOGI(TAG, "ev STA disconnected as requested");
                } else {
                    ESP_LOGI(TAG, "err ev STA disconnected..");
                }
            }

            break;

        // 21 ?
        case WIFI_EVENT_STA_BEACON_TIMEOUT:      // 21, Station beacon timeout
            ESP_LOGI(TAG, "ev WIFI_EVENT_STA_BEACON_TIMEOUT..");
            break;

        // 1 ?
        case WIFI_EVENT_SCAN_DONE:              // 1, Finished scanning AP
            ESP_LOGI(TAG, "ev WIFI_EVENT_SCAN_DONE..");
            break;

        // 43 ?
        case WIFI_EVENT_HOME_CHANNEL_CHANGE:    // 43, WiFi home channel change，doesn't occur when scanning
            ESP_LOGI(TAG, "ev WIFI_EVENT_HOME_CHANNEL_CHANGE..");
            break;

        default:
            line = __LINE__;
            goto err;
        }

    // IP_EVENT
    } else if (ev_base == IP_EVENT) {

        switch (ev_id) {
        case IP_EVENT_STA_GOT_IP:       // source: esp_wifi_connect()

            ip_event_got_ip_t* ev = (ip_event_got_ip_t*) ev_data;
            ESP_LOGI(TAG, "ev connected, ip="IPSTR, IP2STR(&ev->ip_info.ip));
            retry_connect = 0;
            wifi_flag.wifi_got_ip = true;  // ??

            break;
        
        default:
            line = __LINE__;
            goto err;
        }

    // ESP_HTTP_CLIENT_EVENT
    } else if (ev_base == ESP_HTTP_CLIENT_EVENT) {
        // all HTTP events are handled by function: _http_event_handler(..)
        // right now do nothing here

    } else {
        line = __LINE__;
        goto err;
    }
    return;
err:
    ESP_LOGI(TAG, "err, event_base:%s, event id=%d line=%d", ev_base, (int)ev_id, line);
}




void wifi_lwip_init_phase() {
    if (wifi_flag.esp_netif_init == false) {
        ESP_LOGI(TAG, "netif init...");
        ESP_ERROR_CHECK(esp_netif_init());  // creates task "tiT" init LwIP, only once!
        wifi_flag.esp_netif_init = true;
    }

    if (wifi_flag.esp_wifi_init == false) {
        ESP_LOGI(TAG, "wifi init...");
        wifi_init_config_t w = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&w));     // create task "wifi"
        wifi_flag.esp_wifi_init = true;
    }

    if (wifi_flag.esp_event_loop_created == false) {
        ESP_LOGI(TAG, "create event loop and handler...");
        ESP_ERROR_CHECK(esp_event_loop_create_default());   // creates task "sys_evt" and queue
        wifi_flag.esp_event_loop_created = true;
        if (instance_any_id == NULL) {
            ESP_ERROR_CHECK(esp_event_handler_instance_register(ESP_EVENT_ANY_BASE, 
                        ESP_EVENT_ANY_ID, &event_handler_any, NULL, &instance_any_id));
        }
    }

    if (wifi_flag.default_wifi_sta_created == false) {
        ESP_LOGI(TAG, "create default wifi station...");

        // esp_netif_config_t cfg = ESP_NETIF_DEFAULT_WIFI_STA();
        // esp_netif_t *netif = esp_netif_new(&cfg);
        // assert(netif);
        // ESP_ERROR_CHECK(esp_netif_attach_wifi_station(netif));
        // ESP_ERROR_CHECK(esp_wifi_set_default_wifi_sta_handlers());

        my_netif = esp_netif_create_default_wifi_sta();

        // my_netif valid then set hostname (in netif struct)
        esp_netif_set_hostname(my_netif, hostname);

        wifi_flag.default_wifi_sta_created = true;
    }

}



void wifi_deinit_phase() {
    if (wifi_flag.wifi_started == true) {
        ESP_LOGI(TAG, "stop wifi station...");
        ESP_ERROR_CHECK(esp_wifi_stop());   // creates event WIFI_EVENT_STA_DISCONNECTED
        wifi_flag.wifi_started = false;
    }

    if (wifi_flag.default_wifi_sta_created == true) {
        ESP_LOGI(TAG, "destroy default wifi station...");
        if (my_netif != NULL) {

            // esp_wifi_clear_default_wifi_driver_and_handlers(my_netif);
            // esp_netif_destroy(my_netif);
            
            esp_netif_destroy_default_wifi(my_netif);

            my_netif = NULL;

        }
        wifi_flag.default_wifi_sta_created = false;
    }

    if (wifi_flag.esp_event_loop_created == true) {
        if (instance_any_id != NULL) {
            ESP_LOGI(TAG, "remove event handler and loop...");
            ESP_ERROR_CHECK(esp_event_handler_instance_unregister(
                    ESP_EVENT_ANY_BASE, ESP_EVENT_ANY_ID, instance_any_id));
            instance_any_id = NULL;
        }

        // ESP_LOGI(TAG, "...event_loop_delete_default...");
        esp_event_loop_delete_default();    // remove task "sys_evt"
        wifi_flag.esp_event_loop_created = false;
    }

    if (wifi_flag.esp_wifi_init == true) {
        ESP_LOGI(TAG, "wifi deinit...");
        esp_wifi_deinit();          // remove task "wifi"
        wifi_flag.esp_wifi_init = false;
    }

    // remove ssid and password
    memset(&wifi_config, 0, sizeof(wifi_config.sta));
    retry_connect = 0;
}


// my_tasks_loop() is called every 10ms
void my_tasks_loop() {
    uint32_t tn;
    
    // check for notifications from xTaskNotify()
    tn = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    // check and run 10ms system timer
    if (tn & BIT_TIMER_10MS) {
        ewait_timer_task();     // run the ewait timer task
    }
    
    // NOT used, just here as an example
    if (tn & BIT_TEST) {
        tmp_cnt++;
    }

    // run the wifi state machine, every 10ms
    switch (wifi_state) {
    case WIFI_PHASE_IDLE:
        if (++cnt_1 >= T_10MS(100)) {       // 1 sec
            ++cnt_2;
            cnt_1 = 0;
        }
        break;

    case WIFI_PHASE_INIT:          // cmd cmd_wifi_start(..)
        wifi_lwip_init_phase();
        memset(&wifi_config, 0, sizeof(wifi_config.sta));
        strcpy((char*)wifi_config.sta.ssid, ssid);
        if (strlen(password) > 0) {
            strcpy((char*)wifi_config.sta.password, password);
            wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        }

        // esp_wifi_disconnect(); ??
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));  // WIFI_MODE_NULL also possible
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_LOGI(TAG, "start the wifi...");
        ESP_ERROR_CHECK(esp_wifi_start());      // gives event WIFI_EVENT_STA_START
        
        wifi_state = WIFI_PHASE_IDLE;   // wait for  WIFI_EVENT_STA_START then to WIFI_PHASE_CONNECT
        break;


    case WIFI_PHASE_DEINIT:         // cmd cmd_wifi_end(..)
        wifi_deinit_phase();
        ESP_LOGI(TAG, "wifi end done");
        wifi_state = WIFI_PHASE_IDLE;
        break;


    default:
        break;
    }

}


void uart_read() {
    char rx_char = 0;
    
    static int i = 0;
    static uart_event_t event;
    static char rx_buf[RX_BUF_LEN] = { 0 };

    // if(pdPASS == xQueueReceive(uart0_queue, (void*)&event, (TickType_t)portMAX_DELAY)) {
    if(pdPASS == xQueueReceive(uart0_queue, (void*)&event, pdMS_TO_TICKS(10))) {
        if (event.type == UART_DATA) {
            uart_read_bytes(UART_NUM_0, &rx_char, event.size, portMAX_DELAY);
            
            if (rx_char != CR) {
                if (rx_char != LF) {
                    if (rx_char >= 0x20 && rx_char <= 0x7e) {   // 0x20=<space>, 0x7e='~'
                        if (i < RX_BUF_LEN) {
                            rx_buf[i++] = rx_char;
                        }
                    }
                }
            } 
        } else {
            ESP_LOGI(TAG, "error, event type: %d", event.type);
            uart_flush_input(UART_NUM_0);
            xQueueReset(uart0_queue);
        }
    }

    if (rx_char == CR) {        // CR then possibly valid command received
        printf("\r\n");         // always reply with newline
        if (strlen(rx_buf) > 0) {
            if (!cmd_interpreter(rx_buf)) {
                printf("err cmd invalid\r\n");
            }
            memset(rx_buf, 0, RX_BUF_LEN);
        }

        i = 0;
        rx_char = 0;
    } else if (rx_char != 0) {
        uart_write_bytes(UART_NUM_0, &rx_char, 1);   // echo received char
        rx_char = 0;
    }
}


// id is run_task_hdl
static void run_task(void *pvParameters) {
    uart_driver_install(UART_NUM_0, BUF_SIZE, BUF_SIZE, 20, &uart0_queue, 0);
    for(;;) {
        uart_read();
        my_tasks_loop();
    }

    uart_driver_delete(UART_NUM_0);
    vTaskDelete(NULL);
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());      // Initialize NVS, flash is needed by wifi

    // Dev board ESP32-Ethernet-Kit_B_V1.0 has been fitted with green LED
    // on port pin GPIO_NUM_4, if other board is use change pin accordingly
    gpio_reset_pin(GPIO_NUM_4);
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);

    xTaskCreate(run_task, "run_task", 8192, NULL, 3, &run_task_hdl);    // 2048, 4096 does not work
    // xTaskCreate(uart_task, "uart_task", 2048, NULL, 5, NULL);

    ewait_clear_all_timers();       // clear the ewait timer list
    
    // create free running rtos timer with interval of 10ms
    if (led_timer_hdl == NULL) {
        led_timer_hdl = xTimerCreate("timer 10ms", pdMS_TO_TICKS(10), pdTRUE, 0, rtos_timer_cb);
        if (pdTRUE == xTimerStart(led_timer_hdl, 0)) {
            gpio_set_level(GPIO_NUM_4, 0);  // 0: led on
            // ESP_LOGI(TAG, "ok timer 10ms");
        } else {
            ESP_LOGE(TAG, "err xTimerStart");
        }
    } else {
        ESP_LOGE(TAG, "err xTimerCreate");
    }

    // testing only, flash green LED, 20ms on, 30ms off
    cmd_interpreter("gflash=20 30");
    
    ESP_LOGI(TAG, "app running");
}



/*
// play with this
esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(2,
                           ESP_SNTP_SERVER_LIST("time.windows.com", "pool.ntp.org" ) );
esp_netif_sntp_init(&config);


WIFI:
- init
- config
- connect
/home/kl/esp/esp-idf/components/esp_wifi/test_apps
/home/kl/esp/esp-idf/components/esp_wifi/test_apps/wifi_connect/main/test_wifi_conn.c
/home/kl/esp/esp-idf/components/esp_driver_tsens/test_apps/temperature_sensor/main/test_temperature_phy.c

CONFIG_LOG_DEFAULT_LEVEL        // ?
ESP_LOG_BUFFER_HEXDUMP(TAG, pBufT10ms, 24, ESP_LOG_INFO);   // dump mem

greenled_flash(T_10MS(30), T_10MS(80));

// esp_netif
esp_netif_t *esp_netif_create_default_wifi_sta(void)
esp_netif_t *esp_netif_create_default_wifi_ap(void)
esp_netif_t *esp_netif_create_default_wifi_nan(void)
esp_netif_t *esp_netif_create_wifi(wifi_interface_t wifi_if, const esp_netif_inherent_config_t *esp_netif_config)
---
void        esp_netif_destroy_default_wifi(void *my_netif)
---
esp_err_t   esp_wifi_set_default_wifi_sta_handlers(void)
esp_err_t   esp_wifi_set_default_wifi_ap_handlers(void)
esp_err_t   esp_wifi_set_default_wifi_nan_handlers(void)
---
esp_err_t   esp_wifi_clear_default_wifi_driver_and_handlers(void *netif)
---
esp_err_t   esp_netif_attach_wifi_station(esp_netif_t *netif)
esp_err_t   esp_netif_attach_wifi_ap(esp_netif_t *netif)
---

Old: xTaskNotify(), new: xTaskNotifyIndexed() but then with uxIndexToNotify = 0

// notifications giving or sending
BaseType_t  xTaskNotifyGive(       TaskHandle_t xTaskToNotify);
BaseType_t  xTaskNotifyGiveIndexed(TaskHandle_t xTaskToNotify, UBaseType_t uxIndexToNotify);

BaseType_t  xTaskNotify(       TaskHandle_t xTaskToNotify,                               uint32_t ulValue, eNotifyAction eAction);
BaseType_t  xTaskNotifyIndexed(TaskHandle_t xTaskToNotify,  UBaseType_t uxIndexToNotify, uint32_t ulValue, eNotifyAction eAction);

// notifications receiving
uint32_t    ulTaskNotifyTake(                                   BaseType_t xClearCountOnExit, TickType_t xTicksToWait);
uint32_t    ulTaskNotifyTakeIndexed(BaseType_t uxIndexToWaitOn, BaseType_t xClearCountOnExit, TickType_t xTicksToWait);
            xTaskNotifyWait ...
sending notification                receiving notification
-------------------------------------------------------------
xTaskNotifyGive()           <-->    ulTaskNotifyTake()
xTaskNotifyGiveIndexed()    <-->    ulTaskNotifyTakeIndexed()
xTaskNotify()               <-->    ulTaskNotifyTake()          // using this one here!
xTaskNotify()               <-->    xTaskNotifyWait()
xTaskNotifyIndexed()

#define configTASK_NOTIFICATION_ARRAY_ENTRIES    CONFIG_FREERTOS_TASK_NOTIFICATION_ARRAY_ENTRIES
/home/kl/esp/esp-idf/components/freertos/config/include/freertos/FreeRTOSConfig.h

// file:///home/kl/esp/docs/rtos/FreeRTOS_Reference_Manual_V10.0.0.pdf
When a task is using its notification value as a binary or counting semaphore other tasks and
interrupts should send notifications to it using either the xTaskNotifyGive() macro, or the
xTaskNotify() function with the function's eAction parameter set to eIncrement (the two are
equivalent).   

// file:///home/kl/esp/docs/rtos/FreeRTOS_Reference_Manual_V10.0.0.pdf   (2.18 xTaskNotifyGive)
xTaskNotifyGive() is a macro that calls xTaskNotify() with the eAction parameter set to
eIncrement. Therefore pdPASS is always returned.

Where as xTaskNotifyWait() will return when a notification is pending, ulTaskNotifyTake() will
return when the task's notification value is not zero, decrementing the task's notification value
before it returns.

Return Values
The value of the task's notification value before it is decremented or cleared (see the
description of xClearCountOnExit).
uint32_t ulTaskNotifyTake( BaseType_t xClearCountOnExit, TickType_t xTicksToWait );


// event waits then use
EventGroupHandle_t  xEventGroupCreate()
EventBits_t         xEventGroupSetBits(EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToSet)
EventBits_t         xEventGroupWaitBits(const EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToWaitFor,
                            const BaseType_t xClearOnExit, const BaseType_t xWaitForAllBits, TickType_t xTicksToWait);
void                vEventGroupDelete(EventGroupHandle_t xEventGroup)
https://www.freertos.org/event-groups-API.html


// don't know difference
xTimerStart(led_timer_hdl, pdMS_TO_TICKS(0))
xTimerStart(led_timer_hdl, pdMS_TO_TICKS(10))
xTimerStart(led_timer_hdl, pdMS_TO_TICKS(portMAX_DELAY))

configUSE_TIMERS and configSUPPORT_DYNAMIC_ALLOCATION must both be set to 1 in FreeRTOSConfig.h
https://www.freertos.org/FreeRTOS-timers-xTimerCreate.html


uxTaskPriorityGet(NULL): same priority as the current task
    xTaskCreate(application_task, "application_task", 2048, NULL, uxTaskPriorityGet(NULL), NULL);

// what is the difference between these ?? :
vTaskDelay(pdMS_TO_TICKS(1000));        // use this, unit is ms
vTaskDelay(1000 / portTICK_PERIOD_MS);  // do not use


// ESP_GOTO_ON_FALSE  what is this? in: 
/home/kl/esp/esp-idf/components/esp_common/include/esp_check.h

// formatting
uart_tx_chars()                                  // non-blocking
printf("led off %s, %s, %s, %d", __TIME__, __FUNCTION__, __FILE__, __LINE__);
// __FILE__ points to "/apps/test.c" where as __FILENAME__ points to "test.c"
ESP_LOGD(TAG, "%s esp-netif:%p event-id%d", __func__, esp_netif, event_id);

// find task and delete
TaskHandle_t taskhdl = xTaskGetHandle("sys_evt");
if (taskhdl) {
    esp_event_loop_delete_default();        // remove task "sys_evt"
    ESP_LOGI(TAG, "deleted sys_evt task");
}

// hexdump, to JTAG?
ESP_LOG_BUFFER_HEXDUMP(TAG, (void*)&wifi_config, sizeof(wifi_config.sta), ESP_LOG_INFO);
ESP_LOG_BUFFER_HEXDUMP(TAG, buffer, len, ESP_LOG_VERBOSE);
ESP_LOG_BUFFER_HEXDUMP(TAG, pBufT10ms, 24, ESP_LOG_INFO);
/home/kl/esp/esp-idf/components/log/include/esp_log.h
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/log.html?highlight=esp_log_buffer_hexdump#logging-to-host-via-jtag

// examples on github
https://github.com/espressif/esp-idf/tree/d4cd437e/examples/wifi

// toggle led
gpio_set_level(GPIO_NUM_4, (++led_cnt % 2));    // flash led on GPIO_NUM_4, 0 is on

// sniffer info
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#wi-fi-sniffer-mode

// from golang, not sure if all will work
%b 	    Base 2
%d 	    Base 10
%+d 	Base 10 and always show sign
%x 	    Base 16, lowercase
%X 	    Base 16, uppercase
%#x 	Base 16, with leading 0x
%4d 	Pad with spaces (width 4, right justified)
%-4d 	Pad with spaces (width 4, left justified)
%04d 	Pad with zeroes (width 4

ESP_LOGI(UART_TAG, "queue free spaces: %" PRIu32, (uint32_t)uxQueueSpacesAvailable(p_uart_obj[uart_num]->event_queue));

git describe
IDF Version: v5.4-dev-2744-g59e1838270-dirty
IDF Version: v5.4-dev-2306-gdbce23f8a4-dirty 
*/


/* 
static void event_handler_wifi_start(void* arg, esp_event_base_t event_base,
                                            int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_STA_START) {
        // task notify command
        xTaskNotify(run_task_hdl, BIT_WIFI_EVENT_STA_START, eSetBits); // notification to receive: ulTaskNotifyTake()
    } else if (event_id == WIFI_EVENT_SCAN_DONE) {
        // task notify command
        xTaskNotify(run_task_hdl, BIT_WIFI_EVENT_SCAN_DONE, eSetBits); // notification to receive: ulTaskNotifyTake()
    } else {
        ESP_LOGI(TAG, "...unknown id: (%d)...", (int)event_id);
    } 
}

isalpha(c)
isalnum(c)
isspace(c)
isdigit(c)
isxdigit(c)
iscntrl(c)
ispunct(c)
islower(c)
isupper(c)
*/


