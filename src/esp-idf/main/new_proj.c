#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/time.h>
//nvs
#include "nvs.h"
#include "nvs_flash.h"
//filesys
#include "esp_system.h"
#include "esp_littlefs.h"
//bt
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
    //#include "esp_bt_device.h"
    //#include "esp_gap_bt_api.h"
//log
#include "esp_log.h"
//freertos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//test
#include "test.h"
//uart
#include "driver/uart.h"
#include "driver/gpio.h"

//data buffer sizes
#define BUFFER_LEN 64
#define BUFFER_ITEM 13
//gatt tag
#define GATTC_TAG "GATT"

static void* my_fp;
static int my_ctr = 0;
static int my_read = 0;
static int16_t last_time = 0;
//static callback func declare
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
//static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static bool new_time = false;
static SemaphoreHandle_t sem_save;

//PARAMETER STRUCTS
//scan parameters
static esp_ble_scan_params_t ble_scan_params = {
    //active or passive scan, active for more details
    //esp_ble_scan_type_t
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,//PASSIVE
    //adr for scan public for now
    //esp_ble_addr_type_t
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    //filter which allowed advertising packets
    //esp_ble_scan_filter_t
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    //range between start of ble scans  0x0004-0x4000  2.5msec-10.24seconds
    //uint16_t
    .scan_interval          = 0x1000,
    //time between start and end of scan  0x0004-0x4000  2.5msec-10.24seconds
    //uint16_t
    .scan_window            = 0x0FFF,
    //whether to drop duplicate packets, disable means dont drop
    //esp_ble_scan_duplicate_t
    .scan_duplicate         = BLE_SCAN_DUPLICATE_ENABLE_RESET//BLE_SCAN_DUPLICATE_DISABLE
};

//lfs config
static esp_vfs_littlefs_conf_t fs_conf = {
            .base_path = "/mystore",
            .partition_label = "mystore",
            .format_if_mount_failed = true,
            .dont_mount = false,
};

//uart config
static uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
};

//FUNCTIONS
//esp callback
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param){
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    //store last 12 bits of seconds and first 4 bits of nanoseconds //1 hours till rollover, need file write before then
    int16_t curr_time = (int16_t)tv.tv_sec>>4;  //0123.2340
    if(last_time != curr_time){
        last_time = curr_time;
        new_time = true;
        xSemaphoreGive(sem_save);
    }
    int16_t time_us = ((int16_t)(tv.tv_sec <<4)) + ((int16_t)((tv.tv_usec&0x0FFFFF)>>16));// 1234, abcd > 234a
    //FILE *f = fopen("/littlefs/hello.txt", "a");

    //switch of events // there is over 50 of these but not all needed
    //esp_gap_ble_cb_event_t
    switch (event) {

    //When scan parameters set complete
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //scan for 30 secs // sends followign event
        uint32_t duration = 20;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    //esp_ble_gap_read_rssi(esp_bd_addr_t remote_addr)

    //when start scan complete
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:{
        //indicates if scanning start succeeded
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "Scanning start failed: %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Scanning start successful");
        break;
    }

    //When each scan result ready
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        //cast parameter
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;

        //switch of scan result search event enum
        //event types for scan result, eg inquiry types
        //esp_ble_gap_cb_param_t->ble_scan_result_evt_param.esp_gap_search_evt_t
        switch (scan_result->scan_rst.search_evt) {
        //inquiry result event
        case ESP_GAP_SEARCH_INQ_RES_EVT:{
            //get data from parameters
            adv_name = esp_ble_resolve_adv_data_by_type(scan_result->scan_rst.ble_adv,                                                        scan_result->scan_rst.adv_data_len + scan_result->scan_rst.scan_rsp_len,                                                        ESP_BLE_AD_TYPE_NAME_CMPL,&adv_name_len);

            //log data
            ESP_LOGI(GATTC_TAG, "Scan result, device "ESP_BD_ADDR_STR", %d, %lu, %d", ESP_BD_ADDR_HEX(scan_result->scan_rst.bda), scan_result->scan_rst.rssi, sizeof(scan_result->scan_rst.bda), time_us);
            ESP_LOG_BUFFER_CHAR(GATTC_TAG, adv_name, adv_name_len);
            //ESP_LOGI(GATTC_TAG, "val %.2x%.2x%.2x%.2x%.2x%.2x%.8x%.4x",p[0],p[1],p[2],p[3],p[4],p[5],scan_result->scan_rst.rssi,time_us);

            //update ctr
            //checking ctr not used anymre

            void * c = my_fp+(BUFFER_ITEM*my_ctr);
            uint8_t *p = scan_result->scan_rst.bda;
            memcpy(c, p, 6);
            memcpy(c+6, &scan_result->scan_rst.rssi, 4);
            memcpy(c+10, &time_us, 2);
            memset(c+12, 0, 1);
            ESP_LOGI(GATTC_TAG, "Scan result,sd");
            xSemaphoreGive(sem_save);
            my_ctr++;
            my_ctr%=BUFFER_LEN;
            break;

        }
        //inquiry complete event
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }

    //when stop scan completes
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:{
        //indicates if scanning stop succeeded
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "Scanning stop failed:%x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Scanning stop successful");
        //scan
        uint32_t duration = 20;
        esp_ble_gap_start_scanning(duration);
        break;
    }

    //when advertising stop completes
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:{
        //check if success
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "Advertising stop failed: %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Advertising stop successful");
        break;
    }

    //when completed connection params update
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:{
         //logs new parameters
         ESP_LOGI(GATTC_TAG, "Conn params up, status: %d, conn_int: %d, latency: %d, timeout: %d",
                  param->update_conn_params.status,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    }

    //after completed package lenght change
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:{
        //log new params
        ESP_LOGI(GATTC_TAG, "Packet len up, status: %d, rx: %d, tx: %d",
                  param->pkt_data_length_cmpl.status,
                  param->pkt_data_length_cmpl.params.rx_len,
                  param->pkt_data_length_cmpl.params.tx_len);
        break;
    }

    //def
    default:{
        ESP_LOGE(GATTC_TAG, "result event:%x", event);
        break;
    }

    }
}

//setup ble
void s_ble_task(void *pvParameter){
    //release classic bt memory, allows only ble
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    //generate default bt config struct
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    while(1){
        //initialise bt controller with given configs
        esp_err_t ret = esp_bt_controller_init(&bt_cfg);

        //error check
        if (ret) {
            ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
            continue;
        }else{
            ESP_LOGI(GATTC_TAG, "init controller");
        }
        //enable bt controller must use same mode as in controller init
        ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);//ESP_BT_MODE_BTDM);
        //error check
        if (ret) {
            ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
            continue;
        }else{
            ESP_LOGI(GATTC_TAG, "enable controller");
        }

        //init bluetooth code
        //can use esp_bluedroid_init_with_cfg() //takes esp_bluedroid_config_t
        //only config is bool for SSP for classic ble (irrelevant here)
        ret = esp_bluedroid_init();

        //error check
        if (ret) {
            ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
            continue;
        }else{
            ESP_LOGI(GATTC_TAG, "init bluetooth");
        }
        //enable bluetooth code
        ret = esp_bluedroid_enable();

        //error check
        if (ret) {
            ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
            continue;
        }else{
            ESP_LOGI(GATTC_TAG, "enable bluetooth");
        }

        // refister callback for gap event, this case scan result
        // takes typedef of esp_gap_ble_cb_t, defined above as esp_gap_cb
        // takes esp_gap_ble_cb_event_t >> this is enum of event triggering callback
        // also takes esp_ble_gap_cb_param_t  >> struct of function parameters

        // Note from source: Avoid performing time-consuming operations within callback functions.

        ret = esp_ble_gap_register_callback(esp_gap_cb);
        //once again error check
        if (ret){
            ESP_LOGE(GATTC_TAG, "%s gap register failed, err: %x", __func__, ret);
            continue;
        }else{
            ESP_LOGI(GATTC_TAG, "gap register");
        }
        //set scan parameters
        ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        //error check
        if (ret){
            ESP_LOGE(GATTC_TAG, "%s params set failed, err: %x", __func__, ret);
            continue;
        }else{
            ESP_LOGI(GATTC_TAG, "scan param set");
        }

        /*
        //register callback for classic bt
        ret = esp_bt_gap_register_callback(bt_app_gap_cb);
        //err check
        if (ret){
            ESP_LOGE(GATTC_TAG, "%s classic gap register failed, err: %x", __func__, ret);
            continue;
        }else{
            ESP_LOGI(GATTC_TAG, "classic gap register");
        }
        //set bt scan mode
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        //error check
        if (ret){
            ESP_LOGE(GATTC_TAG, "%s classic params set failed, err: %x", __func__, ret);
            continue;
        }else{
            ESP_LOGI(GATTC_TAG, "classic scan param set");
        }
        */

        break;
    }
    while(1){
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        esp_ble_gap_stop_scanning();
    }
    //delete this task
    vTaskDelete(NULL);
}

//setup nvs
void s_nvs_task(void *pvParameter){
    // init flash
    //err code return
    esp_err_t ret = nvs_flash_init();

    //error check in flash init
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    //delete this task
    vTaskDelete(NULL);
}

//mount littlefs and create file
void s_fs_task(void *pvParameter){
    // init littlefs
    esp_err_t ret = esp_vfs_littlefs_register(&fs_conf);

    //error check
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s littlefs failed, err: %x", __func__, ret);
    }

    size_t total = 0;
    size_t used = 0;
    //get partition info
    ret = esp_littlefs_info(fs_conf.partition_label, &total, &used);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s partition check failed, err: %x", __func__, ret);
    }

    //check for file
    struct stat s_buf;
    stat("/mystore/data.bin", &s_buf);

    //create file
    FILE *fptr;
    fptr = fopen("/mystore/data.bin", "w");
    if(fptr == NULL){
        ESP_LOGE(GATTC_TAG, "%s file create failed", __func__);
    }
    fclose(fptr);

    //store start time in file 64 bit second value, all other times are 32 bits so serves as anchor
    struct timeval tv;
    gettimeofday(&tv, NULL);
    fptr = fopen("/mystore/data.bin", "a");
    if(fptr == NULL){
        ESP_LOGE(GATTC_TAG, "%s file create failed", __func__);
    }
    fprintf(fptr, "g%llx", (int64_t)tv.tv_sec);
    fclose(fptr);


    //delete this task
    vTaskDelete(NULL);
}

//unmount littlefs
void s_close_fs_task(void *pvParameter){
    esp_vfs_littlefs_unregister(fs_conf.partition_label);
    vTaskDelete(NULL);
}

//setup uart for usb
void s_uart_usb_task(void *pvParameter){
    // set UART params
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));

    //received data buffer
    uint8_t recv[8];

    // data to send buffer
    char word[256];


    while(1){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        int l = uart_read_bytes(UART_NUM_0, recv, 8, 100 / portTICK_PERIOD_MS);

        if(l>0){//printf("%s\n", recv);
            ESP_LOGE(GATTC_TAG, "start uart");
            //stop debug log because same channel used for file transfer
            esp_log_level_set("*", ESP_LOG_NONE);
            FILE *fptr;
            fptr = fopen("/mystore/data.bin", "r");
            while (1) {
                size_t ln = fread(word, 1, 256, fptr);
                if(ln<=0){
                    fclose(fptr);
                    break;
                }
                // Write data
                uart_write_bytes(UART_NUM_0, (const char *) word, 256);
            }
        }else{
            continue;
        }
        //break;

    }


    //delete task on complete
    vTaskDelete(NULL);
}
//save from circular buffer to file
void save_task(void *pvParameter){
    while(1){
        xSemaphoreTake(sem_save, portMAX_DELAY);
        FILE *fptr;
        fptr = fopen("/mystore/data.bin", "a");
        if(new_time){//save time to file
            ESP_LOGE(GATTC_TAG, "1");
            fprintf(fptr, "ZX");
            for (int i=0;i<2;i++)fprintf(fptr, "%.2x", ((char *)&last_time)[i]);
            new_time=false;
        }
        while(my_read!=my_ctr){
                if(new_time){
                    fprintf(fptr, "ZX");
                    for (int i=0;i<2;i++)fprintf(fptr, "%.2x", ((char *)&last_time)[i]);
                    new_time=false;
                }//save data to file

                fprintf(fptr, "ZY");
                for (int i=0;i<BUFFER_ITEM-1;i++)fprintf(fptr, "%.2x", ((char *)my_fp+BUFFER_ITEM*my_read)[i]);
                my_read++;
                my_read%=BUFFER_LEN;
        }
        fclose(fptr);
        xSemaphoreTake(sem_save, 0);

    }

}
//program entry
void app_main(void){
    //TIMESTAMP 16 bit, mac is 48 bit, rssi is 32 bit + 8 bit delimited => 13 bytes
    my_fp = (void*)malloc(BUFFER_ITEM * BUFFER_LEN);//test_log();

    //BLE callbacks are run in core 0 //rest should run in core 1
    //task func ptr, process name max 16 char, stack depth bytes, parameters pointer, priority, task handle to reference task, core to run on

    //semaphores
    sem_save = xSemaphoreCreateCounting(1, 0);
    if(sem_save==NULL){
         ESP_LOGI(GATTC_TAG, "null sem");
    }
    //esp_log_level_set("*", ESP_LOG_NONE);

    //create tasks
    xTaskCreatePinnedToCore(&s_nvs_task, "s_nvs_task", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(&s_fs_task, "s_fs_task", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(&s_ble_task, "s_ble_task", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(&save_task, "save_task", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(&s_uart_usb_task, "s_uart_usb_task", 4096, NULL, 1, NULL, 0);

    /*
    while (1) {
        extern void bt_hci_log_hci_data_show(void);
        extern void bt_hci_log_hci_adv_show(void);
        bt_hci_log_hci_data_show();
        bt_hci_log_hci_adv_show();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    */

}
