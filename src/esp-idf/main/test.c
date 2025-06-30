#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"

int test_int = 0;

void test_log(){
    ESP_LOGI(NULL, "test int:%d", test_int);


}
