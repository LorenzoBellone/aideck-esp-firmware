// Copyright 2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mdf_common.h"
#include "mwifi.h"

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_event.h"
#include "discovery.h"

#include "esp_transport.h"
#include "spi_transport.h"
#include "uart_transport.h"
#include "router.h"
#include "com.h"
#include "test.h"
#include "wifi.h"
#include "system.h"

// #define MEMORY_DEBUG
#define BLINK_GPIO 4
static uint32_t blink_on_period_ms = 1000;
static uint32_t blink_off_period_ms = 1000;


static const char *TAG = "get_started";
static esp_routable_packet_t txp;

static void root_task(void *arg)
{
    mdf_err_t ret                    = MDF_OK;
    char *data                       = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    size_t size                      = MWIFI_PAYLOAD_LEN;
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};
    mwifi_data_type_t data_type      = {0};

    MDF_LOGI("Root is running");

    for (int i = 0;; ++i) {
        if (!mwifi_is_started()) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        size = MWIFI_PAYLOAD_LEN;
        memset(data, 0, MWIFI_PAYLOAD_LEN);

        ret = mwifi_root_read(src_addr, &data_type, data, &size, portMAX_DELAY);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_root_read", mdf_err_to_name(ret));
        // MDF_LOGI("Root receive, addr: " MACSTR ", size: %d, data_type: %d", MAC2STR(src_addr), size, data_type.custom);

        wifi_send_packet((const char*) data, size);
        // MDF_LOGI("after root write %x", ret);
        // MDF_ERROR_CONTINUE(ret != MDF_OK, "mwifi_root_recv, ret: %x", ret);
        // MDF_LOGI("Root send, addr: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);
    }

    MDF_LOGW("Root is exit");

    MDF_FREE(data);
    vTaskDelete(NULL);
}


/**
 * @brief Timed printing system information
 */
static void print_system_info_timercb(void *timer)
{
    uint8_t primary                 = 0;
    wifi_second_chan_t second       = 0;
    mesh_addr_t parent_bssid        = {0};
    uint8_t sta_mac[MWIFI_ADDR_LEN] = {0};
    wifi_sta_list_t wifi_sta_list   = {0x0};

    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    esp_wifi_get_channel(&primary, &second);
    esp_mesh_get_parent_bssid(&parent_bssid);

    MDF_LOGI("System information, channel: %d, layer: %d, self mac: " MACSTR ", parent bssid: " MACSTR
             ", parent rssi: %d, node num: %d, free heap: %u", primary,
             esp_mesh_get_layer(), MAC2STR(sta_mac), MAC2STR(parent_bssid.addr),
             mwifi_get_parent_rssi(), esp_mesh_get_total_node_num(), esp_get_free_heap_size());

    for (int i = 0; i < wifi_sta_list.num; i++) {
        MDF_LOGI("Child mac: " MACSTR, MAC2STR(wifi_sta_list.sta[i].mac));
        MDF_LOGI("Child rssi: %d", wifi_sta_list.sta[i].rssi);
    }

#ifdef MEMORY_DEBUG

    if (!heap_caps_check_integrity_all(true)) {
        MDF_LOGE("At least one heap is corrupt");
    }

    mdf_mem_print_heap();
    mdf_mem_print_record();
    mdf_mem_print_task();
#endif /**< MEMORY_DEBUG */
}


static void log_stats_timercb(void *timer)
{
    wifi_sta_list_t wifi_sta_list   = {0x0};
    esp_wifi_ap_get_sta_list(&wifi_sta_list);

    MDF_LOGI("RSSI p  %d", mwifi_get_parent_rssi());
    for (int i = 0; i < wifi_sta_list.num; i++) {
        MDF_LOGI("RSSI c%d %d", i, wifi_sta_list.sta[i].rssi);
    }
    MDF_LOGI("LAYER %d", esp_mesh_get_layer());
    MDF_LOGI("NODES %d", esp_mesh_get_total_node_num());
}


static mdf_err_t wifi_init()
{
    mdf_err_t ret          = nvs_flash_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    wifi_init_streaming();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        MDF_ERROR_ASSERT(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    MDF_ERROR_ASSERT(ret);

    MDF_ERROR_ASSERT(esp_netif_init());
    MDF_ERROR_ASSERT(esp_event_loop_create_default()); 
    MDF_ERROR_ASSERT(esp_wifi_init(&cfg));
    MDF_ERROR_ASSERT(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    MDF_ERROR_ASSERT(esp_wifi_set_mode(WIFI_MODE_STA));
    MDF_ERROR_ASSERT(esp_wifi_set_ps(WIFI_PS_NONE));
    MDF_ERROR_ASSERT(esp_mesh_set_6m_rate(false));
    MDF_ERROR_ASSERT(esp_mesh_set_topology(MESH_TOPO_CHAIN));
    MDF_ERROR_ASSERT(esp_wifi_start());

    return MDF_OK;
}

/**
 * @brief All module events will be sent to this task in esp-mdf
 *
 * @Note:
 *     1. Do not block or lengthy operations in the callback function.
 *     2. Do not consume a lot of memory in the callback function.
 *        The task memory of the callback function is only 4KB.
 */
static mdf_err_t event_loop_cb(mdf_event_loop_t event, void *ctx)
{
    MDF_LOGI("event_loop_cb, event: %d", event);

    switch (event) {
        case MDF_EVENT_MWIFI_STARTED:
            MDF_LOGI("MESH is started");
            break;

        case MDF_EVENT_MWIFI_PARENT_CONNECTED:
            MDF_LOGI("Parent is connected on station interface");
            break;

        case MDF_EVENT_MWIFI_PARENT_DISCONNECTED:
            MDF_LOGI("Parent is disconnected on station interface");
            break;
        case MDF_EVENT_MWIFI_ROUTING_TABLE_ADD:
        case MDF_EVENT_MWIFI_ROUTING_TABLE_REMOVE:
            MDF_LOGI("total_num: %d", esp_mesh_get_total_node_num());
            break;
        case MDF_EVENT_MWIFI_ROOT_GOT_IP:
            MDF_LOGI("Root obtains the IP address. It is posted by LwIP stack automatically");
            // xTaskCreate(tcp_client_write_task, "tcp_client_write_task", 4 * 1024,
            //             NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
            // xTaskCreate(tcp_client_read_task, "tcp_server_read", 4 * 1024,
            //             NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
            break;
        default:
            break;
    }

    return MDF_OK;
}

int cpx_and_uart_vprintf(const char * fmt, va_list ap) {
    int len = vprintf(fmt, ap);

    cpxInitRoute(CPX_T_ESP32, CPX_T_STM32, CPX_F_CONSOLE, &txp.route);
    txp.dataLength = vsprintf((char*)txp.data, fmt, ap) + 1;
    espAppSendToRouterBlocking(&txp);

    return len;
}

/* Task for handling WiFi state */
static void streamer_wifi_task(void *pvParameters) {
  wifi_bind_socket();
  while (1) {
    blink_on_period_ms = 1000;
    wifi_wait_for_socket_connected();
    blink_on_period_ms = 300;
    wifi_wait_for_disconnect();
    ESP_LOGI(TAG, "Client disconnected");
  }
}

void app_main()
{
    MDF_LOGI("Starting NINA application as:");
    MDF_LOGI(CONFIG_AGENT_NAME);

    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BLINK_GPIO, 1);

    mwifi_init_config_t cfg = MWIFI_INIT_CONFIG_DEFAULT();
    mwifi_config_t config   = {
        .router_ssid     = CONFIG_WIFI_SSID,
        .router_password = CONFIG_WIFI_PASSWORD,
        //.channel   = CONFIG_MESH_CHANNEL,
        .channel   = 1,
        .mesh_id   = CONFIG_MESH_ID,
        .mesh_type = CONFIG_DEVICE_TYPE,
    };
    cfg.max_connection = 1;

    /**
     * @brief Set the log level for serial port printing.
     */
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set("SPI", ESP_LOG_INFO);
    esp_log_level_set("UART", ESP_LOG_INFO);
    esp_log_level_set("SYS", ESP_LOG_INFO);
    esp_log_level_set("ROUTER", ESP_LOG_INFO);
    esp_log_level_set("COM", ESP_LOG_INFO);
    esp_log_level_set("TEST", ESP_LOG_INFO);
    esp_log_level_set("WIFI", ESP_LOG_INFO);

    /**
     * @brief Initialize wifi mesh.
     */
    MDF_ERROR_ASSERT(mdf_event_loop_init(event_loop_cb));
    MDF_ERROR_ASSERT(wifi_init());
    MDF_ERROR_ASSERT(mwifi_init(&cfg));
    MDF_ERROR_ASSERT(mwifi_set_config(&config));
    MDF_ERROR_ASSERT(mwifi_start());

    // Intalling GPIO ISR service so that other parts of the code can
    // setup individual GPIO interrupt routines
    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);

    const uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, 1000, 1000, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, 0, 25, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI("SYS", "\n\n -- Starting up --\n");
    ESP_LOGI("SYS", "Minimum free heap size: %d bytes", esp_get_minimum_free_heap_size());

    espTransportInit();
    uart_transport_init();
    com_init();
    // esp_log_set_vprintf(cpx_and_uart_vprintf);
    // system_init();
    // discovery_init();

    /**
     * @brief Data transfer between wifi mesh devices
     */
#if defined(CONFIG_AGENT_ROLE_BASE)
    xTaskCreate(root_task, "root_task", 4 * 1024, NULL,
                CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    xTaskCreate(wifi_task, "wifi_task", 4096, NULL, 5, NULL);
#elif defined(CONFIG_AGENT_ROLE_EXPLORER)
    spi_transport_init();
    xTaskCreate(wifi_sending_task, "wifi_sending_task", 4096,
                NULL, 5, NULL);
    blink_on_period_ms = 2000;
    blink_off_period_ms = 100;
    router_init();
#endif

//     // TimerHandle_t timer = xTimerCreate("log_stats", 1000 / portTICK_RATE_MS,
//     //                                    true, NULL, log_stats_timercb);
//     // xTimerStart(timer, 0);


    while(1) {
        int i=0;
        for(i=0; i<2; i++) {
            gpio_set_level(BLINK_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(blink_off_period_ms));
            gpio_set_level(BLINK_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(blink_on_period_ms));
        }
        for(i=0; i<esp_mesh_get_layer(); i++) {
            gpio_set_level(BLINK_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(BLINK_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
