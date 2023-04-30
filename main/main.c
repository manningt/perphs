// main
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#define MESSAGING_TXD_PIN 23
#define MESSAGING_RXD_PIN 22
#define MESSAGING_UART_PORT_NUM 2
#define BUF_SIZE (128) //a buffer small than this causes an exception on uart_driver_install

#define TASK_STACK_SIZE  2048
static const char *TAG = "LOOP";
// static const char *STATUS = "S";

static void loop_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // the NO_CHANGE pins are for the RTS/CTS pins, which our application does not use.
    ESP_ERROR_CHECK(uart_set_pin(MESSAGING_UART_PORT_NUM, MESSAGING_TXD_PIN, 
        MESSAGING_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // BUF_SIZE*2 are the TX & RX buffers; queue size, queue handle and interrupt
    ESP_ERROR_CHECK(uart_driver_install(MESSAGING_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MESSAGING_UART_PORT_NUM, &uart_config));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    // ESP_LOGI(TAG, "portTICK_PERIOD_MS=%lu", portTICK_PERIOD_MS);

    while (1) {
        memset(data,0,8);
        //the read_bytes(port, buffer, #bytes to read, timeout in ticks, where a tick is 1 mSec)
        int len = uart_read_bytes(MESSAGING_UART_PORT_NUM, data, 4, 10 / portTICK_PERIOD_MS);
        if (len) {
            // data[len] = '\0';
            ESP_LOGI(TAG, "Recv str: %s data_len=%d", (char *) data, len);
        }
        // Write data back to the UART
        uart_write_bytes(MESSAGING_UART_PORT_NUM, (const char *) data, len);
    }
}

void app_main(void)
{
    xTaskCreate(loop_task, "loop_task", TASK_STACK_SIZE, NULL, 10, NULL);
}
