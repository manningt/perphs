// main
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#define GET_STATUS 0xC1
#define SET_PERPHS 0xC2
#define STATUS 0xC3
#define DATA_MASK 0x3F
#define DATA_BIT 0x40

#define MESSAGING_TXD_PIN 23
#define MESSAGING_RXD_PIN 22
#define MESSAGING_UART_PORT_NUM 2
#define BUF_SIZE (128) //a buffer small than this causes an exception on uart_driver_install
#define STATUS_SIZE (12) //a buffer small than this causes an exception on uart_driver_install

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
    uint8_t *status = (uint8_t *) malloc(STATUS_SIZE);
    memset(status,0,STATUS_SIZE);

    status[0]= STATUS;
    //test data:
    status[1]= DATA_BIT | 0x04; //0x0123
    status[2]= DATA_BIT | 0x23;
    status[3]= DATA_BIT | 0x3F; //0xFED
    status[4]= DATA_BIT | 0x2D;
    //status 1&2: ADC0; 2&3: ADC1, 4&5:RPM0, 5&6:RPM1, 7: carousel bits; 8: temperature1; 9: tempature2

    static uint32_t print_len_gt_1_count=0;
    static uint32_t print_unrec_count=0;
    static uint32_t print_set_perphs_count=0;
    
    while (1) {
        memset(data,0,8);
        //the read_bytes(port, buffer, #bytes to read, timeout in ticks, where a tick is 1 mSec)
        //https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html#uart-api-running-uart-communication
        int len = uart_read_bytes(MESSAGING_UART_PORT_NUM, data, 1, 10 / portTICK_PERIOD_MS);
        if (len) {
            if (data[0] == GET_STATUS)
            {
                uart_write_bytes(MESSAGING_UART_PORT_NUM, (const char *) status, 10);
            }
            else if (data[0] == SET_PERPHS)
            {
                print_set_perphs_count++;
                // TODO: add loop to get next characters
                if (print_set_perphs_count < 2)
                    ESP_LOGI(TAG, "Received SET_PERPHS");
            }
            else
            {
                print_unrec_count++;
                if (print_set_perphs_count < 2)
                    ESP_LOGI(TAG, "Unrecognized command=0x%02x", data[0]);

            }
            if (len > 1)
            {
               print_len_gt_1_count++;
                if (print_len_gt_1_count < 2)
                    ESP_LOGI(TAG, "length=%d greater than 1", len);
            }
            // ESP_LOGI(TAG, "Recv str: %s data_len=%d", (char *) data, len);
        }
    }
}

void app_main(void)
{
    xTaskCreate(loop_task, "loop_task", TASK_STACK_SIZE, NULL, 10, NULL);
}
