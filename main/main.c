// main
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
// #include "driver/gpio.h"
// #include "sdkconfig.h"
#include "esp_log.h"
#include "driver/dac.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"

#define GET_STATUS 0xC1
#define SET_PERPHS 0xC2
#define STATUS 0xC3
#define DATA_MASK 0x3F
#define DATA_BIT 0x40

#define MESSAGING_TXD_PIN 17
#define MESSAGING_RXD_PIN 16
#define MESSAGING_UART_PORT_NUM 2
#define BUF_SIZE (128) //a buffer small than this causes an exception on uart_driver_install
#define STATUS_SIZE (12) //includes message_type byte
#define COMMAND_SIZE (9)

#define TASK_STACK_SIZE  2048
static const char *TAG = "LOOP";
// static const char *STATUS = "S";

static void loop_task(void *arg)
{
    // initialize the UART
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

    // initialize the ADC
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = false,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_cfg, &adc_handle));
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_9,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_6, &chan_cfg)); // GPIO34
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_7, &chan_cfg)); // GPIO35
    int adc_chan_value[2];

    // initialize the DAC
    dac_output_enable(DAC_CHANNEL_1);
    //approx 0.78 of VDD_A voltage (VDD * 200 / 255). For VDD_A 3.3V, this is 2.59V
    dac_output_voltage(DAC_CHANNEL_1, 0);

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    uint8_t *status = (uint8_t *) malloc(STATUS_SIZE);
    memset(status,0,STATUS_SIZE);

    //status 1&2: ADC0; 2&3: ADC1, 4&5:RPM0, 5&6:RPM1, 7: carousel bits; 8: temperature1; 9: tempature2
    status[0]= STATUS;
    //test data:
    status[1]= DATA_BIT | 0x04; //0x0123
    status[2]= DATA_BIT | 0x23;
    status[3]= DATA_BIT | 0x3F; //0xFED
    status[4]= DATA_BIT | 0x2D;

    static uint32_t print_len_gt_1_count=0;
    static uint32_t print_unrec_count=0;
    static uint32_t print_set_perphs_count=0;
    static uint32_t get_status_count=0;
    
    while (1) {
        memset(data,0,COMMAND_SIZE-1);
        //the read_bytes(port, buffer, #bytes to read, timeout in ticks, where a tick is 1 mSec)
        //https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html#uart-api-running-uart-communication
        int len = uart_read_bytes(MESSAGING_UART_PORT_NUM, data, 1, 10 / portTICK_PERIOD_MS);
        if (len) {
            if (data[0] == GET_STATUS)
            {
                get_status_count++;
                ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_6, &adc_chan_value[0]));
                ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_7, &adc_chan_value[1]));
                //pack the adc values into the status message
                status[2]= DATA_BIT | (adc_chan_value[0] & DATA_MASK);
                status[1]= DATA_BIT | ((adc_chan_value[0]>>6) & DATA_MASK);
                status[3]= DATA_BIT | (adc_chan_value[1] & DATA_MASK);
                status[4]= DATA_BIT | ((adc_chan_value[1]>>6) & DATA_MASK);

                uart_write_bytes(MESSAGING_UART_PORT_NUM, (const char *) status, 10);
            }
            else if (data[0] == SET_PERPHS)
            {
                print_set_perphs_count++;
                memset(data,0,COMMAND_SIZE-1);
                int len_command = uart_read_bytes(MESSAGING_UART_PORT_NUM, data, COMMAND_SIZE-1, 10 / portTICK_PERIOD_MS);
                uint8_t dac[2];
                dac[0]= ((data[0] & DATA_MASK) << 6) | (data[1] & DATA_MASK);
                dac_output_voltage(DAC_CHANNEL_1, dac[0]);

                // if (print_set_perphs_count < 2)
                    ESP_LOGI(TAG, "Received SET_PERPHS: length=%d dac[0]=%u (0x%02x)", len_command, dac[0], dac[0]);
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
