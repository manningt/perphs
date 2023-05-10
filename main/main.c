// main
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
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

/* pins:
TX 17
RX 16
DAC0 25
DAC1 26
Pulse0 4
Pulse1 ?
Direction0 32
Direction1 33
ADC0 34: ADC1-Channel6
ADC1 35: ADC1-Channel7
PWM0-0 27
PWM0-1 14
PWM1-0 12
PWM1-1 13
PWM2-0 18
PWM2-1 19
Ball switch 23
Feed button 36
Osc button ?
Inhibit 39
SDA 21
SDC 22
*/

#define GPIO_DIRECTION_BOTTOM 32
#define GPIO_DIRECTION_TOP 33
#define NUMBER_OF_WHEELS 2
static const uint8_t GPIO_WHEEL_DIRECTION[NUMBER_OF_WHEELS]= {GPIO_DIRECTION_BOTTOM, GPIO_DIRECTION_TOP};
static const uint8_t WHEEL_DAC[NUMBER_OF_WHEELS]= {DAC_CHANNEL_1, DAC_CHANNEL_1};

#define GPIO_MESSAGING_TXD_PIN 17
#define GPIO_MESSAGING_RXD_PIN 16
#define MESSAGING_UART_PORT_NUM 2
#define BUF_SIZE (128) //a buffer small than this causes an exception on uart_driver_install
#define STATUS_SIZE (12) //includes message_type byte
#define COMMAND_SIZE (9)

#define TASK_STACK_SIZE  2048

enum WHEEL_ENUM {BOTTOM, TOP};
enum POSITION_ENUM {ROTARY, ELEVATOR};

uint16_t rpm[2];

static void uart_task(void *arg)
{
    static const char *TAG = "UART";

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
    ESP_ERROR_CHECK(uart_set_pin(MESSAGING_UART_PORT_NUM, GPIO_MESSAGING_TXD_PIN, 
        GPIO_MESSAGING_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
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
        .atten = ADC_ATTEN_DB_11,  // 150 mV ~ 2450 mV measurement range
        .bitwidth = ADC_BITWIDTH_9,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_6, &chan_cfg)); // GPIO34
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_7, &chan_cfg)); // GPIO35
    int adc_chan_value[2]; //temporary storage for value returned from adc_read

    // initialize the DAC
    dac_output_enable(DAC_CHANNEL_1);
    dac_output_enable(DAC_CHANNEL_2);
    //approx 0.78 of VDD_A voltage (VDD * 200 / 255). For VDD_A 3.3V, this is 2.59V
    dac_output_voltage(DAC_CHANNEL_1, 0);
    dac_output_voltage(DAC_CHANNEL_2, 0);

    // initialize the GPIO outputs: 
    gpio_config_t io_conf = {};
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1ULL<<32) | (1ULL<<33));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    uint8_t *status = (uint8_t *) malloc(STATUS_SIZE);
    memset(status,0,STATUS_SIZE);

    //status 1&2: ADC0; 2&3: ADC1, 4&5:RPM0, 5&6:RPM1, 7: carousel bits; 8: temperature1; 9: tempature2
    status[0]= STATUS;
    //test data:
    // status[1]= DATA_BIT | 0x04; //0x0123
    // status[2]= DATA_BIT | 0x23;
    // status[3]= DATA_BIT | 0x3F; //0xFED
    // status[4]= DATA_BIT | 0x2D;

    static uint32_t print_len_gt_1_count=0;
    static uint32_t print_unrec_count=0;
    static uint32_t print_set_perphs_count=0;
    static uint32_t get_status_count=0;
    static uint16_t dac_original[NUMBER_OF_WHEELS];
    static uint8_t  wheel_direction[NUMBER_OF_WHEELS];
    static uint16_t dac_final[NUMBER_OF_WHEELS];
     
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
                //NOTE: these are the raw values.  Conversion to a floating point voltage can be done at the RPi
                //  using this equation: Vout = Dout * Vmax / Dmax where vMax is 2450 mV and Dmax is 4096
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
                for (int i=0; i < NUMBER_OF_WHEELS; i++)
                {
                    uint16_t dac_setting= ((data[0+(i*2)] & DATA_MASK) << 6) | (data[1+(i*2)] & DATA_MASK);
                    // if negative 12-bit number, then 2's complement and drive the direction signal (1 if positive, 0 if negative)
                    uint8_t positive_direction= (dac_setting & 0x0800) ? 0 : 1;
                    dac_original[i]= dac_setting;
                    if (!positive_direction)
                       dac_setting= ~(dac_setting | 0xF000)+1; //sign extend 12 to 16 bit; then do 2's complement
                    gpio_set_level(GPIO_WHEEL_DIRECTION[i], positive_direction);
                    dac_output_voltage(WHEEL_DAC[i], dac_setting>>3); //convert from 11 bit resolution to 8 bit
                    // for debug; printf
                    wheel_direction[i]= positive_direction;
                    dac_final[i]= dac_setting;
                }
                // if (print_set_perphs_count < 2)
                    ESP_LOGI(TAG, "Received SET_PERPHS: length=%d dac[0]=%u (0x%04x) dac[1]=%u (0x%04x) dir[0]=%u dir[1]=%u final[0]=0x%04x final[1]=0x%04x", 
                        len_command, dac_original[0], dac_original[0], dac_original[1], dac_original[1],
                        wheel_direction[0], wheel_direction[1], dac_final[0], dac_final[1]);
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

static void rpms_task(void *arg)
{
    static const char *TAG = "RPMS";
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    static TickType_t xDelayPeriod = pdMS_TO_TICKS(10);  //read counters every N milliseconds 
    TickType_t xLastExecutionTime= xTaskGetTickCount();
    TickType_t xLastReadCountTime= xLastExecutionTime;
    static uint32_t read_count=0;

    while(true)
	{
        read_count++;
        if ((read_count % 2) == 0)
        {
            ESP_LOGD(TAG, "read_count=%lu lastReadMillis=%lu", read_count, pdTICKS_TO_MS(xTaskGetTickCount()-xLastReadCountTime));
            xLastReadCountTime= xTaskGetTickCount();
        }
        //read pulse_counter, get time, calculate rpms

		vTaskDelayUntil( &xLastExecutionTime, 100/xDelayPeriod );
    }
}


void app_main(void)
{
    static const char *TAG = "MAIN";
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    int rc= xTaskCreate(uart_task, "uart_task", TASK_STACK_SIZE, NULL, 10, NULL); //10 is the priority
    if (rc != pdPASS)
        ESP_LOGE(TAG, "Fail: create uart_task: rc=%d", rc);
    else
        ESP_LOGD(TAG, "OK: created uart_task");

    xTaskCreate(rpms_task, "rpms_task", TASK_STACK_SIZE, NULL, 11, NULL);
/*
    the following things are in the freeRTOS tutorial, but it appears that esp-idf does then automatically:
    vTaskStartScheduler();
    for( ;; ); //if this is in the code, then watchdog timeouts occur
*/
}
