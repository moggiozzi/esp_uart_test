/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "soc/uart_reg.h"
#include "esp_log.h"

/**
 * This is an example which echos any data it receives on UART1 back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: UART1
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below
 */

#define ECHO_TEST_TXD    (19)
#define ECHO_TEST_RXD    (21)
#define ECHO_TEST_PIN_DE (22)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

#define RX_BUF_SIZE (256)
#define TX_BUF_SIZE (256)

static QueueHandle_t xMbUartQueue;
static TaskHandle_t  xMbTaskHandle;

#define DBG_TAG "uart_test"

#define MB_SERIAL_BUF_SIZE          (256)
uint16_t mblength = 0;
uint8_t mbbuffer[2*MB_SERIAL_BUF_SIZE];

#define mbuart_dbg_printf(fmt,...) ESP_LOGD("mbuart", fmt, ##__VA_ARGS__)

#define MB_SERIAL_TX_TOUT_TICKS     pdMS_TO_TICKS(100) // timeout for transmission

#define UART_TIMEOT_EVENT (UART_EVENT_MAX+1)
int8_t  mbrtuTmr_init(uint32_t intercharTime);
void  mbrtuTmr_startIntercharTimer(void);
portBASE_TYPE uart_addTimeoutEventFromIsr(void)
{
    portBASE_TYPE hPTaskAwoken = 0;

    uart_event_t xEvent = {.type = UART_TIMEOT_EVENT, .size = 0};
    xQueueSendFromISR(xMbUartQueue, (void * )&xEvent, &hPTaskAwoken);

    return  hPTaskAwoken;
}
int8_t uart_send(uint8_t * pData, uint16_t dataLength)
{
    int16_t length = uart_write_bytes(UART_NUM_1, (char *)pData, dataLength);
    if (length < 0)
    {
        return -1;
    }
    if (length != dataLength)
    {
        return -1;
    }
    //delayMksActive(150); // with delay OK
    //vTaskDelay(pdMS_TO_TICKS(20));
    // Waits while UART sending the packet
    esp_err_t xTxStatus = uart_wait_tx_done(UART_NUM_1, MB_SERIAL_TX_TOUT_TICKS);
    return 0;
}
void usartClientSetModeTx(void) {
	ESP_LOGI(DBG_TAG, "setTx");
	gpio_set_level(ECHO_TEST_PIN_DE, false);
}
void usartClientSetModeRx(void) {
	ESP_LOGI(DBG_TAG, "setRx");
	gpio_set_level(ECHO_TEST_PIN_DE, true);
}
static void rxPoll(size_t xEventSize)
{
	if (xEventSize > 0) {
		// Get received packet into Rx buffer
		int len = xEventSize;
		if (mblength < MB_SERIAL_BUF_SIZE) {
			mblength += uart_read_bytes(UART_NUM_1, &mbbuffer[mblength], MB_SERIAL_BUF_SIZE, 0);
		} else {
			mblength = 0;
		}
	}
}
// UART receive event task
static void vUartTask(void* pvParameters)
{
    uart_event_t xEvent;
    int event;
    usartClientSetModeRx();
    for(;;) {
//        ESP_LOGI(DBG_TAG, "MB_uart queue msgs: %d", uxQueueMessagesWaiting(xMbUartQueue));
        if (xQueueReceive(xMbUartQueue, (void*)&xEvent, portMAX_DELAY) == pdTRUE) { // portMAX_DELAY
            event = xEvent.type;
            ESP_LOGI(DBG_TAG, "event:%d", event);
            switch(event) {
                //Event of UART receiving data
                case UART_DATA:
                    ESP_LOGI(DBG_TAG,"Receive data, len: %d.", xEvent.size);
                    // Read received data and send it to modbus stack
                    rxPoll(xEvent.size);
                    mbrtuTmr_startIntercharTimer();
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    //ESP_LOGI(DBG_TAG, "hw fifo overflow.");
//                    xQueueReset(xMbUartQueue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    //ESP_LOGI(DBG_TAG, "ring buffer full.");
//                    xQueueReset(xMbUartQueue);
                    uart_flush_input(UART_NUM_1);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                	mbuart_dbg_printf("uart rx break.");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                	mbuart_dbg_printf("uart parity error.");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                	mbuart_dbg_printf("uart frame error.");
                    break;
                case UART_TIMEOT_EVENT:
                	mbuart_dbg_printf("uart timeout.");
                    // stop receiving
                    usartClientSetModeTx();
					uart_send(mbbuffer, mblength);
                    //start receiving
					mblength = 0;
                    usartClientSetModeRx();
                    break;
                default:
                	mbuart_dbg_printf("uart event type: %d.", xEvent.type);
                    break;
            }
        }
    }
    vTaskDelete(NULL);
}
static void echo_init(void)
{
    esp_err_t err;
	gpio_config_t io_conf;
	//disable interrupt
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
	//bit mask of the pins that you want to set
	io_conf.pin_bit_mask = ((uint64_t)1 << ECHO_TEST_PIN_DE);
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//disable pull-up mode
	io_conf.pull_up_en = 0;
	//configure GPIO with the given settings
	err = gpio_config(&io_conf);
	if (err != ESP_OK) {
		ESP_LOGE(DBG_TAG, "Config GPIO_%d error %d", ECHO_TEST_PIN_DE, err);
	} else
		ESP_LOGD(DBG_TAG, "Init DE OK");

	int baudRate = 115200;
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = baudRate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 2,
        //.source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE, TX_BUF_SIZE, 256, &xMbUartQueue, ESP_INTR_FLAG_LEVEL3);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);

    uart_intr_config_t uart_intr = {
        .intr_enable_mask = UART_RXFIFO_FULL_INT_ENA_M
                            | UART_RXFIFO_TOUT_INT_ENA_M
                            | UART_FRM_ERR_INT_ENA_M
                            | UART_RXFIFO_OVF_INT_ENA_M
                            | UART_BRK_DET_INT_ENA_M
                            | UART_PARITY_ERR_INT_ENA_M,
        .rxfifo_full_thresh = 32,
        .rx_timeout_thresh = 2,
        .txfifo_empty_intr_thresh = 10
    };
    uart_intr_config(UART_NUM_1, &uart_intr);

    BaseType_t xStatus = xTaskCreate(vUartTask, "uart_queue_task", 4*1024,
                                        NULL, 10, &xMbTaskHandle);
    if (xStatus != pdPASS) {
        vTaskDelete(xMbTaskHandle);
    } else {
        //vTaskSuspend(xMbTaskHandle); // Suspend serial task while stack is not started
    }

    uint32_t uartFifoFillTime = (1000000ul * 11ul * 32ul / baudRate);
    	if (baudRate > 19200) {
    		// fixed: 1750us
    		mbrtuTmr_init(1750 + uartFifoFillTime);
    	} else {
    		// 3.5 char times
    		mbrtuTmr_init(
    				(1000000ul * 11ul * 7ul) / (2ul * baudRate) + uartFifoFillTime);
    	}

}

void app_main(void)
{
    //xTaskCreate(echo_task, "uart_echo_task", 4*1024, NULL, 10, NULL);
	echo_init();
}
