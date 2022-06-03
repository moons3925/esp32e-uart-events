#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "uart_events";

// (1)Tera Termで送受信するUARTのピン番号を指定する
#define CONFIG_EXAMPLE_UART_TXD 23 // (1)
#define CONFIG_EXAMPLE_UART_RXD 22 // (1)

// (2)UART1を指定
#define EX_UART_NUM UART_NUM_1 // (2)

// (3)バッファのバイト数
#define BUF_SIZE (1024) // (3)
#define RD_BUF_SIZE (BUF_SIZE)
// (4)キューのハンドル
static QueueHandle_t uart0_queue;

static void uart_event_task(void *pvParameters)
{
    // (5)イベント構造体
    uart_event_t event;

    // (6)通信用バッファの確保
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for(;;) {
        // (7)イベントの待機
        // (7)第3引数はタイムアウト時間の指定で、portMAX_DELAYは無制限の待ち
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch(event.type) {
                // (8)受信イベント
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    ESP_LOGI(TAG, "[DATA EVT]:");
                    uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);
                    break;
                // (9)FiFoのオーバーフロー
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                // (10)バッファが一杯
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                // (11)受信のブレーク信号
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                // (12)パリティエラー
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                // (13)フレーミングエラー
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    // (14)設定用の構造体
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // (15)ドライバーのインストール
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);

    // (16)パラメーターの設定    
    uart_param_config(EX_UART_NUM, &uart_config);

    // (17)UART1として使うピンの指定
    uart_set_pin(EX_UART_NUM, CONFIG_EXAMPLE_UART_TXD, CONFIG_EXAMPLE_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // (18)イベント用スレッドの生成
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

}
