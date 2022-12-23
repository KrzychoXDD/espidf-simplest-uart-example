#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <strings.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/queue.h"

void app_main(void)
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    const uart_port_t uart_num = UART_NUM_1;

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UART_PIN_NO_CHANGE, 18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    const int uart_buffer_size = 2048;
    QueueHandle_t uart_queue;

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, uart_buffer_size,
                                        uart_buffer_size, 16, &uart_queue, 0));

    uart_enable_pattern_det_baud_intr(uart_num, '\n', 1, 9, 0, 0);
    uart_pattern_queue_reset(uart_num, 16);
    uart_flush(uart_num);

    uint8_t *data = (uint8_t *)malloc(uart_buffer_size);

    uart_event_t event;

    while (1)
    {
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY))
        {
            switch (event.type)
            {
            case UART_PATTERN_DET:
                int pos = uart_pattern_pop_pos(uart_num);
                printf("Pos: %d \n", pos);
                if (pos != -1)
                {
                    uint8_t data[128];
                    int length = uart_read_bytes(uart_num, data, pos + 1, 100 / portTICK_PERIOD_MS);

                    const uint8_t *d = data;
                    while (*d)
                    {
                        if (*d == '$')
                        {
                            printf("Dolar! \n");
                        }
                        else if (*d == ',')
                        {
                            printf("Przecinek! \n");
                        }
                        else if (*d == '*')
                        {
                            printf("Gwiazdka! \n");
                        }
                        else if (*d == '\r')
                        {
                            printf("Znak idz do nowej lini! \n");
                        }
                        d++;
                    }

                    vTaskDelay(500 / portTICK_PERIOD_MS);
                }
                else
                {
                    uart_flush_input(uart_num);
                }
                break;
            case UART_FIFO_OVF:
                printf("HW FIFO Overflow \n");
                uart_flush(uart_num);
                xQueueReset(uart_queue);
                break;
            case UART_BUFFER_FULL:
                printf("Ring Buffer Full \n");
                uart_flush(uart_num);
                xQueueReset(uart_queue);
                break;
            default:
                printf("EVENT DEFAULT\n");
                printf("%d \n", event.type);

                break;
            }
        }
    }

    free(data);
    vTaskDelete(NULL);
}
