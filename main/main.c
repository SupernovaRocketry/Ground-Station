#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#define UART_PORT UART_NUM_0
#define LORA_UART_PORT UART_NUM_2
#define BUF_SIZE 4096

#define LORA_TXD_PIN        17
#define LORA_RXD_PIN        16
#define LORA_AUX_PIN        21
#define LED_PIN             32

#define LORA_UART_BUF_SIZE  512

#define RX_TAG "RECEIVE_TASK"
#define CONFIG_TAG "SETUP"
#define UART_TAG "TRANSMIT_TASK"

QueueHandle_t dataQueue = NULL;

typedef struct // size = 64 bytes
{
    int32_t time;
    uint16_t count;
    uint16_t status;

    float pressure;
    float temperature;
    float bmp_altitude;
    float max_altitude;

    float accel_x;
    float accel_y;
    float accel_z;

    float rotation_x; // Heading
    float rotation_y; // Pitch
    float rotation_z; // Roll

    float latitude;
    float longitude;
    float gps_altitude;
    float voltage;
} data_t;

const uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT
};

esp_err_t lora_uart_init(void)
{
    esp_err_t ret;

    ret = uart_param_config(LORA_UART_PORT, &uart_config);
    if (ret != ESP_OK) {
        //ESP_LOGE(CONFIG_TAG, "Erro na configuração UART LoRa: %d", ret);
        return ret;
    }
    
    ret = uart_set_pin(LORA_UART_PORT, LORA_TXD_PIN, LORA_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        //ESP_LOGE(CONFIG_TAG, "Erro na definição dos pinos UART LoRa: %d", ret);
        return ret;
    }

    ret = uart_driver_install(LORA_UART_PORT, LORA_UART_BUF_SIZE, LORA_UART_BUF_SIZE, 0, NULL, 0);
    if (ret != ESP_OK) {
        //ESP_LOGE(CONFIG_TAG, "Erro na instalação do driver UART LoRa: %d", ret);
        return ret;
    }
    //ESP_LOGI(CONFIG_TAG, "UART LoRa inicializada - TX: %d, RX: %d", LORA_TXD_PIN, LORA_RXD_PIN);
    return ESP_OK;
}

void init_uart() {
    //ESP_LOGI(UART_TAG, "Configurando UART 0");

    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0));

    //ESP_LOGI(UART_TAG, "UART configurada com sucesso.");
}

void lora_receive_task(void *pvParameters) {
    data_t received_data;
    uint8_t rx_buffer[sizeof(data_t)];
    int len;

    while (1) {
        len = uart_read_bytes(LORA_UART_PORT, rx_buffer, sizeof(data_t), pdMS_TO_TICKS(500));
        if (len == sizeof(data_t)) {
            gpio_set_level(LED_PIN, 1);
            memcpy(&received_data, rx_buffer, sizeof(data_t));
            //ESP_LOGI(RX_TAG, "%f, %f, %d", received_data.bmp_altitude, received_data.voltage, received_data.status);

            xQueueSend(dataQueue, &received_data, 0);
        }else{
            //ESP_LOGI(RX_TAG, "Aguardando mensagem...");
            gpio_set_level(LED_PIN, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void uart_send_task(void *pvParameters){
    data_t queueReceived;

    char jsonBuffer[512];

    while(1){
        if(xQueueReceive(dataQueue, (void *)&queueReceived, 0) == pdTRUE)
        {
            int len = snprintf(jsonBuffer, sizeof(jsonBuffer),
                "{"
                "\"time\":%" PRId32 ","
                "\"count\":%u,"
                "\"status\":%u,"
                "\"pressure\":%.2f,"
                "\"temperature\":%.2f,"
                "\"Alt\":%.2f," 
                "\"max_altitude\":%.2f,"
                "\"Acel(x)\":%.2f," 
                "\"Acel(y)\":%.2f," 
                "\"Acel(z)\":%.2f," 
                "\"Gyro(x)\":%.2f," 
                "\"Gyro(y)\":%.2f," 
                "\"Gyro(z)\":%.2f," 
                "\"Lat\":%.8f,"     
                "\"Long\":%.8f,"    
                "\"gps_altitude\":%.2f,"
                "\"Voltage\":%.2f," 
                "\"RSSI\":%d"
                "}\n", 
                queueReceived.time, queueReceived.count, queueReceived.status,
                queueReceived.pressure, queueReceived.temperature, 
                queueReceived.bmp_altitude, queueReceived.max_altitude,
                queueReceived.accel_x, queueReceived.accel_y, queueReceived.accel_z,
                queueReceived.rotation_x, queueReceived.rotation_y, queueReceived.rotation_z,
                queueReceived.latitude, queueReceived.longitude, 
                queueReceived.gps_altitude, queueReceived.voltage,
                -50
            );
        //ESP_LOGI(UART_TAG, "JSON parcial: %s", jsonBuffer);
        uart_write_bytes(UART_PORT, jsonBuffer, strlen(jsonBuffer));
        } else //ESP_LOGE(UART_TAG, "QUEUE EMPTY");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main() {
    init_uart();
    esp_err_t ret = lora_uart_init();
    if (ret != ESP_OK) {
        ESP_LOGE(CONFIG_TAG, "Falha na inicialização da UART LoRa. Erro: %d", ret);
        return;
    }
    
    // Aguardar módulo LoRa estabilizar
    //ESP_LOGI(CONFIG_TAG, "Aguardando módulo LoRa estabilizar...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    int aux_state = gpio_get_level(LORA_AUX_PIN);
    //ESP_LOGI(CONFIG_TAG, "Estado inicial do AUX: %s", aux_state ? "READY" : "BUSY");

    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    for(int i = 0; i<3; i++){
       gpio_set_level(LED_PIN, 1);
       vTaskDelay(pdMS_TO_TICKS(400));
       gpio_set_level(LED_PIN, 0);
    }

    dataQueue = xQueueCreate(10, sizeof(data_t));
    xTaskCreatePinnedToCore(lora_receive_task, "lora_receive_task", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(uart_send_task, "uart_send_task", 4096, NULL, 5, NULL, 1);
}