#include "driver/i2c.h"
#include "esp32-bmp280/BMP280.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// BMP280 Configuration
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_PORT I2C_NUM_0

BMP280 bmp;  // Use BMP280 object from the ESP32-compatible BMP280 library

// LoRa Configuration
#define LORA_TX_PIN 14
#define LORA_RX_PIN 13

// Built-in LED Pin
#define LED_PIN 2

// FreeRTOS Queues
QueueHandle_t tempQueue;
QueueHandle_t cmdQueue;

// I2C initialization
void i2c_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000  // 100kHz I2C clock
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode);
}

// Temperature Sensing Task
void tempSensingTask(void *pvParameters) {
    float temperature;
    while (true) {
        temperature = bmp.readTemperature();  // Read temperature from BMP280
        xQueueSend(tempQueue, &temperature, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(500)); // 500 ms
    }
}

// LoRa Communication Task
void loraCommTask(void *pvParameters) {
    float temperature;
    char command[10];
    while (true) {
        if (xQueueReceive(tempQueue, &temperature, portMAX_DELAY)) {
            // Send temperature over LoRa
            Serial2.printf("TEMP:%.2f\n", temperature);

            // Check for incoming command
            if (Serial2.available()) {
                int len = Serial2.readBytesUntil('\n', command, sizeof(command));
                command[len] = '\0'; // Null-terminate
                xQueueSend(cmdQueue, command, portMAX_DELAY);
            }
        }
    }
}

// LED Control Task
void ledControlTask(void *pvParameters) {
    char command[10];
    while (true) {
        if (xQueueReceive(cmdQueue, &command, portMAX_DELAY)) {
            if (strcmp(command, "ON") == 0) {
                digitalWrite(LED_PIN, HIGH);
            } else if (strcmp(command, "OFF") == 0) {
                digitalWrite(LED_PIN, LOW);
            }
        }
    }
}

// Setup
void setup() {
    // Initialize Serial for debugging and LoRa
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);

    // Initialize I2C
    i2c_init();

    // Initialize BMP280
    if (!bmp.begin(I2C_PORT, 0x76)) {  // Initialize BMP280 at I2C address 0x76
        Serial.println("Could not find BMP280 sensor!");
        while (1);  // Halt if BMP280 initialization fails
    }

    // Initialize LED pin
    pinMode(LED_PIN, OUTPUT);

    // Create FreeRTOS Queues
    tempQueue = xQueueCreate(10, sizeof(float));
    cmdQueue = xQueueCreate(10, sizeof(char[10]));

    // Create FreeRTOS Tasks
    xTaskCreate(tempSensingTask, "Temperature Sensing Task", 2048, NULL, 1, NULL);
    xTaskCreate(loraCommTask, "LoRa Communication Task", 4096, NULL, 1, NULL);
    xTaskCreate(ledControlTask, "LED Control Task", 2048, NULL, 1, NULL);
}

// Loop
void loop() {
    // FreeRTOS tasks handle everything
}
