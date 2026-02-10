/*
This code
- demonstrates the use of FreeRTOS tasks, queues, and semaphores on an ESP32 microcontroller.
- It blinks an LED connected to GPIO 4 and counts the number of blinks using
    a queue to communicate between two tasks.
- It also sets up an interrupt for a button connected to GPIO 0, which can be used to trigger an action when the button is pressed.

*/
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <freertos/queue.h>
#include "freertos/semphr.h"
#include "esp_task_wdt.h" // Required for Watchdog


#define BLINK_GPIO 4
#define BUTTON_GPIO 0

QueueHandle_t blink_queue;
SemaphoreHandle_t xSemaphore = NULL;

// 2. The Interrupt Service Routine (ISR)
// This function MUST be very fast and have the 'IRAM_ATTR' attribute
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    // Notify the task that the button was pressed
    xSemaphoreGiveFromISR(xSemaphore, NULL);
}

void blink_led() {
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    int count = 0;

    while (1) {
        printf("LED ON!\n");
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        printf("LED OFF!\n");
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(500));

        count++;
        xQueueSend(blink_queue, &count, portMAX_DELAY);
    }
}

void print_hello() {
    int received_count = 0;


    while (1) {
        if (xQueueReceive(blink_queue, &received_count, portMAX_DELAY)) {
            printf("LED Blink count: %d\n", received_count);
        }
        // if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
        //     printf("Interrupt Triggered! Button was pressed.\n");
        // }
        // printf("Hello, World!\n");
        // vTaskDelay(pdMS_TO_TICKS(1000)); // without queue, this task will print every second, but with queue, it will print only when it receives a message from the blink_led task.
    }
}

// Watchdog Task to demonstrate the effect of a "Hanging" task
void greedy_task(void *pvParameter) {
    vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds before "crashing"
    
    printf("\n--- GREEDY TASK STARTING INFINITE LOOP (No Sleep!) ---\n");
    
    // This is the "Hanging" code. No vTaskDelay means the CPU never switches.
    // In FreeRTOS, this "starves" the system.
    while(1) {
        // Doing heavy math or just spinning...
        // The Watchdog is watching this!
    }
}

void app_main(void)
{
    blink_queue = xQueueCreate(10, sizeof(int));
    // Create the semaphore
    // xSemaphore = xSemaphoreCreateBinary();
    // // Configure the Button GPIO
    // gpio_config_t io_conf = {
    //     .intr_type = GPIO_INTR_NEGEDGE,    // Trigger on "Falling Edge" (Pressing down)
    //     .mode = GPIO_MODE_INPUT,           // Input mode
    //     .pin_bit_mask = (1ULL << BUTTON_GPIO),
    //     .pull_up_en = 1,                   // Use internal pull-up resistor
    // };
    // gpio_config(&io_conf);

    // // 4. Install the Interrupt Service
    // gpio_install_isr_service(0);
    // // Hook the specific pin to the handler function
    // gpio_isr_handler_add(BUTTON_GPIO, gpio_isr_handler, NULL);

    // Two task running concurrently
    if (blink_queue != NULL) {
        printf("Queue created successfully\n");
        xTaskCreate(blink_led, "LED_TASK", 2048, NULL, 5, NULL);
        xTaskCreate(print_hello, "HELLO_TASK", 2048, NULL, 5, NULL);
        xTaskCreate(greedy_task, "GREEDY_TASK", 2048, NULL, 5, NULL);
    }

}
