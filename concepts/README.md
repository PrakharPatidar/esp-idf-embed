## FreeRTOS concepts

1. Task Management (Concurrency)
Equivalent to: threading.Thread or multiprocessing.Process
Function	Purpose
xTaskCreate()	Spawns a new parallel worker. Requires a function name, stack size (RAM), and priority.
vTaskDelay()	time.sleep(). Crucial: Always use pdMS_TO_TICKS(ms) to convert time to system ticks.
vTaskDelete(NULL)	Kills the current task. Like sys.exit() but for a specific thread.
2. Queues (Data Passing)
Equivalent to: queue.Queue
Function	Purpose
xQueueCreate()	Defines the "mailbox" size and the type of data (e.g., int, struct).
xQueueSend()	Pushes data into the pipe. Use portMAX_DELAY to wait if the pipe is full.
xQueueReceive()	Pulls data out. It blocks (sleeps) the task until data is ready, saving CPU power.
3. Interrupts & Semaphores (Events)
Equivalent to: asyncio.Event or Hardware Callbacks
Component	Key Rule
IRAM_ATTR	Always add this prefix to ISR functions so they stay in fast RAM.
xSemaphoreGiveFromISR()	The only safe way to send a signal from an interrupt. Never printf in an ISR!
xSemaphoreTake()	Used in a normal task to wait for the "signal" from the interrupt.
4. Hardware Safety (The Watchdog)
Equivalent to: A System Health Monitor
Feature	Key Knowledge
Trigger	Occurs if a high-priority task refuses to vTaskDelay() (CPU Starvation).
Reset	Hardware reboots. Standard variables are wiped; RTC RAM survives.
Check	Use esp_reset_reason() at the start of app_main to see if the WDT was the cause.
5. Hardware Control (GPIO)
Equivalent to: RPi.GPIO
Function	Purpose
gpio_config_t	A struct to set direction, pull-ups, and interrupt types all at once.
gpio_set_level()	Writes 1 (3.3V) or 0 (0V) to a pin.
led_strip_...	Used for the S3's onboard RGB LED (GPIO 48) via the RMT Driver.