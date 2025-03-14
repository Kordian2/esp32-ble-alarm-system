#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "hal/gpio_types.h"
#include "ssd1306.h"
//#include "font8x8_basic.h"

#define tag "SSD1306"
#define GPIO_BUTTON GPIO_NUM_4  // Numer pinu przycisku
#define GPIO_PIR GPIO_NUM_18     // Numer pinu dla czujnika PIR
#define DEBOUNCE_TIME_MS 200    // Czas odbicia w milisekundach
#define GPIO_LED GPIO_NUM_2     //Numer pinu dla wbudowanej diody LED
SSD1306_t dev;
/****************************************************************BLUETOOTH CONF*************************************************************/
char *TAG = "BLE-Server";
uint8_t ble_addr_type;
struct ble_gap_adv_params adv_params;
bool status = false;
void ble_app_advertise(void);
bool alarm_state = false;
// Write data to ESP32 defined as server
static int device_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // printf("Data from the client: %.*s\n", ctxt->om->om_len, ctxt->om->om_data);
    char *data = (char *)ctxt->om->om_data;
    printf("%d %s\n", strcmp(data, (char *)"LIGHT ON") == 0, data);
    if (strcmp(data, (char *)"ALARM ON\0") == 0)
    {
        alarm_state = true;
        printf("ALARM ON\n");
        ssd1306_clear_screen(&dev, false);
        ssd1306_display_text(&dev, 0, " Alarm ON", 9, false);
        //Czujnik PIR
        int pir_value = gpio_get_level(GPIO_PIR);  // Odczytaj stan czujnika PIR
            if (pir_value == 1) {
                printf("Ruch wykryty!\n");
                ssd1306_display_text(&dev, 0, "Ruch wykryty!", 15, false);
                gpio_set_level(GPIO_LED,1);
            } else {
                printf("Brak ruchu.\n");
                gpio_set_level(GPIO_LED,0);
            }
    }
    else if (strcmp(data, (char *)"ALARM OFF\0") == 0)
    {
        alarm_state = false;
        printf("ALARM OFF\n");
        ssd1306_clear_screen(&dev, false);
        ssd1306_display_text(&dev, 0, " Alarm OFF", 10, false);
        gpio_set_level(GPIO_LED,0);
    }
    else if (strcmp(data, (char *)"LED ON\0") == 0)
    {
        printf("LED ON\n");
        gpio_set_level(GPIO_NUM_4, 1);
    }
    else if (strcmp(data, (char *)"LED OFF\0") == 0)
    {
        printf("LED OFF\n");
        gpio_set_level(GPIO_NUM_4, 0);
    }
    
    memset(data, 0, strlen(data));
    return 0;
}
// Read data from ESP32 defined as server
static int device_read(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    os_mbuf_append(ctxt->om, "Data from the server", strlen("Data from the server"));
    return 0;
}
static int device_read2(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    os_mbuf_append(ctxt->om, "Data2", strlen("Data2"));
    return 0;
}

// UUID - Universal Unique Identifier
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(0x180), // Define UUID for device type
     .characteristics = (struct ble_gatt_chr_def[]){
         {.uuid = BLE_UUID16_DECLARE(0x1234), // Define UUID for reading  FEF4
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_read},
         {.uuid = BLE_UUID16_DECLARE(0xABCD), // Define UUID for writing   DEAD
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_write},
         {.uuid = BLE_UUID16_DECLARE(0xABCD), // Define UUID for writing   DEAD
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_read2},
         {0}}},
    {0}};
// BLE event handling
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    // Advertise if connected
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        break;
    // Advertise again after completion of the event
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT DISCONNECTED");
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "BLE GAP EVENT");
        ble_app_advertise();
        break;
    default:
        break;
    }
    return 0;
}
// Define the BLE connection
void ble_app_advertise(void)
{
    // GAP - device name definition
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));    //All fields data set to zero
    device_name = ble_svc_gap_device_name(); // Read the BLE device name
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);
    memset(&adv_params, 0, sizeof(adv_params));
    
}
// The application
void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type); 
    ble_app_advertise();                     // Define the BLE connection
}
// The infinite task
void host_task(void *param)
{
    nimble_port_run(); // This function will return only when nimble_port_stop() is executed
}
void connect_ble(void)
{
    nvs_flash_init(); // 1 - Initialize NVS flash using
    // esp_nimble_hci_and_controller_init();      // 2 - Initialize ESP controller
    nimble_port_init();                        // 3 - Initialize the host stack
    ble_svc_gap_device_name_set("BLE-Server"); // 4 - Initialize NimBLE configuration - server name
    ble_svc_gap_init();                        // 4 - Initialize NimBLE configuration - gap service
    ble_svc_gatt_init();                       // 4 - Initialize NimBLE configuration - gatt service
    ble_gatts_count_cfg(gatt_svcs);            // 4 - Initialize NimBLE configuration - config gatt services
    ble_gatts_add_svcs(gatt_svcs);             // 4 - Initialize NimBLE configuration - queues gatt services.
    ble_hs_cfg.sync_cb = ble_app_on_sync;      // 5 - Initialize application
    nimble_port_freertos_init(host_task);      // 6 - Run the thread
}
void boot_creds_clear(void *param)
{
    // printf("%lld\n", n - m);
    int64_t m = esp_timer_get_time();
    while (1)
    {
        if (!gpio_get_level(GPIO_NUM_0))
        {
            int64_t n = esp_timer_get_time();
            if ((n - m) / 1000 >= 2000)
            {
                ESP_LOGI("BOOT BUTTON:", "Button Pressed FOR 3 SECOND\n");
                adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
                adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable
                ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
                status = true;
                vTaskDelay(100);
                m = esp_timer_get_time();
            }
        }
        else
        {
            m = esp_timer_get_time();
        }
        vTaskDelay(10);
        if (status)
        {
            // ESP_LOGI("GAP", "BLE GAP status");
            adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
            adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable
            ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
        }
    }
}

/******************************************************************************************************************************************/
// Queue for communication between ISR and main task
QueueHandle_t button_queue;

static uint32_t last_isr_time = 0;  // Last time the ISR was triggered

// ISR - button interrupt service routine
void IRAM_ATTR button_isr_handler(void* arg) {
    uint32_t current_time = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;

    // Check if the time since the last interrupt is greater than DEBOUNCE_TIME_MS
    if ((current_time - last_isr_time) > DEBOUNCE_TIME_MS) {
        bool button_pressed = true;
        xQueueSendFromISR(button_queue, &button_pressed, NULL);
        last_isr_time = current_time;  // Save the time of the current interrupt
    }
}

// Function to initialize the PIR sensor GPIO
void setup_pir_sensor() {
    gpio_set_direction(GPIO_PIR, GPIO_MODE_INPUT);    // Set pin as input
    gpio_set_pull_mode(GPIO_PIR, GPIO_PULLUP_ONLY);   // High state when sensor does not detect motion
}

// Main task - processes data from the queue
void button_task(void* arg) {
    bool button_state;

    while (1) {
        // Wait for information from the queue
        if (xQueueReceive(button_queue, &button_state, portMAX_DELAY)) {
            if (button_state) {
                // Toggle alarm state
                alarm_state = !alarm_state;

                // Display the state on the OLED screen
                ssd1306_clear_screen(&dev, false);
                if (alarm_state) {
                    printf("Alarm armed\n");
                    ssd1306_display_text(&dev, 0, " Alarm ON", 9, false);
                } else {
                    printf("Alarm disarmed\n");
                    ssd1306_display_text(&dev, 0, " Alarm OFF", 10, false);
                }
            }
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);  // Short delay
    }
}

// Task to read the state of the PIR sensor
void pir_task(void* arg) {
    while (1) {
        if (alarm_state) {
            // If the alarm is armed, monitor the PIR sensor
            int pir_value = gpio_get_level(GPIO_PIR);  // Read PIR sensor state
            if (pir_value == 1) {
                ssd1306_display_text(&dev, 0, "Motion detected!", 15, false);
                gpio_set_level(GPIO_LED, 1);
            } else {
                gpio_set_level(GPIO_LED, 0);
            }
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);  // 500ms delay before the next read
    }
}

void app_main(void) {
    // Initialize GPIO for the button
    gpio_set_direction(GPIO_BUTTON, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_BUTTON, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(GPIO_BUTTON, GPIO_INTR_POSEDGE);

    // Initialize LED
    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);

    // Initialize SSD1306 display
    ESP_LOGI(tag, "INTERFACE is i2c");
    ESP_LOGI(tag, "CONFIG_SDA_GPIO=%d", CONFIG_SDA_GPIO);
    ESP_LOGI(tag, "CONFIG_SCL_GPIO=%d", CONFIG_SCL_GPIO);
    ESP_LOGI(tag, "CONFIG_RESET_GPIO=%d", CONFIG_RESET_GPIO);
    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
    ESP_LOGI(tag, "Panel is 128x64");
    ssd1306_init(&dev, 128, 64);
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);
    ssd1306_display_text(&dev, 0, " System ready", 13, false);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Install GPIO interrupt service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);

    // Initialize queue
    button_queue = xQueueCreate(10, sizeof(bool));

    // Add button interrupt handler
    gpio_isr_handler_add(GPIO_BUTTON, button_isr_handler, NULL);

    // Create main task to handle button input
    xTaskCreate(button_task, "Button Task", 2048, NULL, 10, NULL);

    // Initialize PIR sensor
    setup_pir_sensor();

    // Create task to read PIR sensor state
    xTaskCreate(pir_task, "PIR Task", 2048, NULL, 10, NULL);

    connect_ble(); // Initialize BLE connection (assuming implementation elsewhere)
    xTaskCreate(boot_creds_clear, "boot_creds_clear", 2048, NULL, 5, NULL);

    printf("System ready. Press the button to arm or disarm the alarm.\n");
}
	



	
  	

	
