#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "rom/ets_sys.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "driver/i2c.h"

#include "esp_sleep.h"
#include "esp32/ulp.h"
#include "driver/touch_pad.h"
#include "driver/adc.h"
#include "driver/rtc_io.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"

double vento=0;

#define GPIO_INPUT_IO_0     4
#define GPIO_INPUT_IO_1     5
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 0
#include "ninux_esp32_ota.h"

//static const char *TAG = "MQTT_EXAMPLE";
//const char *TAG = "MQTT_EXAMPLE";

//static EventGroupHandle_t wifi_event_group;
EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

static RTC_DATA_ATTR struct timeval sleep_enter_time;

uint8_t msgData[32];

SemaphoreHandle_t xSemaphore = NULL;


static const gpio_num_t SENSOR_GPIO = 17;
static const uint32_t LOOP_DELAY_MS = 250;
static const int MAX_SENSORS = 8;
static const int RESCAN_INTERVAL = 8;

static xQueueHandle gpio_evt_queue = NULL;
struct timeval now;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}


void anemometer_task(void *pvParameter)
{
   if( xSemaphore != NULL )
   {
       if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
      {
    	uint32_t io_num;
    	uint32_t oldtime=0;
    	uint32_t elapsed=0;
    	float ms=0;
    	float m=0;
    	float elapsed_sec=0;
    	int r=23;
    	int i=0;
    	double mssum=0;
    	extern double vento;
    	for(i=0;i<10;i++) {
    	    if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
    	        if(io_num==GPIO_INPUT_IO_1){
    	    	gettimeofday(&now, NULL);
    	    	int time = now.tv_sec;
    	    	int utime = now.tv_usec;
    	    	uint32_t nowtime=(time*1000000)+utime;
    	    	elapsed=nowtime-oldtime;
    	    	elapsed_sec=(float) elapsed/1000000;
    	    	m=(float) r/100;
    	    	ms=(float) (m*6.28)/elapsed_sec;

    			printf("passati %d usec = %f m/s = %f km/h = %f knots\n",elapsed,ms,(float) (ms*3.6),(float) (ms*1.94384));
    			//printf("passati %d usec = %f m/s = %f km/h = %f knots; m=%f elapsed_sec=%f\n",elapsed,ms,(float) (ms*3.6),(float) (ms*1.94384),m,elapsed_sec);
    			oldtime=nowtime;
    	    	mssum+=ms;
    	    	
    	        }else{
    	        	printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
    	        }
    	    }
    	}
	vento=mssum/10;
	sprintf((char*)msgData,"{\"wind\":%.2f}", vento*10);
	xSemaphoreGive( xSemaphore );
	vTaskDelete(NULL);
        }
    }
}

void init_gpio_for_anemometer(){
    gpio_config_t io_conf;
    io_conf.pull_down_en = 0;
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(anemometer_task, "anemometer_task", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);

}

void sleeppa(int sec)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_EXT1: {
            uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
            if (wakeup_pin_mask != 0) {
                int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                printf("Wake up from GPIO %d\n", pin);
            } else {
                printf("Wake up from GPIO\n");
            }
            break;
        }
        case ESP_SLEEP_WAKEUP_TIMER: {
            printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
            break;
        }
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            printf("Not a deep sleep reset\n");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    const int wakeup_time_sec = sec;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

    const int ext_wakeup_pin_1 = 25;
    const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;
    const int ext_wakeup_pin_2 = 26;
    const uint64_t ext_wakeup_pin_2_mask = 1ULL << ext_wakeup_pin_2;

    printf("Enabling EXT1 wakeup on pins GPIO%d, GPIO%d\n", ext_wakeup_pin_1, ext_wakeup_pin_2);
    esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask | ext_wakeup_pin_2_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    // Isolate GPIO12 pin from external circuits. This is needed for modules
    // which have an external pull-up resistor on GPIO12 (such as ESP32-WROVER)
    // to minimize current consumption.
    rtc_gpio_isolate(GPIO_NUM_12);

    printf("Entering deep sleep\n");
    gettimeofday(&sleep_enter_time, NULL);
    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit();


    //ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MAX_MODEM));
    esp_deep_sleep_start();
}
static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    char mqtt_topic[128];
    bzero(mqtt_topic,sizeof(mqtt_topic));
    sprintf(mqtt_topic,"ambiente/%s/jsondata",CONFIG_MQTT_NODE_NAME);
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_publish(client, mqtt_topic,(const char *) msgData, 0, 1, 0);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            //msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
            //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static int s_retry_num = 0;

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);

            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
        {
            if (s_retry_num < 3) {
            	esp_wifi_connect();
            	xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
                s_retry_num++;
		printf("riprovo la %d volta!!\n",s_retry_num);
	    }else{
		printf("restart!!\n");
		esp_restart();	
	    } 
            break;
	}
        default:
            break;
    }
    return ESP_OK;
}

static void wifi_init(void)
{

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // 1.OTA app partition table has a smaller NVS partition size than the non-OTA
        // partition table. This size mismatch may cause NVS initialization to fail.
        // 2.NVS partition contains data in new format and cannot be recognized by this version of code.
        // If this happens, we erase NVS partition and initialize NVS again.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }


    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    //ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_LOGI(TAG, "start the WIFI SSID:[%s]", CONFIG_WIFI_SSID);
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Waiting for wifi");
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
        .event_handle = mqtt_event_handler,
        // .user_context = (void *)your_context
    };


   while (1) {
   	if( xSemaphore != NULL )
   	{


#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */


    if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE ) { 
      printf("semaforo libero");
      esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
      esp_mqtt_client_start(client);
      
      //vTaskDelay(30 * 1000 / portTICK_PERIOD_MS);
      sleeppa(300);

    }else{
      //printf("semaforo occupato");
    }
    }
  }
}

void app_main()
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);




    //esp_err_t err = nvs_flash_init();
    //if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    //    // 1.OTA app partition table has a smaller NVS partition size than the non-OTA
    //    // partition table. This size mismatch may cause NVS initialization to fail.
    //    // 2.NVS partition contains data in new format and cannot be recognized by this version of code.
    //    // If this happens, we erase NVS partition and initialize NVS again.
    //    ESP_ERROR_CHECK(nvs_flash_erase());
    //    err = nvs_flash_init();
    //}

    wifi_init();
    esp_ota_mark_app_valid_cancel_rollback(); 
    ninux_esp32_ota();


    vSemaphoreCreateBinary( xSemaphore );
    vTaskDelay( 1000 / portTICK_RATE_MS );
    //xTaskCreate( &DHT_task, "DHT_task", 2048, NULL, 5, NULL );
    //i2c_master_init();
    //xTaskCreate(&task_bme280_normal_mode, "bme280_normal_mode",  2048, NULL, 6, NULL);
    init_gpio_for_anemometer();
    //xTaskCreate(anemometer_task, "anemometer_task", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
    vTaskDelay( 3000 / portTICK_RATE_MS );

    mqtt_app_start();
}
