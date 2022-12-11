#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "deps/crc/crc.h"
#include "deps/ringbuffer/ringbuffer.h"

/** @brief TX pin for serial IF to LPC chip */
#define HAL_SERIAL_TXPIN (GPIO_NUM_17)
/** @brief RTX pin for serial IF to LPC chip */
#define HAL_SERIAL_RXPIN (GPIO_NUM_16)
/** @brief UART unit number for serial IF to LPC chip */
#define HAL_SERIAL_UART (UART_NUM_2)
/** @brief UART RTS pin number*/
#define HAL_SERIAL_RTSPIN (GPIO_NUM_4)
#define CBE_RTS_PIN_TX 1
#define CBE_RTS_PIN_RX 0

#define MAX_BUFFSIZE 16384
#define MIN_CHUNKSIZE 512
#define HASH_LEN 32 /* SHA-256 digest length */

const char *SERIAL_OTA_TAG = "SerialOTA";
/*an ota data write buffer ready to write to the flash*/
uint8_t ota_write_data[MAX_BUFFSIZE + 1] = {0};
/*Intermediary buffer*/
uint8_t cbe_serial_ota_data[MAX_BUFFSIZE] = {0};
/*ring buffer cache*/
ring_buffer_t cbe_serial_ota_cache_buffer;

/*Critical error string*/
#define CBE_OTA_ERROR_BUFFER_SIZE 256
char cbe_ota_critical_error_str[CBE_OTA_ERROR_BUFFER_SIZE] = {0};
/*Error print for ota*/
#define CBE_OTA_ERROR_LOG()                                         \
    {                                                               \
        ESP_LOGE(SERIAL_OTA_TAG, "%s", cbe_ota_critical_error_str); \
    }

#define CBE_OTA_ERROR_INSERT(...)            \
    {                                        \
        snprintf(cbe_ota_critical_error_str, \
                 CBE_OTA_ERROR_BUFFER_SIZE,  \
                 __VA_ARGS__);               \
    }

/*Serial OTA header*/
typedef struct
{
    char magic_word[6];
    uint8_t ota_update_mode;
    uint16_t ota_chunk_size;
    uint8_t header_crc; /*has to be last element of the header packet*/
} __attribute__((packed)) cbe_serial_ota_communication_header;

#define CBE_SERIAL_UART_MAGIC_WORD         \
    {                                      \
        0xC0, 0xFF, 0xFE, 0xAA, 0x55, 0x90 \
    }

#define CBE_OTA_UPDATE_MODE_ONLY_NEWER 0x0
#define CBE_OTA_UPDATE_MODE_FORCE_OVERWRITE 0x1

/*Serial OTA single packet header*/
typedef struct
{
    uint16_t packet_id;
    uint8_t packet_crc;
    uint16_t packet_len;
} __attribute__((packed)) cbe_serial_ota_packet_header;

/*Errors define*/
#define CBE_SERIAL_UART_DECODE_ERR_PACK_LEN -1
#define CBE_SERIAL_UART_DECODE_WRONG_CRC -2
#define CBE_SERIAL_UART_DECODE_PACKET_JUST_PRESENT -3
#define CBE_SERIAL_UART_DECODE_PACKET_TOO_SHORT -4
#define CBE_SERIAL_UART_DECODE_PACKET_NO_DATA_TIMEOUT -5
#define CBE_SERIAL_UART_DECODE_PACKET_UNDEF_ERR -6
#define CBE_SERIAL_UART_DECODE_PACKET_LOST -7

#define CBE_SERIAL_UART_DECODE_FINAL_PACKET 0
#define CBE_SERIAL_UART_TIMEOUT_TICKS 250

static void __attribute__((noreturn)) task_fatal_error()
{
    ESP_LOGE(SERIAL_OTA_TAG, "Exiting task due to fatal error...");
    (void)vTaskDelete(NULL);
    vTaskDelay(100);
    esp_restart();
};

/**
 * @brief Listen for cbe_serial_ota_header communication 
 * 
 * @return Received OTA communication header 
 */
cbe_serial_ota_communication_header cbe_serial_ota_wait_header();
int cbe_serial_ota_resend_request();
int cbe_serial_ota_read_packet(uint8_t *buffer, cbe_serial_ota_communication_header communication_header);
void print_sha256(const uint8_t *image_hash, const char *label);
void infinite_loop(void);
esp_err_t uart_setup(void);
/**
 * @brief Check communication header values
 * 
 * @param header pointer to header
 */
void cbe_serial_ota_comm_header_checks(cbe_serial_ota_communication_header *header);
/**
 * @brief In case of error wait for closing packet 
 * 
 */
void cbe_serial_ota_update_error(cbe_serial_ota_communication_header communication_header);
#define CBE_SERIAL_OTA_MAX_NR_ATTEMPS 5
void ota_example_task(void *pvParameter);
/*As an example, init in main*/
void ota_startup();