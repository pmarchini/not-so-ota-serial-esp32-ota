#include "serialOTA.h"

// TODO: Refactor following functions improving readability

serial_ota_communication_header serial_ota_wait_header()
{
    // TODO -> restart timeout
    serial_ota_communication_header t_serial_ota_header;
    uint8_t magic_word[6] = SERIAL_UART_MAGIC_WORD;
    bool crc_flag = false;
    uint8_t crc = 0;
    uint8_t *t_ptr;
    do
    {
        crc = 0;
        uart_flush(HAL_SERIAL_UART);
        uart_read_bytes(HAL_SERIAL_UART, &t_serial_ota_header, sizeof(t_serial_ota_header), 5000);
        if (memcmp(&t_serial_ota_header, magic_word, sizeof(magic_word)) != 0) // TODO -> magic word restart
        {
            ESP_LOGE(SERIAL_OTA_TAG, "recv:");
            ESP_LOG_BUFFER_HEXDUMP(SERIAL_OTA_TAG, &t_serial_ota_header, 6, ESP_LOG_ERROR);
            ESP_LOGE(SERIAL_OTA_TAG, "expected:");
            ESP_LOG_BUFFER_HEXDUMP(SERIAL_OTA_TAG, magic_word, 6, ESP_LOG_ERROR);
            ESP_LOGE(SERIAL_OTA_TAG, "Sync bytes not correct");
            ESP_LOGI(SERIAL_OTA_TAG, "Trying again");
        }
        else
        {
            ESP_LOGI(SERIAL_OTA_TAG, "Sync received");
            t_ptr = &t_serial_ota_header;
            for (int i = 0; i < (sizeof(t_serial_ota_header) - sizeof(t_serial_ota_header.header_crc)); i++)
            {
                CRC8(crc, *t_ptr);
                t_ptr++;
            }
            if (crc == t_serial_ota_header.header_crc)
            {
                crc_flag = true;
                ESP_LOGI(SERIAL_OTA_TAG, "CRC8 correct");
            }
            else
            {
                ESP_LOGW(SERIAL_OTA_TAG, "Wrong CRC8");
            }
        }
    } while (!crc_flag);

    return t_serial_ota_header;
}

int serial_ota_resend_request()
{
    /*needs to set tx to HIGH -> to be checked*/
    gpio_set_level(HAL_SERIAL_RTSPIN, RTS_PIN_TX);
    char bytes[3] = {0x0, 0x1, 0x0};
    uart_write_bytes(HAL_SERIAL_UART, bytes, 3);
    gpio_set_level(HAL_SERIAL_RTSPIN, RTS_PIN_RX);
    return 0;
}

serial_uart_decode_errors serial_ota_read_packet(uint8_t *buffer, serial_ota_communication_header communication_header)
{
    serial_ota_packet_header t_header;
    int data_len = 0;
    unsigned char crc = 0;
    memset(&serial_ota_data, 0, sizeof(serial_ota_data));

    data_len = uart_read_bytes(HAL_SERIAL_UART, serial_ota_data, communication_header.ota_chunk_size + sizeof(serial_ota_packet_header), 5);
    if (data_len < 0)
    {
        ESP_LOGE(SERIAL_OTA_TAG, "Error: UART data read error");
        task_fatal_error();
    }

    if (data_len >= sizeof(serial_ota_packet_header))
    {
        memcpy(&t_header, &serial_ota_data, sizeof(serial_ota_packet_header));
        if (ring_buffer_contains_element(&serial_ota_cache_buffer, t_header.packet_id) == 0)
        {
            if (data_len != (sizeof(serial_ota_packet_header) + t_header.packet_len))
            {
                OTA_ERROR_INSERT("Packet len not correct, expected:%d , found:%d",
                                 (sizeof(serial_ota_packet_header) + t_header.packet_len),
                                 data_len);
                return ERR_PACK_LEN;
            }
            if (!ring_buffer_is_empty(&serial_ota_cache_buffer) &&
                ring_buffer_get_head_element(&serial_ota_cache_buffer) != t_header.packet_id - 1 &&
                t_header.packet_len != 0)
            {
                OTA_ERROR_INSERT("At least one packet is missing, current packet received:%d, last in buffer:%d",
                                 t_header.packet_id,
                                 ring_buffer_get_head_element(&serial_ota_cache_buffer));
                return PACKET_LOST;
            }
            /*Calculate packet crc*/
            for (int index = (sizeof(serial_ota_packet_header)); index < (t_header.packet_len + sizeof(serial_ota_packet_header)); index++)
            {
                CRC8(crc, serial_ota_data[index]);
            }
            if (t_header.packet_crc == crc)
            {
                if (t_header.packet_len == 0)
                {
                    ESP_LOGI(SERIAL_OTA_TAG, "Closing packet received");
                    return FINAL_PACKET;
                }
                /*add to cache the packet id*/
                ring_buffer_queue(&serial_ota_cache_buffer, t_header.packet_id);
                memset(buffer, 0, communication_header.ota_chunk_size + 1);
                memcpy(buffer, &serial_ota_data[sizeof(t_header)], t_header.packet_len);
                return t_header.packet_len;
            }
            else
            {
                OTA_ERROR_INSERT("Packet id:%d, crc expected %d found %d",
                                 t_header.packet_id,
                                 t_header.packet_crc,
                                 crc);
                return WRONG_CRC;
            }
        }
        else
        {
            OTA_ERROR_INSERT("Packet id:%d is just present",
                             t_header.packet_id);
            return PACKET_JUST_PRESENT;
        }
    }
    else if (data_len > 0 && data_len < sizeof(serial_ota_packet_header))
    {
        OTA_ERROR_INSERT("Packet size is too short, expected at least %d, found:%d",
                         sizeof(serial_ota_packet_header),
                         data_len);
        return PACKET_TOO_SHORT;
    }
    else if (data_len == 0)
    {
        OTA_ERROR_INSERT("Data timeout");
        return PACKET_NO_DATA_TIMEOUT;
    }
    else
    {
        OTA_ERROR_INSERT("Undefined error");
        return PACKET_UNDEF_ERR;
    }
}

void print_sha256(const uint8_t *image_hash, const char *label)
{
    char hash_print[HASH_LEN * 2 + 1];
    hash_print[HASH_LEN * 2] = 0;
    for (int i = 0; i < HASH_LEN; ++i)
    {
        sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
    }
    ESP_LOGI(SERIAL_OTA_TAG, "%s: %s", label, hash_print);
}

void infinite_loop(void)
{
    int i = 0;
    ESP_LOGI(SERIAL_OTA_TAG, "Please send data now.");
    while (1)
    {
        ESP_LOGI(SERIAL_OTA_TAG, "Waiting for a new firmware ... %d", ++i);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

esp_err_t uart_setup(void)
{
    esp_err_t ret = ESP_OK;
    const uart_config_t uart_config = {
        .baud_rate = 115200, // TODO -> from header packet, first start -> 9600
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    // update UART config
    ret = uart_param_config(HAL_SERIAL_UART, &uart_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(SERIAL_OTA_TAG, "UART param config failed");
        return ret;
    }

    // set IO pins
    ret = uart_set_pin(HAL_SERIAL_UART, HAL_SERIAL_TXPIN, HAL_SERIAL_RXPIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(SERIAL_OTA_TAG, "UART set pin failed");
        return ret;
    }

    gpio_reset_pin(HAL_SERIAL_RTSPIN);
    gpio_set_direction(HAL_SERIAL_RTSPIN, GPIO_MODE_OUTPUT);
    gpio_set_level(HAL_SERIAL_RTSPIN, 0); // Metto in ricezione

    // Install UART driver with RX and TX buffers
    ret = uart_driver_install(HAL_SERIAL_UART, MAX_BUFFSIZE, MAX_BUFFSIZE, 0, NULL, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(SERIAL_OTA_TAG, "UART driver install failed");
        return ret;
    }

    return ESP_OK;
}

void serial_ota_comm_header_checks(serial_ota_communication_header *header)
{
    if (header->ota_chunk_size > MAX_BUFFSIZE ||
        header->ota_chunk_size < MIN_CHUNKSIZE)
    {
        ESP_LOGE(SERIAL_OTA_TAG, "Chunk size out limits");
        ESP_LOGE(SERIAL_OTA_TAG, "val received :%d", header->ota_chunk_size);
        esp_restart();
    }
}

void serial_ota_update_error(serial_ota_communication_header communication_header)
{
    ESP_LOGE(SERIAL_OTA_TAG, "Update failed, waiting for restart command or timeout");
    int timeout = SERIAL_UART_TIMEOUT_TICKS;
    while (timeout > 0)
    {
        uart_flush(HAL_SERIAL_UART);
        switch (serial_ota_read_packet(ota_write_data, communication_header))
        {
        case PACKET_NO_DATA_TIMEOUT:
            timeout--;
            break;

        case FINAL_PACKET:
            ESP_LOGE(SERIAL_OTA_TAG, "Update last packet arrived, we can safely restart");
            esp_restart();
            break;
        }
#ifdef DEBUG
        OTA_ERROR_LOG()
#endif
    }
    esp_restart();
}

void ota_example_task(void *pvParameter)
{
    esp_err_t err;
    /* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = NULL;

    int serial_ota_timeout = 0;
    int serial_ota_attempts = 0;

    /*Init ring buffer*/
    ring_buffer_init(&serial_ota_cache_buffer);

    ESP_LOGI(SERIAL_OTA_TAG, "Starting Update");

    ESP_LOGI(SERIAL_OTA_TAG, "Setup UART");
    if (uart_setup() != ESP_OK)
    {
        ESP_LOGE(SERIAL_OTA_TAG, "Error initialising UART");
        esp_restart();
    }

    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    if (configured != running)
    {
        ESP_LOGW(SERIAL_OTA_TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
                 configured->address, running->address);
        ESP_LOGW(SERIAL_OTA_TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(SERIAL_OTA_TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             running->type, running->subtype, running->address);

    update_partition = esp_ota_get_next_update_partition(NULL);
    ESP_LOGI(SERIAL_OTA_TAG, "Writing to partition subtype %d at offset 0x%x",
             update_partition->subtype, update_partition->address);
    assert(update_partition != NULL);

    /*Wait for header packet*/
    uart_flush(HAL_SERIAL_UART);
    serial_ota_communication_header ota_header = serial_ota_wait_header();
    ESP_LOGI(SERIAL_OTA_TAG, "Received header :");
    ESP_LOG_BUFFER_HEXDUMP(SERIAL_OTA_TAG, &ota_header, sizeof(ota_header), ESP_LOG_INFO);
    serial_ota_comm_header_checks(&ota_header);

    int binary_file_length = 0;
    /*deal with all receive packet*/
    bool image_header_was_checked = false;
    while (1)
    {
        int data_read = serial_ota_read_packet(ota_write_data, ota_header);
        if (data_read > 0)
        {
            serial_ota_timeout = 0;

            if (image_header_was_checked == false)
            {
                esp_app_desc_t new_app_info;
                if (data_read > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))
                {
                    // check current version with downloading
                    memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                    ESP_LOGI(SERIAL_OTA_TAG, "New firmware version: %s", new_app_info.version);

                    esp_app_desc_t running_app_info;
                    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
                    {
                        ESP_LOGI(SERIAL_OTA_TAG, "Running firmware version: %s", running_app_info.version);
                    }

                    const esp_partition_t *last_invalid_app = esp_ota_get_last_invalid_partition();
                    esp_app_desc_t invalid_app_info;
                    if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK)
                    {
                        ESP_LOGI(SERIAL_OTA_TAG, "Last invalid firmware version: %s", invalid_app_info.version);
                    }

                    // check current version with last invalid partition
                    if (last_invalid_app != NULL)
                    {
                        if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0)
                        {
                            ESP_LOGW(SERIAL_OTA_TAG, "New version is the same as invalid version.");
                            ESP_LOGW(SERIAL_OTA_TAG, "Previously, there was an attempt to launch the firmware with %s version, but it failed.", invalid_app_info.version);
                            ESP_LOGW(SERIAL_OTA_TAG, "The firmware has been rolled back to the previous version.");
                            infinite_loop();
                        }
                    }

                    if (memcmp(new_app_info.version, running_app_info.version, sizeof(new_app_info.version)) == 0)
                    {
                        if (ota_header.ota_update_mode == FORCE_OVERWRITE)
                        {
                            ESP_LOGW(SERIAL_OTA_TAG, "Overwrite mode selected");
                            ESP_LOGW(SERIAL_OTA_TAG, "Current running version is the same as a new, forcing write");
                        }
                        else
                        {
                            ESP_LOGW(SERIAL_OTA_TAG, "Current running version is the same as a new. We will not continue the update.");
                            esp_restart();
                        }
                    }

                    image_header_was_checked = true;

                    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
                    if (err != ESP_OK)
                    {
                        ESP_LOGE(SERIAL_OTA_TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                        task_fatal_error();
                    }
                    ESP_LOGI(SERIAL_OTA_TAG, "esp_ota_begin succeeded");
                }
                else
                {
                    ESP_LOGE(SERIAL_OTA_TAG, "received package is not fit len");
                    task_fatal_error();
                }
            }
            err = esp_ota_write(update_handle, (const void *)ota_write_data, data_read);
            if (err != ESP_OK)
            {
                ESP_LOGE(SERIAL_OTA_TAG, "esp_ota_write error nr:%d", err);
                task_fatal_error();
            }
            binary_file_length += data_read;
            ESP_LOGI(SERIAL_OTA_TAG, "Written image length %d", binary_file_length);
        }
        else if (data_read == 0)
        {
            ESP_LOGI(SERIAL_OTA_TAG, "Connection closed,all data received");
            break;
        }
        else if (data_read < 0)
        {
            OTA_ERROR_LOG()
            switch (data_read)
            {
            case PACKET_NO_DATA_TIMEOUT:
                serial_ota_timeout++;
                if (serial_ota_timeout > SERIAL_UART_TIMEOUT_TICKS)
                {
                    esp_restart();
                }
                break;

            case WRONG_CRC:
                /*TODO -> test this*/
                if (serial_ota_resend_request() < 0 ||
                    serial_ota_attempts > SERIAL_OTA_MAX_NR_ATTEMPS)
                {
                    serial_ota_update_error(ota_header);
                }
                serial_ota_attempts++;
                break;

            case PACKET_LOST:
                serial_ota_update_error(ota_header);
                break;

            case ERR_PACK_LEN:
                if (serial_ota_resend_request() < 0 ||
                    serial_ota_attempts > SERIAL_OTA_MAX_NR_ATTEMPS)
                {
                    serial_ota_update_error(ota_header);
                }
                serial_ota_attempts++;
                serial_ota_update_error(ota_header);
                break;
            }
        }
    }
    ESP_LOGI(SERIAL_OTA_TAG, "Total Write binary data length : %d", binary_file_length);

    if (esp_ota_end(update_handle) != ESP_OK)
    {
        ESP_LOGE(SERIAL_OTA_TAG, "esp_ota_end failed!");
        task_fatal_error();
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK)
    {
        ESP_LOGE(SERIAL_OTA_TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        task_fatal_error();
    }
    ESP_LOGI(SERIAL_OTA_TAG, "Prepare to restart system!");
    esp_restart();
    return;
}

void ota_startup()
{
    uint8_t sha_256[HASH_LEN] = {0};
    esp_partition_t partition;

    // set debug output
    esp_log_level_set("*", ESP_LOG_DEBUG);
    esp_log_level_set(SERIAL_OTA_TAG, ESP_LOG_DEBUG);

    // get sha256 digest for the partition table
    partition.address = ESP_PARTITION_TABLE_OFFSET;
    partition.size = ESP_PARTITION_TABLE_MAX_LEN;
    partition.type = ESP_PARTITION_TYPE_DATA;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for the partition table: ");

    // get sha256 digest for bootloader
    partition.address = ESP_BOOTLOADER_OFFSET;
    partition.size = ESP_PARTITION_TABLE_OFFSET;
    partition.type = ESP_PARTITION_TYPE_APP;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for bootloader: ");

    // get sha256 digest for running partition
    esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
    print_sha256(sha_256, "SHA-256 for current firmware: ");

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK)
    {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
        {
            ESP_LOGI(SERIAL_OTA_TAG, "OTA partition marked valid");
            esp_ota_mark_app_valid_cancel_rollback();
            esp_restart();
        }
    }

    // Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // OTA app partition table has a smaller NVS partition size than the non-OTA
        // partition table. This size mismatch may cause NVS initialization to fail.
        // If this happens, we erase NVS partition and initialize NVS again.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    xTaskCreate(&ota_example_task, "OTA", 8192, NULL, 5, NULL);
}