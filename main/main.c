#include <stdio.h>
#include <serialOTA.h>

void app_main(void)
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

    xTaskCreatePinnedToCore(
        &ota_example_task, /* Function that implements the task. */
        "OTA",             /* Text name for the task. */
        8192,              /* Stack size in words, not bytes. */
        NULL,              /* Parameter passed into the task. */
        5,                 /* Priority at which the task is created. */
        NULL,              /* Used to pass out the created task's handle. */
        0                  /*Core affinity*/
    );
}