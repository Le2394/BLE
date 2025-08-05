#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "esp_log.h"
#include "esp_bt.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "esp_adc/adc_oneshot.h"

static const char *TAG = "BLE_NOTIFY";

static uint16_t notify_val_handle;
uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
adc_oneshot_unit_handle_t adc_handle;

// Dummy access callback
static int dummy_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return 0; // BLE_ATT_ERR_SUCCESS
}
static void ble_app_on_sync(void);
// UUID declarations
static const ble_uuid128_t service_uuid = BLE_UUID128_INIT(
    0xf0, 0xde, 0xbc, 0x9a, 0x12, 0x34, 0x12, 0x34,
    0x34, 0x56, 0x34, 0x12, 0x78, 0x12, 0x78, 0x56
);
static const ble_uuid128_t notify_char_uuid = BLE_UUID128_INIT(
    0xde, 0xad, 0xbe, 0xef,
    0xca, 0xfe,
    0xba, 0xbe,
    0x00, 0x00,
    0x00, 0x00, 0x12, 0x34, 0x56, 0x78
);

// GATT service definition
static struct ble_gatt_chr_def gatt_chars[] = {
    {
        .uuid = (ble_uuid_t *)&notify_char_uuid,
        .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ,
        .access_cb = dummy_access_cb,
        .val_handle = &notify_val_handle,
    },
    {0},
};

static struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (ble_uuid_t *)&service_uuid,
        .characteristics = gatt_chars,
    },
    {0},
};

// GAP event handler
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(TAG, "Client connected");
                conn_handle = event->connect.conn_handle;
            } else {
                ESP_LOGI(TAG, "Connection failed; restarting advertising");
                conn_handle = BLE_HS_CONN_HANDLE_NONE;
                ble_app_on_sync();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Client disconnected");
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_app_on_sync();
            break;
        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG, "Client subscribed to characteristic.");
            // Set a flag or start notifications here
            break;
    }
    return 0;
}

// Called when BLE host is synced and ready
static void ble_app_on_sync(void) {
    ESP_LOGI(TAG, "BLE Host synced");

    ble_svc_gap_device_name_set("DJTME_BLETOOTH");

    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)"DJTME_BLETOOTH";
    fields.name_len = strlen("DJTME_BLETOOTH");
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min = 0x0020,
        .itvl_max = 0x0040,
    };

    ble_gap_adv_start(0, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event_cb, NULL);
    ESP_LOGI(TAG, "Advertising started");
}

// BLE host task
static void host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// BLE notify task
void send_ble_notify(void *arg) {
    while (1) {
        int val = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_6, &val));
        ESP_LOGI("FLEX", "Raw ADC Value: %d", val);
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE && notify_val_handle != 0) {
            char msg[64];
            snprintf(msg, sizeof(msg), "%d", val);

            struct os_mbuf *om = ble_hs_mbuf_from_flat(msg, strlen(msg));
            ble_gattc_notify_custom(conn_handle, notify_val_handle, om);
            ESP_LOGI(TAG, "Sent notify: %s", msg);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_6, &chan_cfg));

    //ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());
    nimble_port_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ESP_ERROR_CHECK(ble_gatts_count_cfg(gatt_svcs));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(gatt_svcs));
    ble_hs_cfg.sync_cb = ble_app_on_sync;

    nimble_port_freertos_init(host_task);
    xTaskCreate(send_ble_notify, "ble_notify_task", 4096, NULL, 5, NULL);
}