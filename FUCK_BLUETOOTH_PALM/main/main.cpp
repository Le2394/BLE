#include <stdio.h>
#include <string.h>
#include <cmath>
#include <stdlib.h>
extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "nvs_flash.h"

    #include "driver/i2c.h"
    #include "driver/gpio.h"
    #include "esp_log.h"

    #include "nimble/nimble_port.h"
    #include "nimble/nimble_port_freertos.h"
    #include "host/ble_hs.h"
    #include "services/gap/ble_svc_gap.h"
    #include "services/gatt/ble_svc_gatt.h"

    #include "esp_adc/adc_oneshot.h"
}

#include "I2Cdev.h"
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"

// --- I2C Configuration ---
#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_10
#define ENABLE_PIN GPIO_NUM_9
#define I2C_PORT I2C_NUM_0
#define TCAADDR 0x70
#define NOTIFY_INTERVAL_MS 100
#define NUM_IMUS 6

static const char *TAG = "DJTME_BLUETOOTH_PALM";

static uint16_t imu_val_handle = 0;

uint16_t conn_handle_global = BLE_HS_CONN_HANDLE_NONE;
bool notify_enabled = false;

static uint16_t notify_val_handle;
uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;

MPU6050 mpu[NUM_IMUS];
Quaternion q[NUM_IMUS];
uint16_t packetSize[NUM_IMUS];
uint8_t fifoBuffer[64];

void tca_select(uint8_t channel) {
    if (channel > 7) return;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TCAADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 1 << channel, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "tca_select failed: %s", esp_err_to_name(err));
    }
}

void init_i2c_and_power() {
    gpio_set_direction(ENABLE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(ENABLE_PIN, 1);

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA_PIN;
    conf.scl_io_num = SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    conf.clk_flags = 0;

    i2c_param_config(I2C_PORT, &conf);
    esp_err_t err = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    ESP_LOGI(TAG, "i2c_driver_install: %s", esp_err_to_name(err));
}

void imu_task(void *arg) {
    TickType_t lastSendTime = xTaskGetTickCount();
    static int current_imu = 0;

    while (true) {
        TickType_t now = xTaskGetTickCount();

        tca_select(current_imu);
        uint16_t fifoCount = mpu[current_imu].getFIFOCount();
        uint8_t mpuIntStatus = mpu[current_imu].getIntStatus();

        if (mpuIntStatus & 0x10 || fifoCount > 1024) {
            ESP_LOGW(TAG, "FIFO overflow on IMU #%d", current_imu);
            mpu[current_imu].resetFIFO();
        } else if (fifoCount >= packetSize[current_imu]) {
            while (fifoCount >= packetSize[current_imu]) {
                mpu[current_imu].getFIFOBytes(fifoBuffer, packetSize[current_imu]);
                fifoCount -= packetSize[current_imu];
            }
            mpu[current_imu].dmpGetQuaternion(&q[current_imu], fifoBuffer);
            ESP_LOGI(TAG, "IMU #%d: Quat = %.3f, %.3f, %.3f, %.3f", current_imu, q[current_imu].w, q[current_imu].x, q[current_imu].y, q[current_imu].z);
        }

        if ((now - lastSendTime) >= pdMS_TO_TICKS(NOTIFY_INTERVAL_MS)) {
            lastSendTime = now;

            if (conn_handle != BLE_HS_CONN_HANDLE_NONE && notify_val_handle != 0) {
                uint8_t data[16];
                memcpy(&data[0],  &q[current_imu].w, 4);
                memcpy(&data[4],  &q[current_imu].x, 4);
                memcpy(&data[8],  &q[current_imu].y, 4);
                memcpy(&data[12], &q[current_imu].z, 4);

                struct os_mbuf *om = ble_hs_mbuf_from_flat(data, sizeof(data));
                int rc = ble_gattc_notify_custom(conn_handle, notify_val_handle, om);
                if (rc != 0) {
                    ESP_LOGW(TAG, "Notify failed for IMU #%d: rc=%d", current_imu, rc);
                }
            }
        }

        current_imu = (current_imu + 1) % NUM_IMUS;
        vTaskDelay(pdMS_TO_TICKS(8));
    }
}

bool i2c_ping(uint8_t addr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK;
}

void task_init_mpu(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(500));

    for (int i = 0; i < NUM_IMUS; i++) {
        tca_select(i);
        vTaskDelay(pdMS_TO_TICKS(50));

        ESP_LOGI(TAG, "Pinging IMU #%d...", i);
        if (!i2c_ping(0x68)) {
            ESP_LOGE(TAG, "IMU #%d not responding on I2C!", i);
            continue;
        }

        mpu[i].initialize();
        if (!mpu[i].testConnection()) {
            ESP_LOGE(TAG, "MPU6050 #%d testConnection() failed!", i);
            continue;
        }

        ESP_LOGI(TAG, "MPU6050 #%d initializing DMP...", i);
        if (mpu[i].dmpInitialize() != 0) {
            ESP_LOGE(TAG, "MPU6050 #%d DMP init failed!", i);
            continue;
        }

        mpu[i].CalibrateAccel(6);
        mpu[i].CalibrateGyro(6);
        mpu[i].setDMPEnabled(true);
        mpu[i].setRate(19); // Lower sample rate to ~40Hz
        packetSize[i] = mpu[i].dmpGetFIFOPacketSize();
        mpu[i].resetFIFO();

        ESP_LOGI(TAG, "MPU6050 #%d ready, packetSize=%d", i, packetSize[i]);
    }

    xTaskCreate(&imu_task, "quaternion_reader", 8192, NULL, 5, NULL);
    vTaskDelete(NULL);
}

static int dummy_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return 0;
}

static void ble_app_on_sync(void);

static const ble_uuid128_t service_uuid = BLE_UUID128_INIT(
    0xf0, 0xde, 0xbc, 0x9a, 0x12, 0x34, 0x12, 0x34,
    0x34, 0x56, 0x34, 0x12, 0x78, 0x12, 0x78, 0x56
);
static const ble_uuid128_t notify_char_uuid = BLE_UUID128_INIT(
    0xde, 0xad, 0xbe, 0xef, 0xca, 0xfe, 0xba, 0xbe,
    0x00, 0x00, 0x00, 0x00, 0x12, 0x34, 0x56, 0x78
);

static struct ble_gatt_chr_def gatt_chars[] = {
    {
        (ble_uuid_t *)&notify_char_uuid, // uuid
        dummy_access_cb,                 // access_cb
        nullptr,                         // arg
        nullptr,                         // descriptors
        BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ, // flags
        0,                               // min_key_size
        &notify_val_handle,              // val_handle
        nullptr                          // cpfd
    },
    {}
};

static struct ble_gatt_svc_def gatt_svcs[] = {
    {
        BLE_GATT_SVC_TYPE_PRIMARY,
        (ble_uuid_t *)&service_uuid,
        nullptr,
        gatt_chars
    },
    {}
};

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
            break;
    }
    return 0;
}

static void ble_app_on_sync(void) {
    ESP_LOGI(TAG, "BLE Host synced");
    ble_svc_gap_device_name_set("DJTME_BLUETOOTH_PALM");

    struct ble_hs_adv_fields fields = {};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)"DJTME_BLUETOOTH_PALM";
    fields.name_len = strlen("DJTME_BLUETOOTH_PALM");
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv_params = {
        BLE_GAP_CONN_MODE_UND,
        BLE_GAP_DISC_MODE_GEN,
        0x0020,
        0x0040,
        0, 0, 0
    };

    ble_gap_adv_start(0, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event_cb, NULL);
    ESP_LOGI(TAG, "Advertising started");
}

static void host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

extern "C" void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_LOGI(TAG, "Initializing I2C and power...");
    init_i2c_and_power();
    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_LOGI(TAG, "Starting MPU6050 quaternion task...");

    nimble_port_init();
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_ERROR_CHECK(ble_gatts_count_cfg(gatt_svcs));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(gatt_svcs));
    ble_hs_cfg.sync_cb = ble_app_on_sync;

    nimble_port_freertos_init(host_task);

    xTaskCreate(&task_init_mpu, "mpu_quaternion_init", 8192, NULL, 5, NULL);
}
