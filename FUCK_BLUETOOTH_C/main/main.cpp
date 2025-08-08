#include <stdio.h>
#include <string.h>
#include <cmath>
extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "nvs_flash.h"

    #include "esp_log.h"

    #include "nimble/nimble_port.h"
        #include "nimble/nimble_port_freertos.h"
    #include "host/ble_hs.h"
    #include "services/gap/ble_svc_gap.h"
    #include "services/gatt/ble_svc_gatt.h"

    #include "esp_adc/adc_oneshot.h"
}

#include "I2Cdev.h"
#include "ICM42688.h"

#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_SDA_IO      17
#define I2C_MASTER_SCL_IO      18
#define I2C_MASTER_FREQ_HZ     400000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

#define MEDIAN_WINDOW 5
#define OVERSAMPLE_COUNT 8
#define EMA_ALPHA 0.3
#define CHANGE_THRESHOLD 2 
#define NOTIFY_INTERVAL_MS 100

static const char *TAG = "DJTME_BLUETOOTH_FLEX";

extern float q[4];  // Thông báo với compiler rằng biến q tồn tại ở nơi khác
I2Cdev i2c_0(I2C_MASTER_NUM);   // Dùng IDF thay vì Wire
ICM42688 imu(&i2c_0);           // Đối tượng cảm biến
bool clockIn = false;           // Không dùng clock input ngoài
float a12, a22, a31, a32, a33;  // Dự phòng cho ma trận định hướng
float pitch, yaw, roll;         // Góc Euler (có thể dùng để hiển thị)
// ======= THAM SỐ MADGWICK =======
const float pi = 3.14159265358979323846f;
const float GyroMeasError = pi * (40.0f / 180.0f);   // rad/s
const float GyroMeasDrift = pi * (0.0f  / 180.0f);   // rad/s/s
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;
float zeta  = sqrtf(3.0f / 4.0f) * GyroMeasDrift;
// ========== THỜI GIAN & QUATERNION ==========
float deltat = 0.0f;                // khoảng thời gian tích phân (s)
uint64_t lastUpdate = 0;           // thời điểm cập nhật gần nhất (µs)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // quaternion định hướng ban đầu
// ========= CÁC BIẾN HIỆU CHUẨN & ĐỌC DỮ LIỆU =========
float aRes = 0.0f, gRes = 0.0f;                   // Độ phân giải accel / gyro
float accelBias[3] = {0.0f, 0.0f, 0.0f};           // Offset accel
float gyroBias[3]  = {0.0f, 0.0f, 0.0f};           // Offset gyro
int16_t accelDiff[3] = {0}, gyroDiff[3] = {0};     // Kết quả so sánh self-test
float  STratio[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};          // self-test results for the accel and gyro
int16_t raw[7] = {0};                              // raw[0] = temp, raw[1-3] = ax ay az, raw[4-6] = gx gy gz
float ax = 0.0f, ay = 0.0f, az = 0.0f;
float gx = 0.0f, gy = 0.0f, gz = 0.0f;
float Gtemperature = 0.0f;        
static uint16_t imu_val_handle = 0;

uint16_t conn_handle_global = BLE_HS_CONN_HANDLE_NONE;
bool notify_enabled = false;

static uint16_t notify_val_handle;
uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
adc_oneshot_unit_handle_t adc_handle;

int median_buffer[MEDIAN_WINDOW] = {0};
int median_index = 0;
float filtered_value = 0;
int last_notified_value = 0;

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
    ble_svc_gap_device_name_set("DJTME_BLUETOOTH_FLEX");

    struct ble_hs_adv_fields fields = {};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)"DJTME_BLUETOOTH_FLEX";
    fields.name_len = strlen("DJTME_BLUETOOTH_FLEX");
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

void M1(float ax, float ay, float az, float gx, float gy, float gz);

void i2c_master_init() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                                       I2C_MASTER_RX_BUF_DISABLE,
                                       I2C_MASTER_TX_BUF_DISABLE, 0));
}

int median_filter(int new_value) {
    median_buffer[median_index++] = new_value;
    if (median_index >= MEDIAN_WINDOW) median_index = 0;

    int sorted[MEDIAN_WINDOW];
    memcpy(sorted, median_buffer, sizeof(sorted));
    for (int i = 0; i < MEDIAN_WINDOW - 1; i++) {
        for (int j = i + 1; j < MEDIAN_WINDOW; j++) {
            if (sorted[i] > sorted[j]) {
                int tmp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = tmp;
            }
        }
    }
    return sorted[MEDIAN_WINDOW / 2];
}

int read_oversampled_adc(int samples) {
    int sum = 0;
    for (int i = 0; i < samples; i++) {
        int val = 0;
        adc_oneshot_read(adc_handle, ADC_CHANNEL_6, &val);
        sum += val;
        esp_rom_delay_us(100);
    }
    return sum / samples;
}

void send_ble_notify(void *arg) {
    TickType_t lastSendTime = xTaskGetTickCount();

    while (true) {
        int raw = read_oversampled_adc(OVERSAMPLE_COUNT);
        int median_val = median_filter(raw);
        filtered_value = EMA_ALPHA * median_val + (1 - EMA_ALPHA) * filtered_value;
        int rounded_val = (int)(filtered_value + 0.5f);

        TickType_t now = xTaskGetTickCount();
        if ((now - lastSendTime) >= pdMS_TO_TICKS(NOTIFY_INTERVAL_MS)) {
            lastSendTime = now;
            if (abs(rounded_val - last_notified_value) >= CHANGE_THRESHOLD) {
                last_notified_value = rounded_val;

                if (conn_handle != BLE_HS_CONN_HANDLE_NONE && notify_val_handle != 0) {
                    char msg[32];
                    snprintf(msg, sizeof(msg), "ADC:%d", rounded_val);
                    struct os_mbuf *om = ble_hs_mbuf_from_flat(msg, strlen(msg));
                    ble_gattc_notify_custom(conn_handle, notify_val_handle, om);
                    ESP_LOGI(TAG, "Notify: %s", msg);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void imu_init() {
    i2c_master_init();
    vTaskDelay(pdMS_TO_TICKS(100));

    imu.reset();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    uint8_t Ascale = AFS_8G;
    uint8_t Gscale = GFS_2000DPS;
    uint8_t AODR   = AODR_500Hz;
    uint8_t GODR   = GODR_500Hz;
    uint8_t aMode  = aMode_LN;
    uint8_t gMode  = gMode_LN;

    aRes = 4.0f / 32768.0f;
    gRes = 250.0f / 32768.0f;

    imu.selfTest(accelDiff, gyroDiff, STratio);

    aRes = imu.getAres(Ascale);
    gRes = imu.getGres(Gscale);

    imu.init(Ascale, Gscale, AODR, GODR, aMode, gMode, clockIn);

    vTaskDelay(pdMS_TO_TICKS(1000));
    imu.offsetBias(accelBias, gyroBias);
    vTaskDelay(pdMS_TO_TICKS(1000));

    lastUpdate = esp_timer_get_time();
}

void imu_task(void *arg) {
    TickType_t lastSendTime = xTaskGetTickCount();

    while (true) {

        imu.readData(raw);

        ax = raw[1] * aRes - accelBias[0];
        ay = raw[2] * aRes - accelBias[1];
        az = raw[3] * aRes - accelBias[2];
        gx = (raw[4] * gRes - gyroBias[0]) * pi / 180.0f;
        gy = (raw[5] * gRes - gyroBias[1]) * pi / 180.0f;
        gz = (raw[6] * gRes - gyroBias[2]) * pi / 180.0f;

        for (uint8_t i = 0; i < 20; i++) {
            uint64_t Now = esp_timer_get_time();
            deltat = ((Now - lastUpdate) / 1000000.0f);
            lastUpdate = Now;

            M1(ax, ay, az, gx, gy, gz);
        }

        //ESP_LOGI(TAG, "%.4f, %.4f, %.4f, %.4f", q[0], q[1], q[2], q[3]);
        
        // Notify BLE every NOTIFY_INTERVAL_MS
        TickType_t now = xTaskGetTickCount();
        if ((now - lastSendTime) >= pdMS_TO_TICKS(NOTIFY_INTERVAL_MS)) {
            lastSendTime = now;

            if (conn_handle != BLE_HS_CONN_HANDLE_NONE && notify_val_handle != 0) {
                uint8_t data[16];
                memcpy(&data[0],  &q[0], 4);
                memcpy(&data[4],  &q[1], 4);
                memcpy(&data[8],  &q[2], 4);
                memcpy(&data[12], &q[3], 4);

                struct os_mbuf *om = ble_hs_mbuf_from_flat(data, sizeof(data));
                int rc = ble_gattc_notify_custom(conn_handle, notify_val_handle, om);
                if (rc != 0) {
                    ESP_LOGW(TAG, "IMU Notify failed: rc=%d", rc);
                } else {
                    //ESP_LOGI(TAG, "%.3f, %.3f, %.3f, %.3f", q[0], q[1], q[2], q[3]);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

extern "C" void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    imu_init();

    adc_oneshot_unit_init_cfg_t init_cfg = {
        ADC_UNIT_1,
        ADC_RTC_CLK_SRC_DEFAULT,
        ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_6, &chan_cfg));

    nimble_port_init();
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_ERROR_CHECK(ble_gatts_count_cfg(gatt_svcs));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(gatt_svcs));
    ble_hs_cfg.sync_cb = ble_app_on_sync;

    nimble_port_freertos_init(host_task);
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
    xTaskCreate(send_ble_notify, "ble_notify_task", 4096, NULL, 5, NULL);
}

void M1(float ax, float ay, float az, float gyrox, float gyroy, float gyroz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
            float norm;                                               // vector norm
            float f1, f2, f3;                                         // objetive funcyion elements
            float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
            float qDot1, qDot2, qDot3, qDot4;
            float hatDot1, hatDot2, hatDot3, hatDot4;
            float gerrx = 0.0f, gerry = 0.0f, gerrz = 0.0f;
            float gbiasx = 0.0f;
            float gbiasy = 0.0f;
            float gbiasz = 0.0f;

            // Auxiliary variables to avoid repeated arithmetic
            float _halfq1 = 0.5f * q1;
            float _halfq2 = 0.5f * q2;
            float _halfq3 = 0.5f * q3;
            float _halfq4 = 0.5f * q4;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;

            // Normalise accelerometer measurement
            norm = sqrtf(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;
            
            // Compute the objective function and Jacobian
            f1 = _2q2 * q4 - _2q1 * q3 - ax;
            f2 = _2q1 * q2 + _2q3 * q4 - ay;
            f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
            J_11or24 = _2q3;
            J_12or23 = _2q4;
            J_13or22 = _2q1;
            J_14or21 = _2q2;
            J_32 = 2.0f * J_14or21;
            J_33 = 2.0f * J_11or24;
          
            // Compute the gradient (matrix multiplication)
            hatDot1 = J_14or21 * f2 - J_11or24 * f1;
            hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
            hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
            hatDot4 = J_14or21 * f1 + J_11or24 * f2;
            
            // Normalize the gradient
            norm = sqrtf(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
            hatDot1 /= norm;
            hatDot2 /= norm;
            hatDot3 /= norm;
            hatDot4 /= norm;
            
            // Compute estimated gyroscope biases
            gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
            gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
            gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
            
            // Compute and remove gyroscope biases
            gbiasx += gerrx * deltat * zeta;
            gbiasy += gerry * deltat * zeta;
            gbiasz += gerrz * deltat * zeta;
            gyrox -= gbiasx;
            gyroy -= gbiasy;
            gyroz -= gbiasz;
            
            // Compute the quaternion derivative
            qDot1 = -_halfq2 * gyrox - _halfq3 * gyroy - _halfq4 * gyroz;
            qDot2 =  _halfq1 * gyrox + _halfq3 * gyroz - _halfq4 * gyroy;
            qDot3 =  _halfq1 * gyroy - _halfq2 * gyroz + _halfq4 * gyrox;
            qDot4 =  _halfq1 * gyroz + _halfq2 * gyroy - _halfq3 * gyrox;

            // Compute then integrate estimated quaternion derivative
            q1 += (qDot1 -(beta * hatDot1)) * deltat;
            q2 += (qDot2 -(beta * hatDot2)) * deltat;
            q3 += (qDot3 -(beta * hatDot3)) * deltat;
            q4 += (qDot4 -(beta * hatDot4)) * deltat;

            // Normalize the quaternion
            norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;
      }