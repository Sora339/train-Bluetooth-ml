#include <Arduino.h>
#include <Wire.h>

// ESP-IDF BLE関連のヘッダー
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"

// I2Cアドレスの定数
#define PRESSURE_ADDRESS 0x48  // HSPPAD143C
#define MPU6050_ADDRESS 0x68   // MPU-6050
#define SDA_PIN 20             // ESP32のデフォルトSDA
#define SCL_PIN 21             // ESP32のデフォルトSCL

static const char* TAG = "BLE5_EXT_ADV";

// 拡張アドバタイズのハンドル定義
#define EXT_ADV_HANDLE     0
#define PERIOD_ADV_HANDLE  0

// センサーデータ構造体
typedef struct {
  float pressure;
  float ax, ay, az;
  float gx, gy, gz;
  unsigned long timestamp;  // 測定時間
} sensor_data_t;

// データバッファの設定
#define MAX_QUEUE_SIZE 8
#define MAX_ENTRIES 8
sensor_data_t dataBuffer[MAX_ENTRIES];
int dataBufferCount = 0;
uint8_t packetId = 0;
const int ledPin = 2;
uint8_t advData[100] = {0};  // アドバタイズデータバッファ

// アドバタイズパラメータ設定を保持
static esp_ble_gap_ext_adv_params_t ext_adv_params_coded;  // C構造体初期化リストの代わりに値を個別に設定

// 定期的アドバタイズパラメータ設定
static esp_ble_gap_periodic_adv_params_t periodic_adv_params;  // C構造体初期化リストの代わりに値を個別に設定

// ランダムアドレス設定
static esp_bd_addr_t rnd_addr = {0xc0, 0xde, 0x52, 0x00, 0x00, 0x00};
// 拡張アドバタイズハンドル
static esp_ble_gap_ext_adv_t ext_adv[1];  // esp_ble_gap_ext_adv_t型の配列として定義

// I2Cレジスタ書き込み
void writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

// 気圧センサーの生データ読み取り
uint32_t readPressure() {
    Wire.beginTransmission(PRESSURE_ADDRESS);
    Wire.write(0x04);  // PRESSURE_REG_POUT
    if (Wire.endTransmission(false) != 0) {
        return 0;
    }

    Wire.requestFrom(PRESSURE_ADDRESS, 3);
    if (Wire.available() < 3) {
        return 0;
    }

    uint32_t msb = Wire.read();
    uint32_t lsb = Wire.read();
    uint32_t xlsb = Wire.read();

    return (msb << 16) | (lsb << 8) | xlsb;
}

// 気圧値をhPaで読み取り
float readPressureHPa() {
    uint32_t rawValue = readPressure();
    float pressurePa = rawValue * 2.0;
    float pressureHPa = pressurePa / 100.0;
    return pressureHPa / 10.0;
}

// 加速度センサー読み取り
void readAccel(float *x, float *y, float *z) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x3B);  // ACCEL_XOUT_H
    if (Wire.endTransmission(false) != 0) {
        *x = *y = *z = 0;
        return;
    }

    Wire.requestFrom(MPU6050_ADDRESS, 6);
    if (Wire.available() < 6) {
        *x = *y = *z = 0;
        return;
    }

    int16_t xRaw = (Wire.read() << 8) | Wire.read();
    int16_t yRaw = (Wire.read() << 8) | Wire.read();
    int16_t zRaw = (Wire.read() << 8) | Wire.read();

    *x = xRaw / 16384.0;
    *y = yRaw / 16384.0;
    *z = zRaw / 16384.0;
}

// ジャイロセンサー読み取り
void readGyro(float *x, float *y, float *z) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x43);  // GYRO_XOUT_H
    if (Wire.endTransmission(false) != 0) {
        *x = *y = *z = 0;
        return;
    }

    Wire.requestFrom(MPU6050_ADDRESS, 6);
    if (Wire.available() < 6) {
        *x = *y = *z = 0;
        return;
    }

    int16_t xRaw = (Wire.read() << 8) | Wire.read();
    int16_t yRaw = (Wire.read() << 8) | Wire.read();
    int16_t zRaw = (Wire.read() << 8) | Wire.read();

    *x = xRaw / 131.0;
    *y = yRaw / 131.0;
    *z = zRaw / 131.0;
}

// センサーのセットアップ
void setupSensors() {
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    // HSPPAD143Cの初期化
    writeRegister(PRESSURE_ADDRESS, 0x11, 0x80);  // RESET
    delay(3);
    writeRegister(PRESSURE_ADDRESS, 0x13, 0x39);  // AVG setting
    delay(10);
    writeRegister(PRESSURE_ADDRESS, 0x0F, 0xA5);  // Start measuring
    delay(10);

    // MPU-6050の初期化
    writeRegister(MPU6050_ADDRESS, 0x6B, 0x00);  // Wake up
    writeRegister(MPU6050_ADDRESS, 0x1B, 0x00);  // Gyro config
    writeRegister(MPU6050_ADDRESS, 0x1C, 0x00);  // Accel config
}

// デバッグ用：バイト配列を16進数で表示
void print_hex_dump(const char* title, const uint8_t* data, uint16_t len) {
    Serial.println(title);
    for (uint16_t i = 0; i < len; i++) {
        if (data[i] < 0x10) Serial.print("0");
        Serial.print(data[i], HEX);
        Serial.print(" ");
        if ((i + 1) % 16 == 0) Serial.println();
    }
    Serial.println();
}

// アドバタイジングデータを更新
void updateAdvertisingData() {
    // パケットIDを3バイト目に設定
    advData[2] = packetId;

    // データ部分を埋める（最大8つのデータを格納）
    int dataCount = dataBufferCount;
    
    // 時間差情報格納用配列
    uint8_t timeDiffs[MAX_QUEUE_SIZE-1] = {0};
    
    // 時間差を計算
    for (int i = 1; i < dataCount; i++) {
        unsigned long timeDiff = dataBuffer[i].timestamp - dataBuffer[i-1].timestamp;
        Serial.print("Time diff ");
        Serial.print(i);
        Serial.print("-");
        Serial.print(i+1);
        Serial.print(": ");
        Serial.print(timeDiff);
        Serial.println(" ms");
        
        // ミリ秒単位の時間差を255以下に収める（単位はms/4）
        timeDiffs[i-1] = min(timeDiff / 4, 255UL);
    }

    // データを製造者データにパック
    for (int i = 0; i < dataCount; i++) {
        sensor_data_t data = dataBuffer[i];

        // データをバイトに変換
        int16_t pressureData = (int16_t)(data.pressure * 10);
        int16_t accelX = (int16_t)(data.ax * 10000);
        int16_t accelY = (int16_t)(data.ay * 10000);
        int16_t accelZ = (int16_t)(data.az * 10000);

        // 各データを製造者データにパック
        int offset = 3 + (i * 8);

        advData[offset + 0] = pressureData & 0xFF;
        advData[offset + 1] = (pressureData >> 8) & 0xFF;

        advData[offset + 2] = accelX & 0xFF;
        advData[offset + 3] = (accelX >> 8) & 0xFF;

        advData[offset + 4] = accelY & 0xFF;
        advData[offset + 5] = (accelY >> 8) & 0xFF;

        advData[offset + 6] = accelZ & 0xFF;
        advData[offset + 7] = (accelZ >> 8) & 0xFF;
    }

    // 製造者IDを設定
    advData[0] = 0xCB;
    advData[1] = 0xCD;

    // 時間差情報を追加
    int timeDiffOffset = 3 + (MAX_QUEUE_SIZE * 8);
    for (int i = 0; i < MAX_QUEUE_SIZE-1; i++) {
        advData[timeDiffOffset + i] = timeDiffs[i];
    }

    // デバッグ出力
    print_hex_dump("Advertisement data:", advData, timeDiffOffset + MAX_QUEUE_SIZE - 1);

    esp_err_t ret;
    // データサイズを計算
    int dataSize = timeDiffOffset + MAX_QUEUE_SIZE - 1;
    
    // 拡張アドバタイズデータを設定
    ret = esp_ble_gap_config_ext_adv_data_raw(EXT_ADV_HANDLE, dataSize, advData);
    if (ret != ESP_OK) {
        Serial.print("Config extended adv data failed: ");
        Serial.println(esp_err_to_name(ret));
    }
    
    // 定期的アドバタイズデータも設定（同じデータを使用）
    ret = esp_ble_gap_config_periodic_adv_data_raw(PERIOD_ADV_HANDLE, dataSize, advData);
    if (ret != ESP_OK) {
        Serial.print("Config periodic adv data failed: ");
        Serial.println(esp_err_to_name(ret));
    }
    
    // LEDを点滅させて動作を確認
    digitalWrite(ledPin, !digitalRead(ledPin));
    
    Serial.print("Sent packet ID: ");
    Serial.print(packetId);
    Serial.print(" with ");
    Serial.print(dataCount);
    Serial.println(" data entries");
}

// BLEイベントハンドラ
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_EXT_ADV_SET_RAND_ADDR_COMPLETE_EVT:
            Serial.print("ESP_GAP_BLE_EXT_ADV_SET_RAND_ADDR_COMPLETE_EVT, status: ");
            Serial.println(param->ext_adv_set_rand_addr.status);
            break;
            
        case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
            Serial.print("ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT, status: ");
            Serial.println(param->ext_adv_set_params.status);
            break;
            
        case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
            Serial.print("ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT, status: ");
            Serial.println(param->ext_adv_data_set.status);
            break;
            
        case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
            Serial.print("ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT, status: ");
            Serial.println(param->ext_adv_start.status);
            break;
            
        case ESP_GAP_BLE_PERIODIC_ADV_SET_PARAMS_COMPLETE_EVT:
            Serial.print("ESP_GAP_BLE_PERIODIC_ADV_SET_PARAMS_COMPLETE_EVT, status: ");
            Serial.println(param->peroid_adv_set_params.status);
            break;
            
        case ESP_GAP_BLE_PERIODIC_ADV_DATA_SET_COMPLETE_EVT:
            Serial.print("ESP_GAP_BLE_PERIODIC_ADV_DATA_SET_COMPLETE_EVT, status: ");
            Serial.println(param->period_adv_data_set.status);
            break;
            
        case ESP_GAP_BLE_PERIODIC_ADV_START_COMPLETE_EVT:
            Serial.print("ESP_GAP_BLE_PERIODIC_ADV_START_COMPLETE_EVT, status: ");
            Serial.println(param->period_adv_start.status);
            break;
            
        default:
            break;
    }
}

// サンプルデータを収集
void collectSensorData() {
    // バッファがいっぱいなら、古いデータを削除
    if (dataBufferCount >= MAX_QUEUE_SIZE) {
        // 全データを1つ左にシフト
        for (int i = 0; i < MAX_QUEUE_SIZE - 1; i++) {
            dataBuffer[i] = dataBuffer[i + 1];
        }
        dataBufferCount--;
    }
    
    // 新しいデータを末尾に追加
    sensor_data_t newData;
    newData.timestamp = millis();
    
    // センサーデータの読み取り
    newData.pressure = readPressureHPa();
    readAccel(&newData.ax, &newData.ay, &newData.az);
    readGyro(&newData.gx, &newData.gy, &newData.gz);
    
    // バッファに追加
    dataBuffer[dataBufferCount] = newData;
    dataBufferCount++;
}

// BLE初期化フェーズ1: コントローラの初期化
void ble_init_phase1() {
    esp_err_t ret;
    Serial.println("BLE init phase 1");
    
    // Bluetoothコントローラスタックのメモリ解放
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        Serial.print("Bluetooth controller release classic bt memory failed: ");
        Serial.println(esp_err_to_name(ret));
        return;
    }
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        Serial.print("Bluetooth controller init failed: ");
        Serial.println(esp_err_to_name(ret));
        return;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        Serial.print("Bluetooth controller enable failed: ");
        Serial.println(esp_err_to_name(ret));
        return;
    }
}

// BLE初期化フェーズ2: HCIとGAPの初期化
void ble_init_phase2() {
    esp_err_t ret;
    Serial.println("BLE init phase 2");
    
    // Bluetooth HCIレイヤー初期化
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        Serial.print("Bluetooth bluedroid init failed: ");
        Serial.println(esp_err_to_name(ret));
        return;
    }
    
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        Serial.print("Bluetooth bluedroid enable failed: ");
        Serial.println(esp_err_to_name(ret));
        return;
    }
    
    // GAPコールバック登録
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret != ESP_OK) {
        Serial.print("GAP register callback failed, error code = ");
        Serial.println(ret, HEX);
        return;
    }
    
    // Bluetooth送信電力の設定
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
}

// BLE初期化フェーズ3: 拡張アドバタイズの設定
void ble_init_phase3() {
    esp_err_t ret;
    Serial.println("BLE init phase 3 - setting up extended advertising");
    
    // パラメータの初期化（構造体を個別に設定）
    memset(&ext_adv_params_coded, 0, sizeof(ext_adv_params_coded));
    ext_adv_params_coded.type = ESP_BLE_GAP_SET_EXT_ADV_PROP_CONNECTABLE;
    ext_adv_params_coded.interval_min = 0x30;
    ext_adv_params_coded.interval_max = 0x60;
    ext_adv_params_coded.channel_map = ADV_CHNL_ALL;
    ext_adv_params_coded.filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
    ext_adv_params_coded.primary_phy = ESP_BLE_GAP_PHY_CODED;
    ext_adv_params_coded.max_skip = 0;
    ext_adv_params_coded.secondary_phy = ESP_BLE_GAP_PHY_CODED;
    ext_adv_params_coded.sid = 0;
    ext_adv_params_coded.scan_req_notif = false;
    ext_adv_params_coded.own_addr_type = BLE_ADDR_TYPE_RANDOM;
    ext_adv_params_coded.tx_power = 9;
    
    // 定期的アドバタイズパラメータの初期化
    memset(&periodic_adv_params, 0, sizeof(periodic_adv_params));
    periodic_adv_params.interval_min = 0x40;
    periodic_adv_params.interval_max = 0x80;
    periodic_adv_params.properties = 0;
    
    // ext_adv構造体の初期化
    memset(ext_adv, 0, sizeof(ext_adv));
    ext_adv[0].instance = EXT_ADV_HANDLE;
    ext_adv[0].duration = 0;
    ext_adv[0].max_events = 0;
    
    // 1. ランダムアドレスの設定（インスタンス毎に設定が必要）
    ret = esp_ble_gap_ext_adv_set_rand_addr(EXT_ADV_HANDLE, rnd_addr);
    if (ret != ESP_OK) {
        Serial.print("Extended adv set random address failed, error code = ");
        Serial.println(ret, HEX);
        return;
    }
    // 設定が完了するまで少し待機
    delay(100);
    
    // 2. 拡張アドバタイズパラメータの設定
    ret = esp_ble_gap_ext_adv_set_params(EXT_ADV_HANDLE, &ext_adv_params_coded);
    if (ret != ESP_OK) {
        Serial.print("Extended adv set params failed, error code = ");
        Serial.println(ret, HEX);
        return;
    }
    // 設定が完了するまで少し待機
    delay(100);
    
    // 3. 初期アドバタイズデータの設定（必須: 空のデータでも必要）
    uint8_t initial_data[3] = {0xCB, 0xCD, 0x00};  // 製造者ID + 初期パケットID
    ret = esp_ble_gap_config_ext_adv_data_raw(EXT_ADV_HANDLE, sizeof(initial_data), initial_data);
    if (ret != ESP_OK) {
        Serial.print("Configure extended adv data failed, error code = ");
        Serial.println(ret, HEX);
        return;
    }
    // 設定が完了するまで少し待機
    delay(100);
    
    // 4. 拡張アドバタイズの開始
    ret = esp_ble_gap_ext_adv_start(1, ext_adv);
    if (ret != ESP_OK) {
        Serial.print("Extended adv start failed, error code = ");
        Serial.println(ret, HEX);
        return;
    }
    // 開始が完了するまで少し待機
    delay(100);
    
    // 5. 定期的アドバタイズパラメータの設定
    ret = esp_ble_gap_periodic_adv_set_params(PERIOD_ADV_HANDLE, &periodic_adv_params);
    if (ret != ESP_OK) {
        Serial.print("Periodic adv set params failed, error code = ");
        Serial.println(ret, HEX);
        return;
    }
    // 設定が完了するまで少し待機
    delay(100);
    
    // 6. 初期定期的アドバタイズデータの設定（必須: 空のデータでも必要）
    ret = esp_ble_gap_config_periodic_adv_data_raw(PERIOD_ADV_HANDLE, sizeof(initial_data), initial_data);
    if (ret != ESP_OK) {
        Serial.print("Configure periodic adv data failed, error code = ");
        Serial.println(ret, HEX);
        return;
    }
    // 設定が完了するまで少し待機
    delay(100);
    
    // 7. 定期的アドバタイズの開始
    ret = esp_ble_gap_periodic_adv_start(PERIOD_ADV_HANDLE);
    if (ret != ESP_OK) {
        Serial.print("Periodic adv start failed, error code = ");
        Serial.println(ret, HEX);
        return;
    }
    
    Serial.println("Extended & periodic advertising setup complete");
}

// 初期化
void setup() {
    // LED初期化
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
    
    // シリアル通信開始
    Serial.begin(115200);
    delay(1000); // シリアルポートが安定するまで待機
    
    Serial.println("Starting BLE 5 sensor data logging with S=8 Coded PHY...");
    
    // NVSの初期化
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // センサーの初期化
    setupSensors();
    
    // BLE初期化
    ble_init_phase1();
    delay(500);
    
    ble_init_phase2();
    delay(500);
    
    ble_init_phase3();
    delay(500);
    
    Serial.println("BLE 5 Extended & Periodic Advertising started");
}

// メインループ
void loop() {
    static unsigned long lastSendTime = 0;
    const int SEND_INTERVAL = 100;  // 10Hz = 100ms
    
    unsigned long currentTime = millis();
    
    // 正確に10Hzのタイミングで送信
    if (currentTime - lastSendTime >= SEND_INTERVAL) {
        // 次の送信時刻を計算（誤差を蓄積させない）
        lastSendTime += SEND_INTERVAL;
        
        // タイミング逆転の防止
        if (currentTime > lastSendTime + SEND_INTERVAL) {
            lastSendTime = currentTime;
        }
        
        // センサーデータを収集
        collectSensorData();
        
        // アドバタイズデータを更新
        updateAdvertisingData();
        
        // パケットIDを更新
        packetId++;
    }
    
    // 他のタスクのために少し待機
    delay(10);
}