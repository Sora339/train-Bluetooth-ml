#include <Wire.h>
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include <queue>
#include <vector>

// I2Cアドレスの定数
#define PRESSURE_ADDRESS 0x48  // HSPPAD143C
#define MPU6050_ADDRESS 0x68   // MPU-6050
#define SDA_PIN 20             // ESP32のデフォルトSDA
#define SCL_PIN 21             // ESP32のデフォルトSCL

// HSPPAD143Cレジスタ
#define PRESSURE_REG_RESET 0x11
#define PRESSURE_REG_AVG 0x13
#define PRESSURE_REG_MEASURE 0x0F
#define PRESSURE_REG_POUT 0x04

// MPU-6050レジスタ
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

// 送信設定
#define SEND_INTERVAL 100          // 10Hz = 100ms
#define DATA_REPEAT_COUNT 3        // 各データを3回送信
#define MAX_QUEUE_SIZE 8           // キューに格納する最大データ数（拡張）
#define MANUFACTURER_DATA_SIZE 100 // 拡張アドバタイズでの最大サイズ（少し小さくした）

// アドバタイズインスタンス
#define ADV_INSTANCE 0             // 拡張アドバタイズのインスタンス番号

// 製造者ID (実際に使われているID)
#define MANUFACTURER_ID 0xCDCB  // 製造者ID (52651)

// グローバル変数
const int ledPin = 2;            // ESP32の内蔵LED
unsigned long lastSendTime = 0;  // 最後の送信時刻
unsigned long startTime = 0;     // 開始時間
uint8_t packetId = 0;            // 8ビットの通し番号ID

// BLE変数
uint8_t advData[MANUFACTURER_DATA_SIZE] = {0};
esp_ble_gap_ext_adv_t ext_adv[1] = {0};
esp_ble_gap_ext_adv_params_t ext_adv_params = {0};
esp_ble_gap_periodic_adv_params_t periodic_adv_params = {0};

// デバッグモード
bool debugMode = true;  // デバッグ出力を有効化

// S=8モードのPHYオプション定義（Arduino Core v3.2.0の定義に合わせる）
#ifndef ESP_BLE_GAP_PHY_OPTIONS_PREFER_S8_CODING
#define ESP_BLE_GAP_PHY_OPTIONS_PREFER_S8_CODING 2
#endif

// センサーデータ構造体
typedef struct {
  float pressure;
  float ax, ay, az;
  float gx, gy, gz;
  unsigned long timestamp;  // 測定時間
} sensor_data_t;

// データキュー
std::queue<sensor_data_t> dataQueue;

// BLE初期化のコールバック
void ble_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
      Serial.println("ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT");
      if (param->ext_adv_set_params.status == ESP_BT_STATUS_SUCCESS) {
        Serial.println("拡張アドバタイズパラメータの設定に成功しました");
      } else {
        Serial.print("拡張アドバタイズパラメータの設定に失敗しました: ");
        Serial.println(param->ext_adv_set_params.status);
      }
      break;
    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
      Serial.println("ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT");
      if (param->ext_adv_data_set.status != ESP_BT_STATUS_SUCCESS) {
        Serial.print("拡張アドバタイズデータの設定に失敗しました: ");
        Serial.println(param->ext_adv_data_set.status);
      }
      break;
    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
      Serial.println("ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT");
      if (param->ext_adv_start.status == ESP_BT_STATUS_SUCCESS) {
        Serial.println("拡張アドバタイズの開始に成功しました");
      } else {
        Serial.print("拡張アドバタイズの開始に失敗しました: ");
        Serial.println(param->ext_adv_start.status);
      }
      break;
    case ESP_GAP_BLE_PERIODIC_ADV_SET_PARAMS_COMPLETE_EVT:
      Serial.println("ESP_GAP_BLE_PERIODIC_ADV_SET_PARAMS_COMPLETE_EVT");
      if (param->peroid_adv_set_params.status != ESP_BT_STATUS_SUCCESS) {
        Serial.print("定期的アドバタイズパラメータの設定に失敗しました: ");
        Serial.println(param->peroid_adv_set_params.status);
      }
      break;
    case ESP_GAP_BLE_PERIODIC_ADV_DATA_SET_COMPLETE_EVT:
      Serial.println("ESP_GAP_BLE_PERIODIC_ADV_DATA_SET_COMPLETE_EVT");
      if (param->period_adv_data_set.status != ESP_BT_STATUS_SUCCESS) {
        Serial.print("定期的アドバタイズデータの設定に失敗しました: ");
        Serial.println(param->period_adv_data_set.status);
      }
      break;
    case ESP_GAP_BLE_PERIODIC_ADV_START_COMPLETE_EVT:
      Serial.println("ESP_GAP_BLE_PERIODIC_ADV_START_COMPLETE_EVT");
      if (param->period_adv_start.status == ESP_BT_STATUS_SUCCESS) {
        Serial.println("定期的アドバタイズの開始に成功しました");
      } else {
        Serial.print("定期的アドバタイズの開始に失敗しました: ");
        Serial.println(param->period_adv_start.status);
      }
      break;
    default:
      break;
  }
}

// センサーセットアップ
bool setupSensors() {
  bool success = true;

  writeRegister(PRESSURE_ADDRESS, PRESSURE_REG_RESET, 0x80);
  delay(3);
  writeRegister(PRESSURE_ADDRESS, PRESSURE_REG_AVG, 0x39);
  delay(10);
  writeRegister(PRESSURE_ADDRESS, PRESSURE_REG_MEASURE, 0xA5);
  delay(10);

  writeRegister(MPU6050_ADDRESS, 0x6B, 0x00);
  writeRegister(MPU6050_ADDRESS, 0x1B, 0x00);
  writeRegister(MPU6050_ADDRESS, 0x1C, 0x00);

  return success;
}

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
  Wire.write(PRESSURE_REG_POUT);
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
  pressureHPa = pressureHPa / 10.0;

  return pressureHPa;
}

// 加速度センサー読み取り
void readAccel(float *x, float *y, float *z) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(ACCEL_XOUT_H);
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
  Wire.write(GYRO_XOUT_H);
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

// バイト配列を16進数文字列に変換（デバッグ表示用）
void dumpHex(uint8_t *data, size_t len) {
  if (!debugMode) return;

  String hexString = "";
  for (size_t i = 0; i < len; i++) {
    if (data[i] < 0x10) hexString += "0";
    hexString += String(data[i], HEX);
    hexString += " ";
  }
  Serial.println("HEX: " + hexString);
}

// コーディングPHYのS=8モード（長距離モード）を有効化
void enableCodingPhyS8() {
  // 重要: デフォルトのPHY設定でS=8モードを指定
  esp_ble_gap_prefer_phy_options_t phy_options = ESP_BLE_GAP_PHY_OPTIONS_PREFER_S8_CODING;
  
  // デフォルトのPHY設定を構成
  esp_ble_gap_phy_mask_t tx_phy_mask = ESP_BLE_GAP_PHY_CODED_PREF_MASK;
  esp_ble_gap_phy_mask_t rx_phy_mask = ESP_BLE_GAP_PHY_CODED_PREF_MASK;
  
  // デフォルトのPHY設定を適用
  esp_err_t err = esp_ble_gap_set_preferred_default_phy(tx_phy_mask, rx_phy_mask);
  
  if (err == ESP_OK) {
    Serial.println("デフォルトPHYをコーディングPHY（S=8）に設定しました");
  } else {
    Serial.print("デフォルトPHYの設定に失敗しました: ");
    Serial.println(err);
  }
}

// BLE 5 拡張アドバタイズセットアップ
void setupBLE5() {
  // BLEスタックの初期化
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
  
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
  ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
  ESP_ERROR_CHECK(esp_bluedroid_init());
  ESP_ERROR_CHECK(esp_bluedroid_enable());
  
  // 最大送信電力を設定
  esp_err_t power_err = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  if (power_err == ESP_OK) {
    Serial.println("送信電力を最大に設定しました");
  }
  
  // デバイス名の設定
  esp_ble_gap_set_device_name("PicoTest");
  
  // BLEコールバック登録
  ESP_ERROR_CHECK(esp_ble_gap_register_callback(ble_callback));
  
  // S=8コーディングPHYを有効化
  enableCodingPhyS8();
  
  // 重要: 拡張アドバタイズの順序は重要
  // 1. 拡張アドバタイズのパラメータを設定
  memset(&ext_adv_params, 0, sizeof(ext_adv_params));
  ext_adv_params.type = ESP_BLE_GAP_SET_EXT_ADV_PROP_CONNECTABLE;
  ext_adv_params.interval_min = 40;  // 最小間隔 (40 * 0.625ms = 25ms)
  ext_adv_params.interval_max = 80;  // 最大間隔 (80 * 0.625ms = 50ms)
  ext_adv_params.primary_phy = ESP_BLE_GAP_PHY_CODED;  // コーディングPHY
  ext_adv_params.secondary_phy = ESP_BLE_GAP_PHY_CODED;  // コーディングPHY
  ext_adv_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
  ext_adv_params.filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
  ext_adv_params.tx_power = 127;  // 最大送信電力を要求 (127 = コントローラーのデフォルト)
  
  Serial.println("拡張アドバタイズパラメータを設定しています...");
  esp_err_t adv_param_err = esp_ble_gap_ext_adv_set_params(ADV_INSTANCE, &ext_adv_params);
  if (adv_param_err != ESP_OK) {
    Serial.print("拡張アドバタイズパラメータの設定に失敗: ");
    Serial.println(adv_param_err);
  }
  delay(100); // パラメータ設定が適用されるまで待機
  
  // 2. 定期的アドバタイズのパラメータ設定
  memset(&periodic_adv_params, 0, sizeof(periodic_adv_params));
  periodic_adv_params.interval_min = 80;   // 最小間隔 (80 * 1.25ms = 100ms)
  periodic_adv_params.interval_max = 160;  // 最大間隔 (160 * 1.25ms = 200ms)
  periodic_adv_params.properties = 0;      // ビットマスク（0=基本的な定期的アドバタイズ）
  
  Serial.println("定期的アドバタイズパラメータを設定しています...");
  esp_err_t periodic_param_err = esp_ble_gap_periodic_adv_set_params(ADV_INSTANCE, &periodic_adv_params);
  if (periodic_param_err != ESP_OK) {
    Serial.print("定期的アドバタイズパラメータの設定に失敗: ");
    Serial.println(periodic_param_err);
  }
  delay(100); // パラメータ設定が適用されるまで待機
  
  // 3. 拡張アドバタイズと定期的アドバタイズを有効化
  ext_adv[0].instance = ADV_INSTANCE;
  ext_adv[0].duration = 0; // 無期限
  ext_adv[0].max_events = 0; // 制限なし
  
  // 初期データとして製造者IDだけを設定（最初のデータ更新の前に必要）
  uint8_t initial_data[3] = {0};
  initial_data[0] = MANUFACTURER_ID & 0xFF;
  initial_data[1] = (MANUFACTURER_ID >> 8) & 0xFF;
  initial_data[2] = 0; // 初期パケットID
  
  // 初期データを設定
  Serial.println("初期拡張アドバタイズデータを設定しています...");
  esp_err_t initial_data_err = esp_ble_gap_config_ext_adv_data_raw(ADV_INSTANCE, 3, initial_data);
  if (initial_data_err != ESP_OK) {
    Serial.print("初期拡張アドバタイズデータの設定に失敗: ");
    Serial.println(initial_data_err);
  }
  delay(100); // データ設定が適用されるまで待機
  
  // 初期定期的アドバタイズデータも設定
  Serial.println("初期定期的アドバタイズデータを設定しています...");
  esp_err_t initial_periodic_data_err = esp_ble_gap_config_periodic_adv_data_raw(ADV_INSTANCE, 3, initial_data);
  if (initial_periodic_data_err != ESP_OK) {
    Serial.print("初期定期的アドバタイズデータの設定に失敗: ");
    Serial.println(initial_periodic_data_err);
  }
  delay(100); // データ設定が適用されるまで待機
  
  // 拡張アドバタイズを開始
  Serial.println("拡張アドバタイズを開始しています...");
  esp_err_t adv_start_err = esp_ble_gap_ext_adv_start(1, ext_adv);
  if (adv_start_err != ESP_OK) {
    Serial.print("拡張アドバタイズの開始に失敗: ");
    Serial.println(adv_start_err);
  } else {
    Serial.println("拡張アドバタイズを開始しました");
  }
  delay(100); // アドバタイズが開始されるまで待機
  
  // 定期的アドバタイズを開始
  Serial.println("定期的アドバタイズを開始しています...");
  esp_err_t periodic_start_err = esp_ble_gap_periodic_adv_start(ADV_INSTANCE);
  if (periodic_start_err != ESP_OK) {
    Serial.print("定期的アドバタイズの開始に失敗: ");
    Serial.println(periodic_start_err);
  } else {
    Serial.println("定期的アドバタイズを開始しました");
  }
  
  Serial.println("BLE 5 拡張アドバタイズとコーディングPHY（S=8）設定完了");
}

// タイムスタンプを生成（mm:ss.d形式）
String getTimeStamp() {
  unsigned long current = millis() - startTime;
  unsigned long seconds = (current / 1000) % 60;
  unsigned long minutes = (current / 60000) % 60;
  unsigned long tenths = (current % 1000) / 100;

  char timeString[10];
  sprintf(timeString, "%02lu:%02lu.%01lu", minutes, seconds, tenths);
  return String(timeString);
}

// センサーデータをシリアルで送信
void sendDataToSerial(const sensor_data_t &data) {
  String timeStamp = getTimeStamp();
  String dataString = timeStamp + " " + String(data.pressure, 1) + " " + String(data.ax, 4) + " " + String(data.ay, 4) + " " + String(data.az, 4) + " " + String(data.gx, 1) + " " + String(data.gy, 1) + " " + String(data.gz, 1);

  Serial.println(dataString);
}

// アドバタイジングデータを更新
void updateAdvertisingData() {
  // パケットIDを3バイト目に設定
  advData[2] = packetId;

  // データ部分を埋める（最大8つのデータを格納）
  int dataCount = 0;
  
  // 時間差情報格納用配列
  uint8_t timeDiffs[MAX_QUEUE_SIZE-1] = {0};
  
  // キューの内容をコピー
  std::queue<sensor_data_t> tempQueue = dataQueue;
  std::vector<sensor_data_t> dataEntries;

  // キューからデータを取り出して配列に保存
  while (!tempQueue.empty() && dataCount < MAX_QUEUE_SIZE) {
    dataEntries.push_back(tempQueue.front());
    tempQueue.pop();
    dataCount++;
  }

  // 時間差を計算
  for (int i = 1; i < dataCount; i++) {
    unsigned long timeDiff = dataEntries[i].timestamp - dataEntries[i-1].timestamp;
    if (debugMode) {
      Serial.print("Time diff ");
      Serial.print(i);
      Serial.print("-");
      Serial.print(i+1);
      Serial.print(": ");
      Serial.print(timeDiff);
      Serial.println(" ms");
    }
    // ミリ秒単位の時間差を255以下に収める（単位はms/4）
    timeDiffs[i-1] = min(timeDiff / 4, 255UL);
  }

  // データを製造者データにパック
  for (int i = 0; i < dataCount; i++) {
    sensor_data_t data = dataEntries[i];

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
  advData[0] = MANUFACTURER_ID & 0xFF;
  advData[1] = (MANUFACTURER_ID >> 8) & 0xFF;

  // 時間差情報を追加
  int timeDiffOffset = 3 + (MAX_QUEUE_SIZE * 8);
  for (int i = 0; i < MAX_QUEUE_SIZE-1; i++) {
    advData[timeDiffOffset + i] = timeDiffs[i];
  }

  // デバッグ出力
  if (debugMode) {
    dumpHex(advData, timeDiffOffset + MAX_QUEUE_SIZE - 1);
  }

  // データサイズを計算
  int dataSize = timeDiffOffset + MAX_QUEUE_SIZE - 1;
  
  // データサイズが最大サイズを超えないように確認
  if (dataSize > MANUFACTURER_DATA_SIZE) {
    Serial.println("警告: データサイズが最大値を超えています。切り詰めます。");
    dataSize = MANUFACTURER_DATA_SIZE;
  }
  
  // BLE 5 拡張アドバタイズデータを設定
  esp_err_t ext_data_err = esp_ble_gap_config_ext_adv_data_raw(ADV_INSTANCE, dataSize, advData);
  
  // 定期的アドバタイズでも同じデータを使用
  esp_err_t periodic_data_err = esp_ble_gap_config_periodic_adv_data_raw(ADV_INSTANCE, dataSize, advData);
  
  if (ext_data_err != ESP_OK || periodic_data_err != ESP_OK) {
    Serial.print("アドバタイズデータの更新に失敗: ext=");
    Serial.print(ext_data_err);
    Serial.print(", periodic=");
    Serial.println(periodic_data_err);
  }

  // LED点滅
  digitalWrite(ledPin, !digitalRead(ledPin));

  // デバッグ情報
  Serial.print("Sent packet ID: ");
  Serial.print(packetId);
  Serial.print(" with ");
  Serial.print(dataCount);
  Serial.println(" data entries");
}

// 初期化
void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // シリアル通信開始
  Serial.begin(115200);
  delay(1000); // 安定化のための待機
  
  Serial.println("Starting BLE 5 sensor data logging with S=8 Coded PHY...");
  Serial.println("timestamp pressure accel_x accel_y accel_z gyro_x gyro_y gyro_z");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // センサー初期化
  if (!setupSensors()) {
    Serial.println("Sensor setup failed!");
    while (1) {
      digitalWrite(ledPin, HIGH);
      delay(200);
      digitalWrite(ledPin, LOW);
      delay(200);
    }
  }

  // BLE 5 初期化
  setupBLE5();
  startTime = millis();
}

// メインループ
void loop() {
  unsigned long currentTime = millis();

  // 正確に10Hzのタイミングで送信
  if (currentTime - lastSendTime >= SEND_INTERVAL) {
    // 次の送信時刻を計算
    lastSendTime += SEND_INTERVAL;

    // タイミング逆転の防止
    if (currentTime > lastSendTime + SEND_INTERVAL) {
      lastSendTime = currentTime;
    }

    // 新しいセンサーデータを読み取り
    sensor_data_t newData;
    newData.timestamp = currentTime;
    
    newData.pressure = readPressureHPa();
    readAccel(&newData.ax, &newData.ay, &newData.az);
    readGyro(&newData.gx, &newData.gy, &newData.gz);

    // キューに新しいデータを追加
    dataQueue.push(newData);

    // キューサイズの管理
    if (dataQueue.size() > MAX_QUEUE_SIZE) {
      dataQueue.pop();
    }

    // アドバタイジングデータを更新
    updateAdvertisingData();
    packetId++;

    // シリアルでデータを送信
    sendDataToSerial(newData);
  }
}