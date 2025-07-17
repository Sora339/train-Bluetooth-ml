#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include <queue>
#include <vector>

// I2Cアドレスの定数
#define PRESSURE_ADDRESS 0x48  // HSPPAD143C
#define MPU6050_ADDRESS 0x68   // MPU-6050
#define SDA_PIN 22             // ESP32のデフォルトSDA
#define SCL_PIN 23             // ESP32のデフォルトSCL

// HSPPAD143Cレジスタ
#define PRESSURE_REG_RESET 0x11
#define PRESSURE_REG_AVG 0x13
#define PRESSURE_REG_MEASURE 0x0F
#define PRESSURE_REG_POUT 0x04

// MPU-6050レジスタ
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

// BLE UUID
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

// 製造者ID (Railcollectorを識別するID)
#define MANUFACTURER_ID 0xCDCB  // 製造者ID (52651)

// 個体ID (Raicollector同士を識別するID) 干渉テスト時にここを別々にしたものを使う
#define RAICOLLECTOR_ID 0xABCD  // 個体ID (43981)

// 送信設定
#define SEND_INTERVAL 100          // 10Hz = 100ms ここは現在は値を書き換える頻度
#define DATA_REPEAT_COUNT 3        // 各データを3回送信
#define MANUFACTURER_DATA_SIZE 29  // 拡張したデータサイズ

// グローバル変数
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
BLEAdvertising *pAdvertising = NULL;
const int ledPin = 2;            // ESP32の内蔵LED
unsigned long lastSendTime = 0;  // 最後の送信時刻
unsigned long startTime = 0;     // 開始時間
uint8_t packetId = 0;            // 8ビットの通し番号ID

// デバッグモード
bool debugMode = true;  // デバッグ出力を有効化

// センサーデータ構造体
typedef struct {
  float pressure;
  float ax, ay, az;
  float gx, gy, gz;
  unsigned long timestamp;  // 測定時間
} sensor_data_t;

// データキュー
std::queue<sensor_data_t> dataQueue;

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
// void readGyro(float *x, float *y, float *z) {
//   Wire.beginTransmission(MPU6050_ADDRESS);
//   Wire.write(GYRO_XOUT_H);
//   if (Wire.endTransmission(false) != 0) {
//     *x = *y = *z = 0;
//     return;
//   }

//   Wire.requestFrom(MPU6050_ADDRESS, 6);
//   if (Wire.available() < 6) {
//     *x = *y = *z = 0;
//     return;
//   }

//   int16_t xRaw = (Wire.read() << 8) | Wire.read();
//   int16_t yRaw = (Wire.read() << 8) | Wire.read();
//   int16_t zRaw = (Wire.read() << 8) | Wire.read();

//   *x = xRaw / 131.0;
//   *y = yRaw / 131.0;
//   *z = zRaw / 131.0;
// }

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

// BLEセットアップ
void setupBLE() {
  BLEDevice::init("PicoTest");
  pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

  pService->start();
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);  // スキャン応答は無効
  pAdvertising->setMinPreferred(0x06);  // モバイルデバイス用の設定

  // 特に重要：アドバタイズの設定
  pAdvertising->setMinInterval(40);  // 最小アドバタイズ間隔
  pAdvertising->setMaxInterval(40);  // 最大アドバタイズ間隔

  // 製造者IDを設定
  BLEAdvertisementData scanResponseData;

  // 製造者IDをStringに変換して設定
  String manufacturerIdData;
  scanResponseData.setManufacturerData(manufacturerIdData);
  pAdvertising->setScanResponseData(scanResponseData);

  BLEDevice::startAdvertising();
  Serial.println("BLE advertising started");
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
  // 29バイトの製造者データを作成
  uint8_t manufacturerData[MANUFACTURER_DATA_SIZE] = { 0 };

  // パケットIDを5バイト目に設定
  manufacturerData[4] = packetId;

  // データ部分を埋める（最大3つのデータを格納）
  int dataCount = 0;            // 格納したデータの数
  // unsigned long timeDiff1 = 0;  // 1つ目と2つ目のデータの時間差
  // unsigned long timeDiff2 = 0;  // 2つ目と3つ目のデータの時間差

  // キューの内容をコピーして、キュー自体を変更しないようにする
  std::queue<sensor_data_t> tempQueue = dataQueue;
  std::vector<sensor_data_t> dataEntries;  // データを一時保存する配列

  // キューからデータを取り出して配列に保存
  while (!tempQueue.empty() && dataCount < 3) {
    dataEntries.push_back(tempQueue.front());
    tempQueue.pop();
    dataCount++;
  }

  // // データが2つ以上ある場合、時間差を計算
  // if (dataCount >= 2) {
  //   timeDiff1 = dataEntries[1].timestamp - dataEntries[0].timestamp;
  //   // デバッグ表示
  //   if (debugMode) {
  //     Serial.print("Time diff 1-2: ");
  //     Serial.print(timeDiff1);
  //     Serial.println(" ms");
  //   }
  //   // ミリ秒単位の時間差を255以下に収める（単位はms/4）
  //   timeDiff1 = min(timeDiff1 / 4, 255UL);
  // }

  // // データが3つある場合、2つ目の時間差も計算
  // if (dataCount >= 3) {
  //   timeDiff2 = dataEntries[2].timestamp - dataEntries[1].timestamp;
  //   // デバッグ表示
  //   if (debugMode) {
  //     Serial.print("Time diff 2-3: ");
  //     Serial.print(timeDiff2);
  //     Serial.println(" ms");
  //   }
  //   // ミリ秒単位の時間差を255以下に収める（単位はms/4）
  //   timeDiff2 = min(timeDiff2 / 4, 255UL);
  // }

  // データを製造者データにパック
  for (int i = 0; i < min(dataCount, 3); i++) {
    sensor_data_t data = dataEntries[i];

    // データをバイトに変換
    int16_t pressureData = (int16_t)(data.pressure * 10);  // 1桁の小数点精度を保持
    int16_t accelX = (int16_t)(data.ax * 10000);           // 4桁の小数点精度を保持
    int16_t accelY = (int16_t)(data.ay * 10000);
    int16_t accelZ = (int16_t)(data.az * 10000);

    // 各データを製造者データにパック（6バイト目以降に配置）
    int offset = 5 + (i * 8);  // 各データは8バイト

    manufacturerData[offset + 0] = pressureData & 0xFF;
    manufacturerData[offset + 1] = (pressureData >> 8) & 0xFF;

    manufacturerData[offset + 2] = accelX & 0xFF;
    manufacturerData[offset + 3] = (accelX >> 8) & 0xFF;

    manufacturerData[offset + 4] = accelY & 0xFF;
    manufacturerData[offset + 5] = (accelY >> 8) & 0xFF;

    manufacturerData[offset + 6] = accelZ & 0xFF;
    manufacturerData[offset + 7] = (accelZ >> 8) & 0xFF;
  }

  manufacturerData[0] = MANUFACTURER_ID & 0xFF;
  manufacturerData[1] = (MANUFACTURER_ID >> 8) & 0xFF;

  manufacturerData[2] = RAICOLLECTOR_ID & 0xFF;
  manufacturerData[3] = (RAICOLLECTOR_ID >> 8) & 0xFF;

  // 時間差情報を追加（最後の2バイト）
  // manufacturerData[27] = (uint8_t)timeDiff1;  // 1つ目と2つ目の時間差
  // manufacturerData[28] = (uint8_t)timeDiff2;  // 2つ目と3つ目の時間差

  // デバッグ出力
  if (debugMode) {
    dumpHex(manufacturerData, MANUFACTURER_DATA_SIZE);
  }

  // BLEアドバタイジングデータを作成
  // Arduino Stringクラスを使用
  String manufacturerString;

  // manufacturerDataの内容をArduino Stringに変換
  for (int i = 0; i < MANUFACTURER_DATA_SIZE; i++) {
    manufacturerString += (char)manufacturerData[i];
  }

  BLEAdvertisementData advData;
  advData.setManufacturerData(manufacturerString);

  // アドバタイジングデータを設定して開始
  pAdvertising->setAdvertisementData(advData);
  pAdvertising->start();

  // LED点滅
  digitalWrite(ledPin, !digitalRead(ledPin));

  // デバッグ情報
  Serial.print("Sent packet ID: ");
  Serial.print(packetId);
  Serial.print(" with ");
  Serial.print(dataCount);
  Serial.print(" data entries. Time diffs: ");
  // Serial.print(timeDiff1 * 4);  // 元の単位に戻す
  Serial.print("ms, ");
  // Serial.print(timeDiff2 * 4);  // 元の単位に戻す
  Serial.println("ms");
}

// 初期化
void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // シリアル通信開始（高速に設定）
  Serial.begin(115200);
  Serial.println("Starting sensor data logging with enhanced packet structure...");
  Serial.println("timestamp pressure accel_x accel_y accel_z gyro_x gyro_y gyro_z");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // センサー初期化を呼び出し
  if (!setupSensors()) {
    Serial.println("Sensor setup failed!");
    while (1) {
      digitalWrite(ledPin, HIGH);
      delay(200);
      digitalWrite(ledPin, LOW);
      delay(200);
    }
  }

  setupBLE();
  startTime = millis();  // 開始時間を記録
}

// メインループ
void loop() {
  unsigned long currentTime = millis();

  // 正確に10Hzのタイミングで送信
  if (currentTime - lastSendTime >= SEND_INTERVAL) {
    // 次の送信時刻を計算（誤差を蓄積させない）
    lastSendTime += SEND_INTERVAL;

    // タイミング逆転の防止
    if (currentTime > lastSendTime + SEND_INTERVAL) {
      lastSendTime = currentTime;
    }

    // 新しいセンサーデータを読み取り
    sensor_data_t newData;
    newData.timestamp = currentTime;
    
    // ここでセンサーデータを実際に読み取る
    newData.pressure = readPressureHPa();
    readAccel(&newData.ax, &newData.ay, &newData.az);
    // readGyro(&newData.gx, &newData.gy, &newData.gz);

    // キューに新しいデータを追加
    dataQueue.push(newData);

    // キューが大きくなりすぎないように古いデータを削除（メモリ管理）
    if (dataQueue.size() > 5) {  // 最大5個のデータを保持
      dataQueue.pop();
    }

    // アドバタイジングデータを更新
    updateAdvertisingData();
    packetId++;  // IDをインクリメント（255の次は自動的に0になる）

    // シリアルでデータを送信（最新のデータのみ）
    sendDataToSerial(newData);
  }
}