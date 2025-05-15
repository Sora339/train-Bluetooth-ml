#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include <queue>

// I2Cアドレスの定数
#define PRESSURE_ADDRESS 0x48  // HSPPAD143C
#define MPU6050_ADDRESS 0x68   // MPU-6050
#define SDA_PIN 20            // ESP32のデフォルトSDA
#define SCL_PIN 21            // ESP32のデフォルトSCL

// HSPPAD143Cレジスタ
#define PRESSURE_REG_RESET 0x11
#define PRESSURE_REG_AVG 0x13
#define PRESSURE_REG_MEASURE 0x0F
#define PRESSURE_REG_POUT 0x04

// MPU-6050レジスタ
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43

// BLE UUID
#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

// 製造者ID (0xFFFF)
#define MANUFACTURER_ID     0xFFFF

// 送信設定
#define SEND_INTERVAL 100  // 10Hz = 100ms
#define DATA_REPEAT_COUNT 3  // 各データを3回送信
#define MANUFACTURER_DATA_SIZE 27  // 拡張したデータサイズ

// グローバル変数
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
BLEAdvertising* pAdvertising = NULL;
const int ledPin = 2;  // ESP32の内蔵LED
unsigned long lastSendTime = 0;  // 最後の送信時刻
unsigned long startTime = 0;  // 開始時間
uint8_t packetId = 0;  // 8ビットの通し番号ID

// センサーデータ構造体
typedef struct {
  float pressure;
  float ax, ay, az;
  float gx, gy, gz;
  unsigned long timestamp;  // 測定時間
} sensor_data_t;

// データキュー
std::queue<sensor_data_t> dataQueue;

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
    newData.pressure = readPressureHPa();
    readAccel(&newData.ax, &newData.ay, &newData.az);
    readGyro(&newData.gx, &newData.gy, &newData.gz);
    newData.timestamp = currentTime;
    
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

// アドバタイジングデータを更新
void updateAdvertisingData() {
  // 27バイトの製造者データを作成
  uint8_t manufacturerData[MANUFACTURER_DATA_SIZE] = {0};
  
  // 最初の2バイトは識別子（変更なし）
  manufacturerData[0] = 0xAB;
  manufacturerData[1] = 0xCD;
  
  // 3バイト目はパケットID（通し番号）
  manufacturerData[2] = packetId;
  
  // データ部分を埋める（最大3つのデータを格納）
  int dataIndex = 3;  // データ部分の開始インデックス
  int dataCount = 0;  // 格納したデータの数
  
  // キューの内容をコピーして、キュー自体を変更しないようにする
  std::queue<sensor_data_t> tempQueue = dataQueue;
  
  while (!tempQueue.empty() && dataCount < 3 && dataIndex + 8 <= MANUFACTURER_DATA_SIZE) {
    sensor_data_t data = tempQueue.front();
    tempQueue.pop();
    
    // データをバイトに変換
    int16_t pressureData = (int16_t)(data.pressure * 10);  // 1桁の小数点精度を保持
    int16_t accelX = (int16_t)(data.ax * 10000);  // 4桁の小数点精度を保持
    int16_t accelY = (int16_t)(data.ay * 10000);
    int16_t accelZ = (int16_t)(data.az * 10000);
    
    // 27バイトに収めるため、ジャイロデータは省略
    
    // 各データを製造者データにパック
    manufacturerData[dataIndex++] = pressureData & 0xFF;
    manufacturerData[dataIndex++] = (pressureData >> 8) & 0xFF;
    
    manufacturerData[dataIndex++] = accelX & 0xFF;
    manufacturerData[dataIndex++] = (accelX >> 8) & 0xFF;
    
    manufacturerData[dataIndex++] = accelY & 0xFF;
    manufacturerData[dataIndex++] = (accelY >> 8) & 0xFF;
    
    manufacturerData[dataIndex++] = accelZ & 0xFF;
    manufacturerData[dataIndex++] = (accelZ >> 8) & 0xFF;
    
    dataCount++;
  }
  
  // 残りのバイトをゼロで埋める
  while (dataIndex < MANUFACTURER_DATA_SIZE) {
    manufacturerData[dataIndex++] = 0;
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
  Serial.println(" data entries");
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
void sendDataToSerial(const sensor_data_t& data) {
  String timeStamp = getTimeStamp();
  String dataString = timeStamp + " " +
                     String(data.pressure, 1) + " " +
                     String(data.ax, 4) + " " +
                     String(data.ay, 4) + " " +
                     String(data.az, 4) + " " +
                     String(data.gx, 1) + " " +
                     String(data.gy, 1) + " " +
                     String(data.gz, 1);
  
  Serial.println(dataString);
}

// I2Cレジスタ書き込み
void writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
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

// BLEセットアップ
void setupBLE() {
  BLEDevice::init("PicoTest");
  pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  
  pService->start();
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x06);
  
  BLEDevice::startAdvertising();
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