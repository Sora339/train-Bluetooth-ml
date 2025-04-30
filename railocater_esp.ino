#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include <string>  // ★追加

// I2Cアドレスの定数
#define PRESSURE_ADDRESS 0x48  // HSPPAD143C
#define MPU6050_ADDRESS 0x68   // MPU-6050
#define SDA_PIN 20            // ESP32のデフォルトSDA
#define SCL_PIN 21        // ESP32のデフォルトSCL

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

// グローバル変数
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
BLEAdvertising* pAdvertising = NULL;
const int ledPin = 2;  // ESP32の内蔵LED

// センサーデータ構造体
typedef struct {
  float pressure;
  float ax, ay, az;
  float gx, gy, gz;
} sensor_data_t;

sensor_data_t sensorData;

// アドバタイジングデータを更新する関数


void updateAdvertisingData() {
  Serial.print("sensorData.az: ");
  Serial.println(sensorData.az, 6); 
  digitalWrite(ledPin, !digitalRead(ledPin));

  uint8_t sensorDataBytes[12];  // ← 12バイトに拡張
  sensorDataBytes[0] = 0x01;

  int16_t pressureData = (int16_t)(sensorData.pressure * 10000);
  int16_t accelX = (int16_t)(sensorData.ax * 10000);
  int16_t accelY = (int16_t)(sensorData.ay * 10000);
  int16_t accelZ = (int16_t)(sensorData.az * 10000);

  sensorDataBytes[1] = pressureData & 0xFF;
  sensorDataBytes[2] = (pressureData >> 8) & 0xFF;

  sensorDataBytes[3] = accelX & 0xFF;
  sensorDataBytes[4] = (accelX >> 8) & 0xFF;

  sensorDataBytes[5] = accelY & 0xFF;
  sensorDataBytes[6] = (accelY >> 8) & 0xFF;

  sensorDataBytes[7] = accelZ & 0xFF;
  sensorDataBytes[8] = (accelZ >> 8) & 0xFF;

  // パディングバイト（ゴミ）：固定でも乱数でもOK
  sensorDataBytes[9] = 0xAB;
  sensorDataBytes[10] = 0xCD;
  sensorDataBytes[11] = 0xEF;

  // デバッグ出力
  Serial.print("アドバタイズデータ: ");
  for (int i = 0; i < 12; i++) {
    Serial.printf("%02X ", sensorDataBytes[i]);
  }
  Serial.println();

  // バイナリデータをStringに変換（文字列ではなくバイナリデータとして）
  String manufacturerData;
  manufacturerData.reserve(12); // 12バイト分の領域を確保
  
  for (int i = 0; i < 12; i++) {
    manufacturerData += (char)sensorDataBytes[i];
  }
  
  // BLEライブラリにStringとして渡す
  BLEAdvertisementData advData;
  advData.setManufacturerData(manufacturerData);
  pAdvertising->setAdvertisementData(advData);
  pAdvertising->start();
}






void setup() {
  // シリアル通信の初期化
  Serial.begin(9600);
  Serial.println("起動中...");
  
  // LED初期化
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  // ESP32特有のピンでI2Cを初期化
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  // 400kHz
  
  Serial.println("電源投入開始");
  delay(200);
  
  // I2Cデバイスのスキャン
  Serial.print("検出されたI2Cデバイス: ");
  byte error, address;
  int deviceCount = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("0x");
      Serial.print(address, HEX);
      Serial.print(" ");
      deviceCount++;
    }
  }
  Serial.println();
  
  // センサーの初期化
  if (!setupSensors()) {
    Serial.println("センサーの初期化に失敗しました");
    while (1) {
      digitalWrite(ledPin, HIGH);
      delay(200);
      digitalWrite(ledPin, LOW);
      delay(200);
    }
  }
  
  // BLEの初期化
  setupBLE();
  
  Serial.println("セットアップ完了");
}

void loop() {
  // センサーデータの読み取り
  static unsigned long lastRead = 0;
  static int count = 0;
  
  if (millis() - lastRead >= 100) {  // 10Hzの更新レート
    lastRead = millis();
    count++;
    
    // 圧力センサーの読み取り
    sensorData.pressure = readPressure() / 5000.0;
    
    // 加速度計の読み取り
    readAccel(&sensorData.ax, &sensorData.ay, &sensorData.az);
    
    // ジャイロスコープの読み取り
    readGyro(&sensorData.gx, &sensorData.gy, &sensorData.gz);
    
    // 定期的にデータを表示（5回に1回）
    if (count % 5 == 0) {
      Serial.print("T:"); Serial.print(millis());
      Serial.print(" P:"); Serial.println(sensorData.pressure, 6);
      Serial.print("ジャイロ - X:"); Serial.print(sensorData.gx, 6);
      Serial.print(" Y:"); Serial.print(sensorData.gy, 6);
      Serial.print(" Z:"); Serial.println(sensorData.gz, 6);
      Serial.print("加速度 - X:"); Serial.print(sensorData.ax, 6);
      Serial.print(" Y:"); Serial.print(sensorData.ay, 6);
      Serial.print(" Z:"); Serial.println(sensorData.az, 6);
    }
    
    // アドバタイジングデータを更新
    updateAdvertisingData();
  }
  
  // CPU負荷軽減のための小さな遅延
  delay(10);
}

bool setupSensors() {
  bool success = true;
  
  // 圧力センサーの初期化
  writeRegister(PRESSURE_ADDRESS, PRESSURE_REG_RESET, 0x80);
  delay(3);
  writeRegister(PRESSURE_ADDRESS, PRESSURE_REG_AVG, 0x39);
  delay(10);
  writeRegister(PRESSURE_ADDRESS, PRESSURE_REG_MEASURE, 0xA5);
  delay(10);
  
  // MPU-6050の初期化
  writeRegister(MPU6050_ADDRESS, 0x6B, 0x00);  // スリープモード解除
  delay(10);
  writeRegister(MPU6050_ADDRESS, 0x1B, 0x00);  // ジャイロスケール ±250°/s
  delay(10);
  writeRegister(MPU6050_ADDRESS, 0x1C, 0x00);  // 加速度スケール ±2g
  delay(10);
  
  Serial.println("センサー初期化完了");
  return success;
}

void setupBLE() {
  // BLEデバイスの作成
  BLEDevice::init("PicoTest");

  // BLEサーバーの作成
  pServer = BLEDevice::createServer();

  // BLEサービスの作成
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // BLE特性の作成（接続時に使用するため）
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // サービスの開始
  pService->start();

  // アドバタイジングの設定
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x06);  // iPhoneの接続問題を修正
  
  // アドバタイジング開始
  BLEDevice::startAdvertising();
  Serial.println("BLEが'PicoTest'として初期化され、アドバタイズ中");
}

void writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.print("I2C書き込みエラー (0x");
    Serial.print(addr, HEX);
    Serial.print("): ");
    Serial.println(error);
  }
}

uint32_t readPressure() {
  Wire.beginTransmission(PRESSURE_ADDRESS);
  Wire.write(PRESSURE_REG_POUT);
  if (Wire.endTransmission(false) != 0) {
    Serial.println("圧力読み取りエラー");
    return 0;
  }
  
  Wire.requestFrom(PRESSURE_ADDRESS, 3);
  if (Wire.available() < 3) {
    Serial.println("圧力センサーからのデータが不足しています");
    return 0;
  }
  
  uint32_t msb = Wire.read();
  uint32_t lsb = Wire.read();
  uint32_t xlsb = Wire.read();
  
  return (msb << 16) | (lsb << 8) | xlsb;
}

void readAccel(float *x, float *y, float *z) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    Serial.println("加速度計読み取りエラー");
    *x = *y = *z = 0;
    return;
  }
  
  Wire.requestFrom(MPU6050_ADDRESS, 6);
  if (Wire.available() < 6) {
    Serial.println("加速度計からのデータが不足しています");
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

void readGyro(float *x, float *y, float *z) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(GYRO_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    Serial.println("ジャイロスコープ読み取りエラー");
    *x = *y = *z = 0;
    return;
  }
  
  Wire.requestFrom(MPU6050_ADDRESS, 6);
  if (Wire.available() < 6) {
    Serial.println("ジャイロスコープからのデータが不足しています");
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
