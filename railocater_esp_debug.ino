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


// updateAdvertisingData関数の修正版
void updateAdvertisingData() {
  // センサーデータの詳細をダンプ
  Serial.println("\n===== センサーデータの値とバイナリ表現 =====");
  Serial.print("圧力(hPa): "); Serial.print(sensorData.pressure, 6);
  
  int16_t pressureData = (int16_t)(sensorData.pressure);
  Serial.println(pressureData);
  
  Serial.print("加速度X: "); Serial.print(sensorData.ax, 6);
  int16_t accelX = (int16_t)(sensorData.ax * 10000);
  Serial.print(" -> 変換値(x10000): "); Serial.println(accelX);
  
  Serial.print("加速度Y: "); Serial.print(sensorData.ay, 6);
  int16_t accelY = (int16_t)(sensorData.ay * 10000);
  Serial.print(" -> 変換値(x10000): "); Serial.println(accelY);
  
  Serial.print("加速度Z: "); Serial.print(sensorData.az, 6);
  int16_t accelZ = (int16_t)(sensorData.az * 10000);
  Serial.print(" -> 変換値(x10000): "); Serial.println(accelZ);

  // ビット表現を出力
  Serial.println("\n各値のバイト表現（リトルエンディアン）:");
  Serial.print("圧力: 下位バイト=0x"); Serial.print(pressureData & 0xFF, HEX);
  Serial.print(", 上位バイト=0x"); Serial.println((pressureData >> 8) & 0xFF, HEX);
  
  Serial.print("加速度X: 下位バイト=0x"); Serial.print(accelX & 0xFF, HEX);
  Serial.print(", 上位バイト=0x"); Serial.println((accelX >> 8) & 0xFF, HEX);
  
  Serial.print("加速度Y: 下位バイト=0x"); Serial.print(accelY & 0xFF, HEX);
  Serial.print(", 上位バイト=0x"); Serial.println((accelY >> 8) & 0xFF, HEX);
  
  Serial.print("加速度Z: 下位バイト=0x"); Serial.print(accelZ & 0xFF, HEX);
  Serial.print(", 上位バイト=0x"); Serial.println((accelZ >> 8) & 0xFF, HEX);

  // アドバタイズデータのバイト列を構築
  // BLEライブラリによって会社IDが付加されるため注意が必要
  uint8_t sensorDataBytes[10]; // 会社IDを考慮して12バイトに変更
  
  // パディングバイト
  sensorDataBytes[0] = 0xAB;
  sensorDataBytes[1] = 0xCD;
  // 圧力データ - 最初の2バイトは会社IDで上書きされるため、
  // [2]に圧力の下位バイトを入れる
  sensorDataBytes[2] = pressureData & 0xFF;        // 圧力下位バイト
  sensorDataBytes[3] = (pressureData >> 8) & 0xFF; // 圧力上位バイト
  
  // 加速度データ - 会社ID分のオフセットを考慮
  sensorDataBytes[4] = accelX & 0xFF;
  sensorDataBytes[5] = (accelX >> 8) & 0xFF;
  
  sensorDataBytes[6] = accelY & 0xFF;
  sensorDataBytes[7] = (accelY >> 8) & 0xFF;
  
  sensorDataBytes[8] = accelZ & 0xFF;
  sensorDataBytes[9] = (accelZ >> 8) & 0xFF;

  // アドバタイズデータの内容を表示
  Serial.print("\nアドバタイズデータ(16進数): ");
  for (int i = 0; i < 10; i++) {
    Serial.printf("%02X ", sensorDataBytes[i]);
  }
  Serial.println();
  
  // 製造者データとしてBLEライブラリに渡す
  String manufacturerData;
  manufacturerData.reserve(10);
  
  for (int i = 0; i < 10; i++) {
    manufacturerData += (char)sensorDataBytes[i];
  }
  
  // BLEライブラリが内部でどのように処理するかを確認するためのデバッグ
  Serial.println("\n製造者データをBLEライブラリに渡す前のバイナリダンプ:");
  Serial.print("バイト長: "); Serial.println(manufacturerData.length());
  Serial.print("バイト (16進数): ");
  for (int i = 0; i < manufacturerData.length(); i++) {
    Serial.printf("%02X ", (uint8_t)manufacturerData[i]);
  }
  Serial.println();
  
  // BLEAdvertisementDataオブジェクトを作成
  BLEAdvertisementData advData;
  
  // 会社IDを指定して製造者データをセット
  advData.setManufacturerData(manufacturerData);
  
  // アドバタイジングを開始
  pAdvertising->setAdvertisementData(advData);
  pAdvertising->start();
  
  // LEDを点滅
  digitalWrite(ledPin, !digitalRead(ledPin));
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
  Wire.setClock(400000);
  
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
    
    // 圧力センサーの読み取り（修正: PaからhPaへの変換を含める）
    sensorData.pressure = readPressureHPa();
    
    // 加速度計の読み取り
    readAccel(&sensorData.ax, &sensorData.ay, &sensorData.az);
    
    // ジャイロスコープの読み取り
    readGyro(&sensorData.gx, &sensorData.gy, &sensorData.gz);
    
    // 定期的にデータを表示（5回に1回）
    if (count % 5 == 0) {
      Serial.print("T:"); Serial.print(millis());
      Serial.print(" P:"); Serial.println(sensorData.pressure, 2);
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

// writeRegister関数の追加
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
  Serial.println("BLEデバイス初期化: PicoTest");

  // BLEサーバーの作成
  pServer = BLEDevice::createServer();
  Serial.println("BLEサーバー作成");

  // BLEサービスの作成
  BLEService *pService = pServer->createService(SERVICE_UUID);
  Serial.println("BLEサービス作成: " SERVICE_UUID);

  // BLE特性の作成
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  Serial.println("BLE特性作成: " CHARACTERISTIC_UUID);

  // サービスの開始
  pService->start();
  Serial.println("BLEサービス開始");

  // アドバタイジングの設定
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x06);
  
  // IMPORTANT: 製造者IDを確認
  Serial.println("\n重要な設定情報:");
  Serial.print("製造者ID: 0x"); Serial.println(MANUFACTURER_ID, HEX);
  Serial.print("サービスUUID: "); Serial.println(SERVICE_UUID);
  
  // アドバタイジング開始
  BLEDevice::startAdvertising();
  Serial.println("BLEアドバタイジング開始");
  Serial.println("デバイス名: PicoTest");
}

// 生のセンサーデータを読み取り
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

// しまゆうへ:圧力の倍率調節よくわからんあとは頼んだ
float readPressureHPa() {
  uint32_t rawValue = readPressure();
  
  // センサーから取得した生の値をPaに変換（データシート: Pressure [Pa] = POUT[LSB] x 2）
  float pressurePa = rawValue * 2.0;
  
  // PaからhPaに変換（1 hPa = 100 Pa）
  float pressureHPa = pressurePa / 100.0;

  //倍率調整
  pressureHPa = pressureHPa / 10.0;
  
  return pressureHPa;
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