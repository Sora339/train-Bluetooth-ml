#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include <string>

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
unsigned long lastSendTime = 0;  // 最後の送信時刻
const unsigned long SEND_INTERVAL = 100;  // 10Hz = 100ms

// センサーデータ構造体
typedef struct {
  float pressure;
  float ax, ay, az;
  float gx, gy, gz;
} sensor_data_t;

sensor_data_t sensorData;

void updateAdvertisingData() {
  int16_t pressureData = (int16_t)(sensorData.pressure);
  int16_t accelX = (int16_t)(sensorData.ax * 10000);
  int16_t accelY = (int16_t)(sensorData.ay * 10000);
  int16_t accelZ = (int16_t)(sensorData.az * 10000);

  uint8_t sensorDataBytes[10]; 
  
  sensorDataBytes[0] = 0xAB;
  sensorDataBytes[1] = 0xCD;
  sensorDataBytes[2] = pressureData & 0xFF;
  sensorDataBytes[3] = (pressureData >> 8) & 0xFF;
  
  sensorDataBytes[4] = accelX & 0xFF;
  sensorDataBytes[5] = (accelX >> 8) & 0xFF;
  
  sensorDataBytes[6] = accelY & 0xFF;
  sensorDataBytes[7] = (accelY >> 8) & 0xFF;
  
  sensorDataBytes[8] = accelZ & 0xFF;
  sensorDataBytes[9] = (accelZ >> 8) & 0xFF;
  
  // Arduino String型を使用
  String manufacturerData;
  
  // センサーデータをStringに変換して追加
  for (int i = 0; i < 10; i++) {
    manufacturerData += (char)sensorDataBytes[i];
  }
  
  BLEAdvertisementData advData;
  advData.setManufacturerData(manufacturerData);
  
  pAdvertising->setAdvertisementData(advData);
  
  // 広告を確実に開始
  pAdvertising->start();
  
  digitalWrite(ledPin, !digitalRead(ledPin));
}

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  
  if (!setupSensors()) {
    while (1) {
      digitalWrite(ledPin, HIGH);
      delay(200);
      digitalWrite(ledPin, LOW);
      delay(200);
    }
  }
  
  setupBLE();
}

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
    
    // センサーデータ読み取り
    sensorData.pressure = readPressureHPa();
    readAccel(&sensorData.ax, &sensorData.ay, &sensorData.az);
    readGyro(&sensorData.gx, &sensorData.gy, &sensorData.gz);
    
    // アドバタイジングデータを更新
    updateAdvertisingData();
  }
}

void writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

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

float readPressureHPa() {
  uint32_t rawValue = readPressure();
  float pressurePa = rawValue * 2.0;
  float pressureHPa = pressurePa / 100.0;
  pressureHPa = pressureHPa / 10.0;
  
  return pressureHPa;
}

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