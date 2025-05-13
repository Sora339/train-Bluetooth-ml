import asyncio
import datetime
import csv
import os
from bleak import BleakScanner
from collections import deque
import time

# デバイス名
DEVICE_NAME = "PicoTest"

# データをCSVに保存するための設定
CSV_FILENAME = f"{datetime.datetime.now().strftime('%Y%m%d%H%M%S')}_sensor_data.csv"
SAVE_TO_CSV = True
csv_file = None
csv_writer = None

# データ受信カウンター
data_count = 0
data_buffer = deque(maxlen=100)  # 一時バッファ
last_process_time = time.time()
last_status_time = time.time()

# CSVファイルを初期化する関数
def init_csv():
    global csv_file, csv_writer
    file_exists = os.path.isfile(CSV_FILENAME)
    csv_file = open(CSV_FILENAME, 'a', newline='', buffering=1)  # 行バッファリング
    csv_writer = csv.writer(csv_file)
    if not file_exists:
        csv_writer.writerow(['timestamp', 'pressure', 'accel_x', 'accel_y', 'accel_z'])

# デバイス検出時のコールバック関数
def detection_callback(device, advertisement_data):
    global data_buffer
    
    if device.name != DEVICE_NAME:
        return
        
    # 製造者データを確認
    manufacturer_data = advertisement_data.manufacturer_data
    if not manufacturer_data:
        return
        
    # 現在のタイムスタンプを記録
    timestamp = datetime.datetime.now()
    
    for company_id, data in manufacturer_data.items():
        if len(data) < 8:
            continue
            
        try:
            # センサーデータの解析
            pressure = int.from_bytes(data[0:2], 'little', signed=True) / 10
            accel_x = int.from_bytes(data[2:4], 'little', signed=True) / 10000
            accel_y = int.from_bytes(data[4:6], 'little', signed=True) / 10000
            accel_z = int.from_bytes(data[6:8], 'little', signed=True) / 10000
            
            # バッファにデータを追加 (処理を分離)
            data_buffer.append((timestamp, pressure, accel_x, accel_y, accel_z))
            return  # 早期リターン
                
        except Exception as e:
            print(f"データ解析エラー: {e}")
            return

# データ処理関数 (コールバックから分離)
async def process_data():
    global data_count, data_buffer, last_process_time, last_status_time
    
    start_time = time.time()
    last_process_time = start_time
    
    while True:
        # バッファからデータを取り出して処理
        buffer_processed = 0
        while data_buffer:
            timestamp, pressure, accel_x, accel_y, accel_z = data_buffer.popleft()
            data_count += 1
            buffer_processed += 1
            
            # 表示を間引く
            if data_count % 10 == 0:
                print(f"#{data_count} P:{pressure:.1f}hPa, A:({accel_x:.2f},{accel_y:.2f},{accel_z:.2f})")
            
            # CSVに保存
            if SAVE_TO_CSV and csv_writer:
                timestamp_str = timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')
                csv_writer.writerow([timestamp_str, pressure, accel_x, accel_y, accel_z])
        
        # バッファ処理数をログ出力（デバッグ用）
        if buffer_processed > 5:
            print(f"バッファから{buffer_processed}個のデータを処理しました")
        
        # 5秒ごとに統計情報を表示
        current_time = time.time()
        if current_time - last_status_time >= 5.0:
            elapsed = current_time - start_time
            rate = data_count / elapsed if elapsed > 0 else 0
            print(f"測定開始からの平均データレート: {rate:.2f}Hz (目標: 10Hz)")
            print(f"合計: {data_count}件 バッファサイズ: {len(data_buffer)}件")
            last_status_time = current_time
            
            # CSVファイルを確実にフラッシュ
            if csv_file:
                csv_file.flush()
                os.fsync(csv_file.fileno())
        
        # 短い待機でCPU使用率を抑える
        await asyncio.sleep(3)  # 1sの待機

# メイン関数
async def main():
    try:
        # CSVファイルを初期化
        if SAVE_TO_CSV:
            init_csv()
        
        print(f"{DEVICE_NAME}スキャン中... Ctrl+Cで終了")
        
        # スキャナーの設定
        scanner = BleakScanner()
        scanner.register_detection_callback(detection_callback)
        
        # データ処理タスクを開始
        processing_task = asyncio.create_task(process_data())
        
        # スキャンを開始
        await scanner.start()
        print("スキャン開始しました。データを待機中...")
        
        # メインループを維持
        while True:
            await asyncio.sleep(1)
            
    except KeyboardInterrupt:
        print("\n終了")
    finally:
        await scanner.stop()
        if csv_file:
            csv_file.close()
            print(f"データ{data_count}件を{CSV_FILENAME}に保存")

if __name__ == "__main__":  
    # 実行
    asyncio.run(main())