import asyncio
import datetime
import csv
import os
from bleak import BleakScanner
from collections import deque, OrderedDict
import time
import struct

# デバイス名
DEVICE_NAME = "PicoTest"

# データをCSVに保存するための設定
CSV_FILENAME = f"{datetime.datetime.now().strftime('%Y%m%d%H%M%S')}_sensor_data.csv"
SAVE_TO_CSV = True
csv_file = None
csv_writer = None

# データ受信カウンター
data_count = 0
last_process_time = time.time()
last_status_time = time.time()

# データ構造設定
PACKET_ID_INDEX = 2  # パケットIDの位置（0から始まる）
DATA_START_INDEX = 3  # データの開始位置
BYTES_PER_RECORD = 8  # 各データレコードのバイト数
MAX_DATA_ENTRIES = 3  # パケットあたりの最大データエントリ数

# データ処理用のバッファ
# キー: パケットID, 値: 受信したデータ（重複を含む）
received_data_buffer = OrderedDict()
# 重複を排除したデータキュー（実際に保存するデータ）
processed_data_queue = deque(maxlen=1000)
# 最後に処理したパケットID
last_processed_id = None

# CSVファイルを初期化する関数
def init_csv():
    global csv_file, csv_writer
    file_exists = os.path.isfile(CSV_FILENAME)
    csv_file = open(CSV_FILENAME, 'a', newline='', buffering=1)  # 行バッファリング
    csv_writer = csv.writer(csv_file)
    if not file_exists:
        csv_writer.writerow(['timestamp', 'packet_id', 'pressure', 'accel_x', 'accel_y', 'accel_z'])

# デバイス検出時のコールバック関数
def detection_callback(device, advertisement_data):
    global received_data_buffer, last_processed_id
    
    if device.name != DEVICE_NAME:
        return
        
    # 製造者データを確認
    manufacturer_data = advertisement_data.manufacturer_data
    if not manufacturer_data:
        return
        
    # 現在のタイムスタンプを記録
    timestamp = datetime.datetime.now()
    
    for company_id, data in manufacturer_data.items():
        # 最低限必要なデータ長をチェック（識別子2バイト + パケットID1バイト + データ最低8バイト）
        if len(data) < 11 or data[0] != 0xAB or data[1] != 0xCD:
            continue
            
        try:
            # パケットIDを取得
            packet_id = data[PACKET_ID_INDEX]
            
            # 各データエントリを解析
            entries = []
            for i in range(MAX_DATA_ENTRIES):
                start_idx = DATA_START_INDEX + (i * BYTES_PER_RECORD)
                
                # データ部分の範囲をチェック
                if start_idx + BYTES_PER_RECORD > len(data):
                    break
                    
                # データが全て0の場合はスキップ（未使用エントリ）
                if all(b == 0 for b in data[start_idx:start_idx+BYTES_PER_RECORD]):
                    continue
                
                # センサーデータの解析
                pressure = int.from_bytes(data[start_idx:start_idx+2], 'little', signed=True) / 10
                accel_x = int.from_bytes(data[start_idx+2:start_idx+4], 'little', signed=True) / 10000
                accel_y = int.from_bytes(data[start_idx+4:start_idx+6], 'little', signed=True) / 10000
                accel_z = int.from_bytes(data[start_idx+6:start_idx+8], 'little', signed=True) / 10000
                
                # 有効なデータのみを追加
                if pressure != 0 or accel_x != 0 or accel_y != 0 or accel_z != 0:
                    entries.append((pressure, accel_x, accel_y, accel_z))
            
            # データがあれば受信バッファに追加
            if entries:
                if packet_id not in received_data_buffer:
                    received_data_buffer[packet_id] = []
                    
                # タイムスタンプとパケットID、データエントリを受信バッファに追加
                for entry in entries:
                    received_data_buffer[packet_id].append((timestamp, packet_id, *entry))
                
                # バッファが大きくなりすぎないように古いデータを削除
                if len(received_data_buffer) > 20:  # 最大20個のパケットIDを保持
                    oldest_key = next(iter(received_data_buffer))
                    del received_data_buffer[oldest_key]
                
                # デバッグ出力
                print(f"パケットID {packet_id} を受信、データエントリ {len(entries)} 個")
                
            return  # 早期リターン
                
        except Exception as e:
            print(f"データ解析エラー: {e}")
            return

# データ処理関数
async def process_data():
    global data_count, received_data_buffer, processed_data_queue, last_processed_id, last_process_time, last_status_time
    
    start_time = time.time()
    last_process_time = start_time
    
    while True:
        try:
            # 受信バッファから重複を排除してデータを処理
            process_received_data()
            
            # 処理済みキューからデータを取り出してCSVに保存
            save_processed_data()
            
            # 5秒ごとに統計情報を表示
            current_time = time.time()
            if current_time - last_status_time >= 5.0:
                elapsed = current_time - start_time
                rate = data_count / elapsed if elapsed > 0 else 0
                buffer_size = sum(len(entries) for entries in received_data_buffer.values())
                
                print(f"測定開始からの平均データレート: {rate:.2f}Hz (目標: 10Hz)")
                print(f"合計: {data_count}件のデータを処理")
                print(f"受信バッファ: {len(received_data_buffer)}パケット、{buffer_size}エントリ")
                print(f"処理済みキュー: {len(processed_data_queue)}エントリ")
                
                # バッファの内容をデバッグ出力（最大5エントリ）
                if received_data_buffer:
                    print("最新の受信データ:")
                    for packet_id, entries in list(received_data_buffer.items())[-5:]:
                        print(f"  ID:{packet_id} → {len(entries)}エントリ")
                
                last_status_time = current_time
                
                # CSVファイルを確実にフラッシュ
                if csv_file:
                    csv_file.flush()
                    os.fsync(csv_file.fileno())
        
        except Exception as e:
            print(f"データ処理エラー: {e}")
        
        # 短い待機でCPU使用率を抑える
        await asyncio.sleep(0.5)  # 0.5秒の待機

# 受信データバッファから重複を排除して処理
def process_received_data():
    global received_data_buffer, processed_data_queue, last_processed_id
    
    # データがなければ何もしない
    if not received_data_buffer:
        return
    
    # パケットIDを順番に処理（数値的に連続していなくても良い）
    packet_ids = sorted(received_data_buffer.keys())
    
    for packet_id in packet_ids:
        # 既に処理済みのパケットはスキップ
        if last_processed_id is not None:
            # 8ビット循環を考慮した比較（255の次は0）
            if (packet_id == last_processed_id or 
                (last_processed_id > 200 and packet_id < 50 and abs(packet_id + 256 - last_processed_id) > abs(packet_id - last_processed_id))):
                continue
        
        # このパケットのデータエントリを処理
        entries = received_data_buffer[packet_id]
        
        # 同じエントリでも受信時間が違う場合があるため、データ内容で重複を排除
        unique_entries = {}
        for entry in entries:
            # キーはデータ部分のみ（タイムスタンプとパケットIDを除く）
            data_key = entry[2:]  # (pressure, accel_x, accel_y, accel_z)
            
            # 最も新しいタイムスタンプでエントリを上書き
            if data_key not in unique_entries or entry[0] > unique_entries[data_key][0]:
                unique_entries[data_key] = entry
        
        # 重複を排除したエントリを処理済みキューに追加
        for entry in unique_entries.values():
            processed_data_queue.append(entry)
        
        # 処理済みとしてマーク
        last_processed_id = packet_id

# 処理済みデータをCSVに保存
def save_processed_data():
    global data_count, processed_data_queue
    
    buffer_processed = 0
    
    # 処理済みキューからデータを取り出して保存
    while processed_data_queue:
        entry = processed_data_queue.popleft()
        timestamp, packet_id, pressure, accel_x, accel_y, accel_z = entry
        data_count += 1
        buffer_processed += 1
        
        # 表示を間引く
        if data_count % 10 == 0:
            print(f"#{data_count} ID:{packet_id} P:{pressure:.1f}hPa, A:({accel_x:.2f},{accel_y:.2f},{accel_z:.2f})")
        
        # CSVに保存
        if SAVE_TO_CSV and csv_writer:
            timestamp_str = timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')
            csv_writer.writerow([timestamp_str, packet_id, pressure, accel_x, accel_y, accel_z])
    
    # バッファ処理数をログ出力（デバッグ用）
    if buffer_processed > 5:
        print(f"処理済みキューから{buffer_processed}個のデータを保存しました")

# メイン関数
async def main():
    try:
        # CSVファイルを初期化
        if SAVE_TO_CSV:
            init_csv()
        
        print(f"{DEVICE_NAME}スキャン中... 拡張パケット形式（27バイト）... Ctrl+Cで終了")
        
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