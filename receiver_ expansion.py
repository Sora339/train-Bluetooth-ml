import asyncio
import datetime
import csv
import os
from bleak import BleakScanner
from collections import deque
import time
import logging
import platform

# ロギング設定（レベルをINFOに上げてデバッグログを削減）
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# デバイス名
DEVICE_NAME = "PicoTest"

# データをCSVに保存するための設定
CSV_FILENAME = f"{datetime.datetime.now().strftime('%Y%m%d%H%M%S')}_sensor_data.csv"
SAVE_TO_CSV = True
csv_file = None
csv_writer = None

# データ受信カウンター
data_count = 0
last_status_time = time.time()

# データ構造設定
PACKET_ID_INDEX = 0     # パケットIDの位置（0から始まる、3バイト目）
DATA_START_INDEX = 1    # データの開始位置（パケットIDの後）
BYTES_PER_RECORD = 8    # 各データレコードのバイト数
MAX_DATA_ENTRIES = 8    # パケットあたりの最大データエントリ数（拡張）
TIMEDIFF_START_INDEX = 3 + (MAX_DATA_ENTRIES * BYTES_PER_RECORD)  # 時間差情報の開始位置

# データ処理用のバッファ
unique_data_buffer = {}  # 重複を排除したデータバッファ
processed_data_queue = deque(maxlen=1000)  # 処理済みデータキュー

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
    global unique_data_buffer, processed_data_queue
    
    # PicoTestデバイスのみを処理
    if device.name == DEVICE_NAME:
        # 製造者データを確認
        manufacturer_data = advertisement_data.manufacturer_data
        if not manufacturer_data:
            return
        
        # 製造者データの内容をログ
        for company_id, data in manufacturer_data.items():
            # データ長のチェック（最小サイズの確認）
            min_size = TIMEDIFF_START_INDEX + 1
            if len(data) < min_size:
                logger.warning(f"データサイズが小さすぎます: {len(data)} バイト")
                continue
                
            try:
                # 現在のタイムスタンプを記録
                timestamp = datetime.datetime.now()
                
                # パケットIDを取得（3バイト目）
                packet_id = data[PACKET_ID_INDEX]
                
                # 時間差情報を抽出
                time_diffs = []
                for i in range(MAX_DATA_ENTRIES - 1):
                    diff_index = TIMEDIFF_START_INDEX + i
                    if diff_index < len(data):
                        time_diff = data[diff_index] * 4 if data[diff_index] != 0xFF else 0
                        time_diffs.append(time_diff)
                    else:
                        time_diffs.append(0)
                
                # タイムスタンプの計算（最新データから遡る）
                data_timestamps = [timestamp]  # 最新データのタイムスタンプ
                
                # 時間差を使って過去のデータのタイムスタンプを計算
                current_ts = timestamp
                for diff in reversed(time_diffs):
                    if diff > 0:
                        prev_ts = current_ts - datetime.timedelta(milliseconds=diff)
                    else:
                        prev_ts = current_ts
                    
                    # 時間の逆転を防止
                    if prev_ts > current_ts:
                        prev_ts = current_ts
                        
                    data_timestamps.insert(0, prev_ts)
                    current_ts = prev_ts
                
                # タイムスタンプリストを確認
                if len(data_timestamps) != MAX_DATA_ENTRIES:
                    # 配列を正しいサイズに調整
                    while len(data_timestamps) < MAX_DATA_ENTRIES:
                        data_timestamps.append(data_timestamps[-1])
                
                # 各データエントリを解析
                entries = []
                
                # データエントリを解析（3バイト目以降のデータ）
                for i in range(MAX_DATA_ENTRIES):
                    offset = DATA_START_INDEX + (i * BYTES_PER_RECORD)
                    
                    # データ範囲のチェック
                    if offset + BYTES_PER_RECORD > len(data):
                        break
                    
                    try:
                        # センサーデータの解析
                        pressure = int.from_bytes(data[offset:offset+2], byteorder='little', signed=True) / 10
                        accel_x = int.from_bytes(data[offset+2:offset+4], byteorder='little', signed=True) / 10000
                        accel_y = int.from_bytes(data[offset+4:offset+6], byteorder='little', signed=True) / 10000
                        accel_z = int.from_bytes(data[offset+6:offset+8], byteorder='little', signed=True) / 10000
                        
                        # 有効なデータのみを追加
                        if is_valid_data(pressure, accel_x, accel_y, accel_z):
                            entries.append((data_timestamps[i], packet_id, pressure, accel_x, accel_y, accel_z))
                    except Exception as e:
                        logger.error(f"データ解析エラー: {e}")
                
                # データがあれば重複を排除して処理
                if entries:
                    # 新しいデータを見つけたカウント
                    new_data_count = 0
                    
                    # 重複を排除して追加
                    for entry in entries:
                        entry_time, entry_id, pressure, accel_x, accel_y, accel_z = entry
                        
                        # 一意のキーを作成
                        data_key = (entry_id, round(pressure, 1), round(accel_x, 3), round(accel_y, 3), round(accel_z, 3))
                        
                        # 既に処理済みかチェック
                        if data_key not in unique_data_buffer:
                            # 新しいデータをバッファとキューに追加
                            unique_data_buffer[data_key] = entry
                            processed_data_queue.append(entry)
                            new_data_count += 1
                    
                    # 処理済みキューからデータを保存
                    if new_data_count > 0:
                        save_processed_data()
                
            except Exception as e:
                logger.error(f"データ処理エラー: {e}")

# 有効なデータかをチェック
def is_valid_data(pressure, accel_x, accel_y, accel_z):
    # 値が全て0に近い場合はスキップ
    if abs(pressure) < 0.1 and abs(accel_x) < 0.0001 and abs(accel_y) < 0.0001 and abs(accel_z) < 0.0001:
        return False
    return True

# データ処理関数
async def process_data():
    global data_count, last_status_time
    
    start_time = time.time()
    
    while True:
        try:
            # 処理済みキューからデータを取り出してCSVに保存
            save_processed_data()
            
            # 10秒ごとに統計情報を表示
            current_time = time.time()
            if current_time - last_status_time >= 10.0:
                elapsed = current_time - start_time
                rate = data_count / elapsed if elapsed > 0 else 0
                
                logger.info(f"測定開始からの平均データレート: {rate:.2f}Hz (合計: {data_count}件)")
                
                # CSVファイルをフラッシュ
                if csv_file:
                    csv_file.flush()
                
                last_status_time = current_time
        
        except Exception as e:
            logger.error(f"データ処理エラー: {e}")
        
        # 待機時間を延長してCPU使用率を抑制
        await asyncio.sleep(1.0)

# 処理済みデータをCSVに保存
def save_processed_data():
    global data_count, processed_data_queue
    
    saved_count = 0
    
    # 処理済みキューからデータを取り出して保存
    while processed_data_queue:
        entry = processed_data_queue.popleft()
        timestamp, packet_id, pressure, accel_x, accel_y, accel_z = entry
        data_count += 1
        saved_count += 1
        
        # 表示を大幅に間引く（100件ごとに1回）
        if data_count % 100 == 0:
            time_str = timestamp.strftime('%H:%M:%S.%f')[:-3]
            logger.info(f"#{data_count} [{time_str}] ID:{packet_id} P:{pressure:.1f}hPa, A:({accel_x:.3f},{accel_y:.3f},{accel_z:.3f})")
        
        # CSVに保存
        if SAVE_TO_CSV and csv_writer:
            timestamp_str = timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')
            csv_writer.writerow([timestamp_str, packet_id, pressure, accel_x, accel_y, accel_z])

# BLE 5の拡張アドバタイズメントをスキャンするための設定
async def setup_ble5_scanner():
    # プラットフォームを確認
    system = platform.system()
    logger.info(f"実行環境: {system}")
    
    # BLE 5の拡張アドバタイズメントとコーディングPHYをサポートするためのスキャン設定
    scanner = BleakScanner(
        detection_callback=detection_callback,
        scanning_mode="active",  # アクティブスキャンを有効化
    )
    
    # プラットフォーム固有の設定
    try:
        if system == "Linux":
            # BlueZバックエンド（Linux）での拡張設定 - 適切なメソッドチェック
            if hasattr(scanner, "set_scanning_filter"):
                scanner.set_scanning_filter("Transport", "le")  # BLE専用
                scanner.set_scanning_filter("DuplicateData", False)  # 重複データを受信
                scanner.set_scanning_filter("PHY", "coded")  # コーディングPHY (S=8) を使用
                logger.info("Linux環境でコーディングPHY設定を有効化しました")
                
        elif system == "Windows":
            # Windows 10/11の場合の特別な設定
            if hasattr(scanner, "_backend") and hasattr(scanner._backend, "_adapter"):
                adapter = scanner._backend._adapter
                if hasattr(adapter, "SetLEScanFilterPolicy"):
                    # Windowsでの拡張設定
                    adapter.SetLEScanFilterPolicy(0)  # すべてのアドバタイズを受信
                    
                    # コーディングPHYをサポートする場合の設定
                    if hasattr(adapter, "SetScanningPHY"):
                        adapter.SetScanningPHY(0x03)  # 1M PHYとコーディングPHYの両方を有効化
                        logger.info("Windows環境でコーディングPHY設定を有効化しました")
                    
        elif system == "Darwin":  # macOS
            # macOSでは現在、BLE 5拡張機能のネイティブサポートが限られている
            logger.info("macOS環境では拡張スキャン設定が限定されている場合があります")
            # macOSではBleakの現在のバージョンで直接拡張スキャンを設定する方法はないが、
            # スキャン自体は動作する
    except Exception as e:
        logger.warning(f"BLE 5スキャン設定中にエラーが発生しました: {e}")
        logger.info("標準設定でスキャンを続行します")
    
    return scanner

# メイン関数
async def main():
    scanner = None
    try:
        # CSVファイルを初期化
        if SAVE_TO_CSV:
            init_csv()
        
        logger.info(f"{DEVICE_NAME}スキャン中... BLE 5の拡張アドバタイズメントに対応。Ctrl+Cで終了")
        
        # BLE 5対応のスキャナーを設定
        scanner = await setup_ble5_scanner()
        
        # データ処理タスクを開始
        processing_task = asyncio.create_task(process_data())
        
        # スキャンを開始
        await scanner.start()
        logger.info("BLE 5拡張アドバタイズメントスキャンを開始しました")
        
        # メインループを維持
        while True:
            await asyncio.sleep(1)
            
    except KeyboardInterrupt:
        logger.info("\n終了")
    except Exception as e:
        logger.error(f"エラーが発生しました: {e}")
    finally:
        if scanner:
            await scanner.stop()
        if csv_file:
            csv_file.close()
            logger.info(f"データ{data_count}件を{CSV_FILENAME}に保存")

if __name__ == "__main__":
    # 実行
    asyncio.run(main())