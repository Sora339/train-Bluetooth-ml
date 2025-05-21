import asyncio
import datetime
import csv
import os
from bleak import BleakScanner, BleakClient
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

# データ構造設定 - レガシーアドバタイズ対応
PACKET_ID_INDEX = 0     # パケットIDの位置（0から始まる、3バイト目）
DATA_START_INDEX = 1    # データの開始位置（パケットIDの後）
BYTES_PER_RECORD = 8    # 各データレコードのバイト数
MAX_DATA_ENTRIES = 3    # パケットあたりの最大データエントリ数（レガシー）
TIMEDIFF_START_INDEX = 3 + (MAX_DATA_ENTRIES * BYTES_PER_RECORD)  # 時間差情報の開始位置

# 製造者ID (実際に使われているID)
MANUFACTURER_ID = 0xCDCB  # 製造者ID (52651)

# デバイス情報を保持する辞書
found_devices = {}

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
    global unique_data_buffer, processed_data_queue, found_devices
    
    # デバイスログ
    if device.name and device.name not in found_devices:
        logger.info(f"デバイスを発見: {device.name} ({device.address})")
        found_devices[device.name] = device.address
    
    # PicoTestデバイスのみを処理
    if device.name == DEVICE_NAME:
        # 製造者データを確認
        manufacturer_data = advertisement_data.manufacturer_data
        if not manufacturer_data:
            return
        
        # 製造者データの内容をログ（最初の検出時のみ）
        for company_id, data in manufacturer_data.items():
            # 最初の検出時のみデータ構造をログ
            if company_id not in unique_data_buffer:
                logger.info(f"製造者ID: 0x{company_id:04X}, データ長: {len(data)}バイト")
                if debugMode:
                    hex_str = ' '.join([f'{b:02X}' for b in data])
                    logger.debug(f"HEX: {hex_str}")
            
            # データが小さすぎる場合はスキップ
            min_size = 4  # パケットIDとデータ1セットの最小サイズ
            if len(data) < min_size:
                continue
                
            try:
                # 現在のタイムスタンプを記録
                timestamp = datetime.datetime.now()
                
                # パケットIDを取得（3バイト目）
                packet_id = data[PACKET_ID_INDEX]
                
                # 実際のデータエントリ数を計算
                actual_entries = min((len(data) - 3) // BYTES_PER_RECORD, MAX_DATA_ENTRIES)
                
                # 時間差情報を抽出
                time_diffs = []
                for i in range(actual_entries - 1):
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
                if len(data_timestamps) != actual_entries:
                    # 配列を正しいサイズに調整
                    while len(data_timestamps) < actual_entries:
                        data_timestamps.append(data_timestamps[-1])
                
                # 各データエントリを解析
                entries = []
                
                # データエントリを解析（3バイト目以降のデータ）
                for i in range(actual_entries):
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
                
                # 発見済みデバイスのリスト表示
                if found_devices and len(found_devices) > 0:
                    logger.info(f"発見済みデバイス: {', '.join(found_devices.keys())}")
        
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
        if data_count % 100 == 0 or data_count == 1:
            time_str = timestamp.strftime('%H:%M:%S.%f')[:-3]
            logger.info(f"#{data_count} [{time_str}] ID:{packet_id} P:{pressure:.1f}hPa, A:({accel_x:.3f},{accel_y:.3f},{accel_z:.3f})")
        
        # CSVに保存
        if SAVE_TO_CSV and csv_writer:
            timestamp_str = timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')
            csv_writer.writerow([timestamp_str, packet_id, pressure, accel_x, accel_y, accel_z])

# すべてのBLEデバイスをスキャンする関数
async def scan_for_devices(timeout=10.0):
    """指定された時間だけBLEデバイスをスキャンし、結果を返す"""
    logger.info(f"{timeout}秒間スキャン中...")
    devices = await BleakScanner.discover(timeout=timeout)
    return devices

# macOS用の補助関数
async def connect_to_device(address, callback):
    """デバイスへの接続とコールバック設定"""
    try:
        client = BleakClient(address)
        await client.connect()
        logger.info(f"デバイス {address} に接続しました")
        return client
    except Exception as e:
        logger.error(f"接続エラー: {e}")
        return None

# BLEスキャナーをセットアップ
async def setup_scanner():
    """プラットフォーム対応のBLEスキャナーをセットアップ"""
    global debugMode
    
    # デバッグモード
    debugMode = False
    
    # プラットフォームを確認
    system = platform.system()
    logger.info(f"実行環境: {system}")
    
    # スキャンモード - アクティブモードを使用
    scan_mode = "active"
    
    # スキャナーを作成
    scanner = BleakScanner(
        detection_callback=detection_callback,
        scanning_mode=scan_mode
    )
    
    # プラットフォーム固有の設定
    try:
        if system == "Linux":
            # LinuxではBlueZを使用
            logger.info("Linux環境でスキャン設定を適用します")
            # 一部のLinuxでは特別な設定が必要な場合がある
            
        elif system == "Windows":
            # Windowsでは.NETベースのBluetoothLEを使用
            logger.info("Windows環境でスキャン設定を適用します")
            # Windowsではデフォルトで重複パケットをフィルタリングするので無効にする
            
        elif system == "Darwin":  # macOS
            # macOSではCore Bluetoothを使用
            logger.info("macOS環境でスキャン設定を適用します")
            # macOSでは特別な設定は通常不要
    
    except Exception as e:
        logger.warning(f"プラットフォーム固有の設定中にエラー: {e}")
    
    return scanner

# メイン関数
async def main():
    global data_count
    
    scanner = None
    try:
        # CSVファイルを初期化
        if SAVE_TO_CSV:
            init_csv()
        
        logger.info(f"{DEVICE_NAME}スキャン中... Ctrl+Cで終了")
        
        # スキャナーのセットアップ
        scanner = await setup_scanner()
        
        # データ処理タスクを開始
        processing_task = asyncio.create_task(process_data())
        
        # スキャンを開始
        await scanner.start()
        logger.info("スキャンを開始しました")
        
        # 最初は1分間だけ全デバイスをスキャン
        if len(found_devices) == 0:
            logger.info("最初の30秒間はすべてのBLEデバイスを検索します...")
            await asyncio.sleep(30)
        
        # メインループを維持
        while True:
            # 定期的にスキャンステータスを確認
            if len(found_devices) == 0 and data_count == 0:
                # デバイスが見つからない場合は全デバイススキャンを実行
                logger.info("デバイスが見つかりません。すべてのBLEデバイスをスキャンします...")
                devices = await scan_for_devices(timeout=5.0)
                logger.info(f"{len(devices)}個のBLEデバイスを検出しました")
                
                for device in devices:
                    if device.name:
                        logger.info(f"  - {device.name} ({device.address})")
                        # macOSの場合は明示的に接続を試みる
                        if platform.system() == "Darwin" and device.name == DEVICE_NAME:
                            client = await connect_to_device(device.address, detection_callback)
                            if client:
                                logger.info(f"{device.name}に接続しました")
            
            # 通常の待機
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