import asyncio
import datetime
import csv
import os
import struct
from bleak import BleakScanner
import time
import binascii

# デバイス名
DEVICE_NAME = "PicoTest"

# データをCSVに保存するための設定
CSV_FILENAME = f"{datetime.datetime.now().strftime('%Y%m%d%H%M%S')}_sensor_data.csv"
SAVE_TO_CSV = True
csv_file = None
csv_writer = None

# 最新のセンサーデータを保持するグローバル変数
latest_data = {
    'timestamp': '',
    'P': 0,    # 圧力
    'AX': 0, 'AY': 0, 'AZ': 0,  # 加速度
    'GX': 0, 'GY': 0, 'GZ': 0   # ジャイロ
}

# リアルタイムでデータを表示する関数
def print_data():
    """センサーデータを整形して表示する"""
    print(f"=== センサーデータ {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===")
    
    # 各種センサー値の表示
    print(f"圧力: {latest_data.get('P', 'N/A')} hPa")
    
    print("\n加速度:")
    print(f"  X: {latest_data.get('AX', 'N/A')}")
    print(f"  Y: {latest_data.get('AY', 'N/A')}")
    print(f"  Z: {latest_data.get('AZ', 'N/A')}")
    
    print("\nジャイロ:")
    print(f"  X: {latest_data.get('GX', 'N/A')}°/s")
    print(f"  Y: {latest_data.get('GY', 'N/A')}°/s")
    print(f"  Z: {latest_data.get('GZ', 'N/A')}°/s")
    
    print("\n(Ctrl+Cで終了)")

# CSVファイルを初期化する関数
def init_csv():
    """CSVファイルを初期化し、ヘッダーを書き込む"""
    global csv_file, csv_writer
    
    file_exists = os.path.isfile(CSV_FILENAME)
    csv_file = open(CSV_FILENAME, 'a', newline='')
    csv_writer = csv.writer(csv_file)
    
    # ファイルが存在しない場合はヘッダーを書き込む
    if not file_exists:
        csv_writer.writerow([
            'timestamp', 'pressure', 
            'accel_x', 'accel_y', 'accel_z', 
            'gyro_x', 'gyro_y', 'gyro_z'
        ])

# CSVにデータを書き込む関数
def write_to_csv():
    """データをCSVファイルに書き込む"""
    if not SAVE_TO_CSV or csv_writer is None:
        return
        
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
    latest_data['timestamp'] = timestamp
    
    csv_writer.writerow([
        timestamp,
        latest_data.get('P', ''),
        latest_data.get('AX', ''),
        latest_data.get('AY', ''),
        latest_data.get('AZ', ''),
        latest_data.get('GX', ''),
        latest_data.get('GY', ''),
        latest_data.get('GZ', '')
    ])
    csv_file.flush()  # すぐにディスクに書き込む

# アドバタイジングデータのバイト列をダンプ
def dump_bytes(data):
    """バイト列を16進数で表示"""
    return ' '.join([f"{b:02X}" for b in data])

            
# detection_callback関数を修正
def detection_callback(device, advertisement_data):
    """BLEデバイス検出時のコールバック"""
    if device.name == DEVICE_NAME:
        print(f"\n===== デバイス検出: {device.name} ({device.address}) =====")
        print(f"RSSI: {device.rssi}dB")
        
        # アドバタイズデータの属性を直接参照（__dict__ではなく）
        print("\n--- アドバタイズデータの属性 ---")
        # 主要な属性のみチェック
        for attr_name in ['manufacturer_data', 'service_data', 'service_uuids', 'local_name']:
            attr_value = getattr(advertisement_data, attr_name, None)
            if attr_value:
                print(f"{attr_name}: {attr_value}")
        
        # 製造者データがあるか確認
        manufacturer_data = advertisement_data.manufacturer_data
        if manufacturer_data:
            print("\n--- 製造者データの詳細 ---")
            for company_id, data in manufacturer_data.items():
                print(f"会社ID: {company_id} (0x{company_id:04X})")
                
                # 統合した関数を呼び出し
                if parse_sensor_data(data):
                    write_to_csv()
                    return
        
        # 通常のアドバタイズペイロードをチェック
        service_data = advertisement_data.service_data
        if service_data:
            print("\n--- サービスデータの詳細 ---")
            for uuid, data in service_data.items():
                print(f"UUID: {uuid}")
                
                # 統合した関数を呼び出し
                if parse_sensor_data(data):
                    write_to_csv()
                    return

# デバイスからのデータを解析・表示する統合関数
def parse_sensor_data(raw_bytes):
    """センサーデータを解析して表示する統合関数"""
    print("\n--- センサーデータの解析 ---")
    
    # データの基本情報を表示
    print(f"データ長: {len(raw_bytes)}バイト")
    print(f"データ (Hex): {dump_bytes(raw_bytes)}")
    
    # バイト列の二進数表現
    binary_repr = ' '.join([f'{b:08b}' for b in raw_bytes])
    print(f"データ (バイナリ): {binary_repr}")
    
    # バイト列の長さをチェック
    if len(raw_bytes) < 8:
        print("データが不十分: 少なくとも8バイト必要（圧力+3軸加速度）")
        return False
    
    try:
        # 2バイトずつのデータを解析（詳細表示）
        print("\n位置 | バイト1 | バイト2 | リトルE整数 | ビッグE整数 | リトルE浮動小数点 | ビッグE浮動小数点")
        print("-----|--------|--------|------------|------------|-----------------|---------------")
        
        pairs = []
        for i in range(0, len(raw_bytes)-1, 2):
            if i+1 < len(raw_bytes):
                # リトルエンディアンとビッグエンディアンの両方で解析
                little_endian = int.from_bytes(raw_bytes[i:i+2], 'little', signed=True)
                big_endian = int.from_bytes(raw_bytes[i:i+2], 'big', signed=True)
                
                # 10000で割った値（浮動小数点）
                little_float = little_endian / 10000.0
                big_float = big_endian / 10000.0
                
                pairs.append((i, raw_bytes[i], raw_bytes[i+1], 
                             little_endian, big_endian, 
                             little_float, big_float))
                
                print(f"{i:4d} | 0x{raw_bytes[i]:02X}   | 0x{raw_bytes[i+1]:02X}   | {little_endian:10d} | {big_endian:10d} | {little_float:16.6f} | {big_float:16.6f}")
        
        # センサー値の解析と更新
        latest_data['P'] = int.from_bytes(raw_bytes[0:2], 'little', signed=True) / 10000.0 * 1000
        latest_data['AX'] = int.from_bytes(raw_bytes[2:4], 'little', signed=True) / 10000.0
        latest_data['AY'] = int.from_bytes(raw_bytes[4:6], 'little', signed=True) / 10000.0
        latest_data['AZ'] = int.from_bytes(raw_bytes[6:8], 'little', signed=True) / 10000.0
        
        # ジャイロデータがある場合（オプション）
        if len(raw_bytes) >= 14:
            latest_data['GX'] = int.from_bytes(raw_bytes[8:10], 'little', signed=True) / 10000.0
            latest_data['GY'] = int.from_bytes(raw_bytes[10:12], 'little', signed=True) / 10000.0
            latest_data['GZ'] = int.from_bytes(raw_bytes[12:14], 'little', signed=True) / 10000.0
        
        # 解析結果のサマリーを表示
        print("\n--- 解析結果 ---")
        print(f"圧力 (hPa): {latest_data['P']}")
        print(f"加速度 X: {latest_data['AX']}")
        print(f"加速度 Y: {latest_data['AY']}")
        print(f"加速度 Z: {latest_data['AZ']}")
        
        if len(raw_bytes) >= 14:
            print(f"ジャイロ X (°/s): {latest_data['GX']}")
            print(f"ジャイロ Y (°/s): {latest_data['GY']}")
            print(f"ジャイロ Z (°/s): {latest_data['GZ']}")
        
        # センサーデータの整形表示
        print_data()
        return True
        
    except Exception as e:
        print(f"データ解析エラー: {e}")
        return False

# メイン関数
async def main():
    """メインのスキャンループ"""
    try:
        # CSVファイルを初期化
        if SAVE_TO_CSV:
            init_csv()
        
        print(f"{DEVICE_NAME}をスキャン中...")
        
        # 継続的なスキャンを開始
        scanner = BleakScanner()
        scanner.register_detection_callback(detection_callback)
        
        await scanner.start()
        print("スキャン開始... (Ctrl+Cで終了)")
        
        # スキャンを継続
        while True:
            await asyncio.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\nプログラムを終了します")
    finally:
        # スキャンを停止
        await scanner.stop()
        
        # CSVファイルを閉じる
        if csv_file:
            csv_file.close()
            print(f"データは {CSV_FILENAME} に保存されました")

if __name__ == "__main__":
    # セットアップ方法を表示
    print("=== センサーデータスキャン ===")
    print(f"デバイス名: {DEVICE_NAME}")
    print(f"データ保存: {'有効' if SAVE_TO_CSV else '無効'} ({CSV_FILENAME})")
    print("-------------------------------")
    
    # メインループを実行
    asyncio.run(main())