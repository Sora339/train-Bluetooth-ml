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

# アドバタイジングデータからセンサー値を解析
def parse_manufacturer_data(manufacturer_data):
    try:
        # データが既にバイナリ形式の場合はそのまま使用
        if isinstance(manufacturer_data, bytes):
            raw_bytes = manufacturer_data
        else:
            # UTF-8文字列（16進表現）としてデコード
            hex_str = manufacturer_data.decode('utf-8')
            raw_bytes = binascii.unhexlify(hex_str)
        
        print(f"デコード成功（{len(raw_bytes)} bytes）: {dump_bytes(raw_bytes)}")

        if len(raw_bytes) < 9:
            print("加速度Z軸までのデータが不足しています")
            return False

        latest_data['P'] = int.from_bytes(raw_bytes[1:3], 'little', signed=True) / 10000.0 * 1000
        latest_data['AX'] = int.from_bytes(raw_bytes[3:5], 'little', signed=True) / 10000.0
        latest_data['AY'] = int.from_bytes(raw_bytes[5:7], 'little', signed=True) / 10000.0
        latest_data['AZ'] = int.from_bytes(raw_bytes[7:9], 'little', signed=True) / 10000.0

        latest_data['GX'] = latest_data['GY'] = latest_data['GZ'] = 0.0  # ジャイロは送ってない

        print_data()
        return True

    except Exception as e:
        print(f"データ解析エラー: {e}")
        return False


# アドバタイジングパケットの検出コールバック
def detection_callback(device, advertisement_data):
    """BLEデバイス検出時のコールバック"""
    if device.name == DEVICE_NAME:
        # 製造者データがあるか確認
        manufacturer_data = advertisement_data.manufacturer_data
        
        if manufacturer_data:
            for company_id, data in manufacturer_data.items():
                print(f"受信データ: 会社ID={company_id}, データ長={len(data)}バイト, データ={dump_bytes(data)}")
                # データを解析
                if parse_manufacturer_data(data):
                    # データを書き込み、表示を更新
                    write_to_csv()
                    print_data()
                    return
        
        # 通常のアドバタイズペイロードをチェック
        service_data = advertisement_data.service_data
        if service_data:
            for uuid, data in service_data.items():
                print(f"サービスデータ: UUID={uuid}, データ長={len(data)}バイト, データ={dump_bytes(data)}")
                if parse_manufacturer_data(data):
                    write_to_csv()
                    print_data()
                    return

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