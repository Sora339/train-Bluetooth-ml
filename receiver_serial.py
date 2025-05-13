import serial
import time
import csv
from datetime import datetime

# シリアルポートの設定
SERIAL_PORT = '/dev/cu.usbmodem14301'  # Windowsの場合。Macの場合は'/dev/tty.usbserial'などに変更
BAUD_RATE = 115200

def main():
    try:
        # シリアルポートを開く
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Serial port {SERIAL_PORT} opened successfully")
        
        # CSVファイル名に現在の日時を含める
        current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = f"sensor_data_{current_time}.csv"
        
        with open(csv_filename, 'w', newline='') as csvfile:
            # 最初の2行はヘッダーなのでスキップするフラグ
            header_skipped = 0
            # 新しいヘッダーを定義（実際の日時を含める）
            new_header = "date_time timestamp pressure accel_x accel_y accel_z gyro_x gyro_y gyro_z"
            
            try:
                print("Logging sensor data... Press Ctrl+C to stop.")
                csvfile.write(new_header + '\n')
                
                while True:
                    # シリアルポートからデータを読み取る
                    line = ser.readline().decode('utf-8').strip()
                    
                    if line:
                        # ヘッダー行の処理
                        if header_skipped < 2:
                            print(f"Header: {line}")
                            header_skipped += 1
                            continue
                        
                        # データ行の処理
                        # 現在の日時を取得
                        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        
                        # ESP32から受信したデータを分割
                        data_parts = line.split()
                        if len(data_parts) >= 8:  # データが完全な形式であることを確認
                            try:
                                # 圧力値を10で割る
                                timestamp = data_parts[0]
                                pressure = float(data_parts[1]) / 10.0  # 圧力を10で割る
                                accel_x = data_parts[2]
                                accel_y = data_parts[3]
                                accel_z = data_parts[4]
                                gyro_x = data_parts[5]
                                gyro_y = data_parts[6]
                                gyro_z = data_parts[7]
                                
                                # 調整したデータを作成
                                modified_line = f"{now} {timestamp} {pressure:.1f} {accel_x} {accel_y} {accel_z} {gyro_x} {gyro_y} {gyro_z}"
                                print(f"Data: {modified_line}")
                                csvfile.write(modified_line + '\n')
                                csvfile.flush()  # バッファをフラッシュして確実に書き込む
                            except (ValueError, IndexError) as e:
                                print(f"Error processing data: {e} - Line: {line}")
                        else:
                            print(f"Invalid data format: {line}")
                    
            except KeyboardInterrupt:
                print("\nData logging stopped by user")
                
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
    finally:
        # 終了時にシリアルポートを閉じる
        if 'ser' in locals():
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    main()