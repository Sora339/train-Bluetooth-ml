import asyncio
import datetime
import csv
import os
import time
import logging
from collections import deque, Counter
from bleak import BleakScanner

# ロギング設定
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# 定数
MANUFACTURER_ID = 0xCDCB
RAICOLLECTOR_ID_INDEX = 0
PACKET_ID_INDEX = 2
DATA_START_INDEX = 3
BYTES_PER_RECORD = 8
MAX_DATA_ENTRIES = 3

# 重複排除用TTL (秒)
DUPLICATE_TTL = 1.0  # 送信側再送間隔に応じて調整

# CSV 出力設定
CSV_FILENAME = f"{datetime.datetime.now().strftime('%Y%m%d%H%M%S')}_sensor_data.csv"
SAVE_TO_CSV = True
csv_file = None
csv_writer = None

# 状態保持用キュー
received_ids_queue       = deque(maxlen=100)
processed_data_queue     = deque(maxlen=1000)   # (timestamp, packet_id, entry_idx, p, ax, ay, az)
processed_packet_history = deque()              # (packet_id, timestamp)
last_raicollector_decision_time = time.time()

RAICOLLECTOR_ID = 0
data_count      = 0
last_hz_time    = time.time()
csv_data_count  = 0
start_time      = time.time()

def init_csv():
    global csv_file, csv_writer
    file_exists = os.path.isfile(CSV_FILENAME)
    csv_file = open(CSV_FILENAME, 'a', newline='', buffering=1)
    csv_writer = csv.writer(csv_file)
    if not file_exists:
        csv_writer.writerow(['id', 'timestamp', 'packet_id', 'pressure', 'accel_x', 'accel_y', 'accel_z'])

def packet_id_distance(id1, id2):
    """モジュロ256での最短差分を返す"""
    return min((id1 - id2) % 256, (id2 - id1) % 256)

def select_best_raicollector_id():
    global RAICOLLECTOR_ID, received_ids_queue, last_raicollector_decision_time
    if not received_ids_queue:
        return
    best_id, best_count = Counter(received_ids_queue).most_common(1)[0]
    if RAICOLLECTOR_ID != best_id:
        logger.info(f"RAICOLLECTOR_ID changed from {RAICOLLECTOR_ID} to {best_id}")
        RAICOLLECTOR_ID = best_id
    last_raicollector_decision_time = time.time()

def log_hz_if_needed():
    global last_hz_time, csv_data_count, start_time
    current_time = time.time()
    if current_time - last_hz_time >= 10.0:  # 10秒ごと
        total_elapsed = current_time - start_time
        cumulative_hz = csv_data_count / total_elapsed
        logger.info(f"Hz: {cumulative_hz:.2f}")
        last_hz_time = current_time

def detection_callback(device, advertisement_data):
    global RAICOLLECTOR_ID, processed_packet_history, packet_count

    mdata = advertisement_data.manufacturer_data
    if not mdata or MANUFACTURER_ID not in mdata:
        return

    data = mdata[MANUFACTURER_ID]
    if len(data) != 27:
        return

    try:
        now = time.time()
        # TTL超過分をクリア
        while processed_packet_history and now - processed_packet_history[0][1] > DUPLICATE_TTL:
            processed_packet_history.popleft()

        timestamp = datetime.datetime.now()
        raicollector_id = int.from_bytes(data[RAICOLLECTOR_ID_INDEX:RAICOLLECTOR_ID_INDEX+2], 'little')
        received_ids_queue.append(raicollector_id)

        packet_id = data[PACKET_ID_INDEX]
        # 重複判定
        for prev_id, prev_ts in processed_packet_history:
            if packet_id_distance(packet_id, prev_id) <= 2:
                return  # 重複と判断してスキップ

        # 初回 ID 設定
        if RAICOLLECTOR_ID == 0:
            RAICOLLECTOR_ID = raicollector_id
            logger.info(f"RAICOLLECTOR_ID initially set to {RAICOLLECTOR_ID}")

        if raicollector_id != RAICOLLECTOR_ID:
            return

        # 各サブデータのタイムスタンプ
        t3 = timestamp
        t2 = t3 - datetime.timedelta(milliseconds=100)
        t1 = t2 - datetime.timedelta(milliseconds=100)
        timestamps = [t1, t2, t3]

        for i in range(MAX_DATA_ENTRIES):
            off = DATA_START_INDEX + i * BYTES_PER_RECORD
            if off + BYTES_PER_RECORD > len(data):
                break

            p  = int.from_bytes(data[off:off+2], 'little', signed=True)   / 10
            ax = int.from_bytes(data[off+2:off+4], 'little', signed=True) / 10000
            ay = int.from_bytes(data[off+4:off+6], 'little', signed=True) / 10000
            az = int.from_bytes(data[off+6:off+8], 'little', signed=True) / 10000

            if not (abs(p) < 0.1 and abs(ax) < 0.0001 and abs(ay) < 0.0001 and abs(az) < 0.0001):
                processed_data_queue.append((timestamps[i], packet_id, i+1, p, ax, ay, az))

        # 履歴に追加
        processed_packet_history.append((packet_id, now))

    except Exception as e:
        pass  # エラーログも削除

async def monitor_raicollector_selection():
    while True:
        await asyncio.sleep(1)
        if time.time() - last_raicollector_decision_time >= 5.0:
            select_best_raicollector_id()

def save_processed_data():
    global data_count, csv_data_count
    while processed_data_queue:
        ts, pid, entry_idx, p, ax, ay, az = processed_data_queue.popleft()
        row_id = f"{pid}_{entry_idx}"
        data_count += 1
        if SAVE_TO_CSV and csv_writer:
            csv_writer.writerow([
                row_id,
                ts.strftime('%Y-%m-%d %H:%M:%S.%f'),
                pid,
                p,
                ax,
                ay,
                az
            ])
            csv_data_count += 1

async def process_data():
    while True:
        save_processed_data()
        log_hz_if_needed()
        await asyncio.sleep(0.1)  # より頻繁にチェック

async def main():
    if SAVE_TO_CSV:
        init_csv()
    scanner = BleakScanner(detection_callback=detection_callback)
    await scanner.start()
    asyncio.create_task(monitor_raicollector_selection())
    asyncio.create_task(process_data())
    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        await scanner.stop()
        if csv_file:
            csv_file.close()

if __name__ == "__main__":
    asyncio.run(main())