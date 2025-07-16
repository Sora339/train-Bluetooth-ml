import asyncio
import datetime
import csv
import os
from bleak import BleakScanner
from collections import deque
import time
import logging

# ロギング設定
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

MANUFACTURER_ID = 0xCDCB
CSV_FILENAME = f"{datetime.datetime.now().strftime('%Y%m%d%H%M%S')}_sensor_data.csv"
SAVE_TO_CSV = True
csv_file = None
csv_writer = None

data_count = 0
last_status_time = time.time()

PACKET_ID_INDEX = 0
DATA_START_INDEX = 1
BYTES_PER_RECORD = 8
MAX_DATA_ENTRIES = 3

unique_data_buffer = {}
processed_data_queue = deque(maxlen=1000)

def init_csv():
    global csv_file, csv_writer
    file_exists = os.path.isfile(CSV_FILENAME)
    csv_file = open(CSV_FILENAME, 'a', newline='', buffering=1)
    csv_writer = csv.writer(csv_file)
    if not file_exists:
        csv_writer.writerow(['timestamp', 'packet_id', 'pressure', 'accel_x', 'accel_y', 'accel_z'])

def packet_id_distance(id1, id2):
    return min((id1 - id2) % 256, (id2 - id1) % 256)

def detection_callback(device, advertisement_data):
    global unique_data_buffer, processed_data_queue

    manufacturer_data = advertisement_data.manufacturer_data
    if not manufacturer_data:
        return

    if MANUFACTURER_ID not in manufacturer_data:
        return

    data = manufacturer_data[MANUFACTURER_ID]
    if len(data) != 27:
        logger.warning(f"Unexpected data length: {len(data)} bytes (expected 27)")
        return

    try:
        timestamp = datetime.datetime.now()
        packet_id = data[PACKET_ID_INDEX]
        time_diff_1 = 100
        time_diff_2 = 100

        data3_time = timestamp
        data2_time = timestamp - datetime.timedelta(milliseconds=time_diff_2)
        data1_time = data2_time - datetime.timedelta(milliseconds=time_diff_1)

        if data1_time > data2_time:
            data1_time = data2_time
        if data2_time > data3_time:
            data2_time = data3_time

        data_timestamps = [data1_time, data2_time, data3_time]
        entries = []

        for i in range(MAX_DATA_ENTRIES):
            offset = DATA_START_INDEX + (i * BYTES_PER_RECORD)
            if offset + BYTES_PER_RECORD > len(data) - 2:
                break

            try:
                pressure = int.from_bytes(data[offset:offset+2], 'little', signed=True) / 10
                accel_x = int.from_bytes(data[offset+2:offset+4], 'little', signed=True) / 10000
                accel_y = int.from_bytes(data[offset+4:offset+6], 'little', signed=True) / 10000
                accel_z = int.from_bytes(data[offset+6:offset+8], 'little', signed=True) / 10000

                if is_valid_data(pressure, accel_x, accel_y, accel_z):
                    entries.append((data_timestamps[i], packet_id, pressure, accel_x, accel_y, accel_z))
            except Exception:
                pass

        if entries:
            new_data_count = 0

            for entry in entries:
                entry_time, entry_id, pressure, accel_x, accel_y, accel_z = entry
                data_key = (round(pressure, 1), round(accel_x, 3), round(accel_y, 3), round(accel_z, 3))

                if data_key in unique_data_buffer:
                    existing_entry = unique_data_buffer[data_key]
                    existing_packet_id = existing_entry[1]
                    if packet_id_distance(entry_id, existing_packet_id) <= 2:
                        continue  # skip duplicate

                unique_data_buffer[data_key] = entry
                processed_data_queue.append(entry)
                new_data_count += 1

            if new_data_count > 0:
                save_processed_data()

    except Exception as e:
        logger.error(f"Error processing data: {e}")

def is_valid_data(pressure, accel_x, accel_y, accel_z):
    return not (
        abs(pressure) < 0.1 and
        abs(accel_x) < 0.0001 and
        abs(accel_y) < 0.0001 and
        abs(accel_z) < 0.0001
    )

async def process_data():
    global data_count, last_status_time
    start_time = time.time()

    while True:
        try:
            save_processed_data()
            current_time = time.time()
            if current_time - last_status_time >= 10.0:
                elapsed = current_time - start_time
                rate = data_count / elapsed if elapsed > 0 else 0
                logger.info(f"測定開始からの平均データレート: {rate:.2f}Hz (合計: {data_count}件)")
                if csv_file:
                    csv_file.flush()
                last_status_time = current_time
        except Exception as e:
            logger.error(f"データ処理エラー: {e}")
        await asyncio.sleep(1.0)

def save_processed_data():
    global data_count, processed_data_queue
    while processed_data_queue:
        entry = processed_data_queue.popleft()
        timestamp, packet_id, pressure, accel_x, accel_y, accel_z = entry
        data_count += 1

        if data_count % 100 == 0:
            time_str = timestamp.strftime('%H:%M:%S.%f')[:-3]
            logger.info(f"#{data_count} [{time_str}] ID:{packet_id} P:{pressure:.1f}hPa, A:({accel_x:.3f},{accel_y:.3f},{accel_z:.3f})")

        if SAVE_TO_CSV and csv_writer:
            timestamp_str = timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')
            csv_writer.writerow([timestamp_str, packet_id, pressure, accel_x, accel_y, accel_z])

async def main():
    try:
        if SAVE_TO_CSV:
            init_csv()

        logger.info(f"製造者ID 0x{MANUFACTURER_ID:04x} のデバイスをスキャン中... Ctrl+Cで終了")

        scanner = BleakScanner(detection_callback=detection_callback)
        processing_task = asyncio.create_task(process_data())
        await scanner.start()
        logger.info("スキャン開始しました")

        while True:
            await asyncio.sleep(1)

    except KeyboardInterrupt:
        logger.info("\n終了")
    finally:
        await scanner.stop()
        if csv_file:
            csv_file.close()
            logger.info(f"データ{data_count}件を{CSV_FILENAME}に保存")

if __name__ == "__main__":
    asyncio.run(main())
