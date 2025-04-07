import asyncio
import json
import datetime
import csv
import os
import numpy as np
import pandas as pd
import tensorflow as tf
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import joblib
import time
from bleak import BleakScanner, BleakClient
import matplotlib.pyplot as plt
from collections import Counter

# Pico Wデバイスが使用しているBLE UUIDs
DEVICE_NAME = "PicoTest"
SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
CHAR_UUID_TX = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

# 機械学習モデルの設定
WINDOW_SIZE = 10  # 窓サイズ（10サンプル≒約5秒）
OVERLAP = 5       # オーバーラップ
NUM_CLASSES = 3   # 1号車、2号車、3号車
MODEL_PATH = 'train_identification_model.tflite'
H5_MODEL_PATH = 'train_identification_model.h5'
SCALER_PATH = 'scaler.joblib'
PREDICTION_INTERVAL = 45  # 予測間隔（秒）
TRAINING_FILES = {
    1: '1号車データ.csv',  # 1号車データのCSVファイル
    2: '2号車データ.csv',  # 2号車データのCSVファイル
    3: '3号車データ.csv'   # 3号車データのCSVファイル
}

# データをCSVに保存するための設定
CSV_FILENAME = str(datetime.datetime.now().strftime('%Y%m%d%H%M%S'))+"sensor_data.csv"
TEMP_CSV_FILENAME = "temp_sensor_data.csv"
SAVE_TO_CSV = True
csv_file = None
csv_writer = None
temp_csv_file = None
temp_csv_writer = None

# 最新のセンサーデータを保持するグローバル変数
latest_data = {
    'timestamp': '',
    'P': 0,    # 圧力
    'AX': 0, 'AY': 0, 'AZ': 0,  # 加速度
    'GX': 0, 'GY': 0, 'GZ': 0   # ジャイロ
}

# 予測に関する変数
prediction_timer = 0
prediction_enabled = False
last_prediction_time = 0
prediction_history = []  # 予測履歴を保存するリスト
CLEAR_SCREEN = False     # 画面クリアを無効化

# モデル学習関連の関数
def load_and_preprocess_data(file_path, car_number):
    """
    CSVファイルを読み込み、前処理を行う
    """
    print(f"{file_path} を読み込んでいます...")
    
    try:
        # CSVを読み込む
        df = pd.read_csv(file_path)
        
        # タイムスタンプ形式のチェックと標準化
        if 'timestamp' in df.columns:
            print(f"タイムスタンプ形式: サンプル = {df['timestamp'].iloc[0]}")
        
        # 必要なカラムがあるか確認
        required_columns = ['timestamp', 'pressure', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z']
        missing_columns = [col for col in required_columns if col not in df.columns]
        
        if missing_columns:
            print(f"警告: {file_path} に以下のカラムがありません: {missing_columns}")
            print(f"実際のカラム: {df.columns.tolist()}")
            
            # カラム名のマッピング試行
            if 'pressure' not in df.columns and 'P' in df.columns:
                df['pressure'] = df['P']
            
            if 'accel_x' not in df.columns and 'AX' in df.columns:
                df['accel_x'] = df['AX']
            
            if 'accel_y' not in df.columns and 'AY' in df.columns:
                df['accel_y'] = df['AY']
                
            if 'accel_z' not in df.columns and 'AZ' in df.columns:
                df['accel_z'] = df['AZ']
            
            if 'gyro_x' not in df.columns and 'GX' in df.columns:
                df['gyro_x'] = df['GX']
                
            if 'gyro_y' not in df.columns and 'GY' in df.columns:
                df['gyro_y'] = df['GY']
                
            if 'gyro_z' not in df.columns and 'GZ' in df.columns:
                df['gyro_z'] = df['GZ']
            
            # 再度確認
            missing_columns = [col for col in required_columns if col not in df.columns]
            if missing_columns:
                print(f"エラー: マッピング後も不足しているカラム: {missing_columns}")
                return [], []
    
        # データ基本情報
        print(f"データ行数: {len(df)}")
        print(f"カラム: {df.columns.tolist()}")
        
        # 各カラムの基本統計量
        numeric_columns = ['pressure', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z']
        print("データの基本統計:")
        for col in numeric_columns:
            if col in df.columns:
                print(f"  {col}: 平均={df[col].mean():.2f}, 標準偏差={df[col].std():.2f}, 最小={df[col].min():.2f}, 最大={df[col].max():.2f}")
        
        # 特徴量とラベルの準備
        features = []
        labels = []
        
        # 窓ごとに特徴抽出
        for i in range(0, len(df) - WINDOW_SIZE + 1, OVERLAP):
            window = df.iloc[i:i+WINDOW_SIZE]
            
            # 基本統計量の特徴抽出
            feature_vector = []
            
            # センサーごとの特徴量抽出
            for sensor in numeric_columns:
                if sensor not in df.columns:
                    continue
                
                # 窓内の平均
                feature_vector.append(window[sensor].mean())
                # 標準偏差
                feature_vector.append(window[sensor].std())
                # 範囲（最大値 - 最小値）
                feature_vector.append(window[sensor].max() - window[sensor].min())
                # 変化の大きさ（隣接点間の差分の絶対値の平均）
                feature_vector.append(abs(window[sensor].diff().dropna()).mean())
                # ピーク数（単純な検出）
                peaks = 0
                for j in range(1, len(window) - 1):
                    if (window[sensor].iloc[j] > window[sensor].iloc[j-1] and 
                        window[sensor].iloc[j] > window[sensor].iloc[j+1] and
                        abs(window[sensor].iloc[j] - window[sensor].iloc[j-1]) > 0.1 * window[sensor].std()):
                        peaks += 1
                feature_vector.append(peaks)
            
            features.append(feature_vector)
            labels.append(car_number - 1)  # 0-based indexingに変換
        
        print(f"{car_number}号車データから {len(features)} 個の特徴ベクトルを抽出しました。")
        print(f"特徴量の次元: {len(feature_vector)}")
        
        return features, labels
        
    except Exception as e:
        print(f"エラー: {file_path} の読み込み中に例外が発生しました: {e}")
        return [], []

def build_model(input_shape):
    """
    モデルの構築
    """
    model = tf.keras.Sequential([
        tf.keras.layers.Dense(128, activation='relu', input_shape=(input_shape,)),
        tf.keras.layers.Dropout(0.3),
        tf.keras.layers.Dense(64, activation='relu'),
        tf.keras.layers.Dropout(0.2),
        tf.keras.layers.Dense(32, activation='relu'),
        tf.keras.layers.Dense(NUM_CLASSES, activation='softmax')
    ])
    
    # モデルのコンパイル
    model.compile(
        optimizer='adam',
        loss='sparse_categorical_crossentropy',
        metrics=['accuracy']
    )
    
    return model

def train_model(force_retrain=False):
    """
    データの読み込み、前処理、モデルの学習を行う
    """
    # すでに学習済みモデルが存在し、再学習が指定されていない場合はスキップ
    if os.path.exists(H5_MODEL_PATH) and os.path.exists(SCALER_PATH) and not force_retrain:
        print(f"既存のモデル ({H5_MODEL_PATH}) とスケーラー ({SCALER_PATH}) が見つかりました。")
        print("モデルを再学習するには force_retrain=True を指定してください。")
        
        # モデルとスケーラーをロード
        model = tf.keras.models.load_model(H5_MODEL_PATH)
        scaler = joblib.load(SCALER_PATH)
        
        return model, scaler
    
    print("モデルの学習を開始します...")
    
    # データセットの準備
    features = []
    labels = []
    
    # 全ての号車データを読み込む
    for car_number, file_path in TRAINING_FILES.items():
        if not os.path.exists(file_path):
            print(f"警告: {file_path} が見つかりません。")
            continue
            
        car_features, car_labels = load_and_preprocess_data(file_path, car_number)
        features.extend(car_features)
        labels.extend(car_labels)
    
    if not features or not labels:
        print("エラー: 特徴量またはラベルが空です。学習データを確認してください。")
        return None, None
    
    # データセットバランスの確認
    label_counts = Counter(labels)
    print(f"クラスバランス: {dict(label_counts)}")
    
    # NumPy配列に変換
    X = np.array(features)
    y = np.array(labels)
    
    # 特徴量の正規化
    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(X)
    
    # 正規化後のデータ確認
    print("正規化後のデータ形状:", X_scaled.shape)
    print("正規化後の平均:", X_scaled.mean(axis=0)[:5], "...")  # 最初の5次元だけ表示
    print("正規化後の標準偏差:", X_scaled.std(axis=0)[:5], "...")
    
    # 学習データとテストデータに分割
    X_train, X_test, y_train, y_test = train_test_split(X_scaled, y, test_size=0.2, random_state=42, stratify=y)
    
    # モデルの構築
    model = build_model(X_train.shape[1])
    
    # モデル構造の表示
    model.summary()
    
    # モデルの学習
    history = model.fit(
        X_train, y_train,
        epochs=50,
        batch_size=32,
        validation_data=(X_test, y_test),
        verbose=1
    )
    
    # モデルの評価
    test_loss, test_acc = model.evaluate(X_test, y_test)
    print(f'テスト精度: {test_acc:.4f}')
    
    # 混同行列の計算
    y_pred = np.argmax(model.predict(X_test), axis=1)
    conf_matrix = tf.math.confusion_matrix(y_test, y_pred)
    print("混同行列:")
    print(conf_matrix.numpy())
    
    # 学習曲線のプロット
    plt.figure(figsize=(12, 4))
    
    plt.subplot(1, 2, 1)
    plt.plot(history.history['accuracy'], label='訓練精度')
    plt.plot(history.history['val_accuracy'], label='検証精度')
    plt.title('モデル精度')
    plt.xlabel('エポック')
    plt.ylabel('精度')
    plt.legend()
    
    plt.subplot(1, 2, 2)
    plt.plot(history.history['loss'], label='訓練損失')
    plt.plot(history.history['val_loss'], label='検証損失')
    plt.title('モデル損失')
    plt.xlabel('エポック')
    plt.ylabel('損失')
    plt.legend()
    
    plt.tight_layout()
    plt.savefig('training_history.png')
    plt.close()
    
    # モデルの保存
    model.save(H5_MODEL_PATH)
    print(f"モデルを {H5_MODEL_PATH} に保存しました。")
    
    # スケーラーの保存
    joblib.dump(scaler, SCALER_PATH)
    print(f"スケーラーを {SCALER_PATH} に保存しました。")
    
    # TensorFlow Liteモデルへの変換
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    tflite_model = converter.convert()
    
    # TFLiteモデルの保存
    with open(MODEL_PATH, 'wb') as f:
        f.write(tflite_model)
    print(f"TFLiteモデルを {MODEL_PATH} に保存しました。")
    
    return model, scaler

def convert_to_optimized_tflite(keras_model_path=H5_MODEL_PATH):
    """
    Kerasモデルを最適化されたTFLiteモデルに変換する
    """
    # Kerasモデルの読み込み
    model = tf.keras.models.load_model(keras_model_path)
    
    # TFLite変換（動的量子化を適用）
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    
    # 動的範囲量子化を適用（モデルサイズの縮小とパフォーマンス向上）
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    
    # 変換の実行
    tflite_quant_model = converter.convert()
    
    # 量子化モデルの保存
    quant_model_path = 'train_identification_model_quantized.tflite'
    with open(quant_model_path, 'wb') as f:
        f.write(tflite_quant_model)
    
    print(f"量子化モデルを {quant_model_path} に保存しました。")
    print(f"量子化モデルサイズ: {len(tflite_quant_model) / 1024:.2f} KB")
    return tflite_quant_model

# TensorFlow Liteモデルを使った推論用クラス
class TrainCarPredictor:
    def __init__(self, model_path=MODEL_PATH, scaler_path=SCALER_PATH):
        """
        TFLiteモデルと特徴量スケーラーを読み込む
        """
        print(f"予測器の初期化: モデル={model_path}, スケーラー={scaler_path}")
        
        # TFLiteモデルの読み込み
        self.interpreter = tf.lite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        
        # 入出力テンソルの取得
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        
        # モデル情報の表示
        print(f"モデル入力形状: {self.input_details[0]['shape']}")
        print(f"モデル出力形状: {self.output_details[0]['shape']}")
        
        # スケーラーの読み込み
        self.scaler = joblib.load(scaler_path)
    
    def extract_features(self, data_file):
        """
        データファイルから特徴量を抽出する
        """
        # CSVを読み込む
        df = pd.read_csv(data_file)
        print(f"データファイル {data_file} を読み込みました。行数: {len(df)}")
        
        # カラム名確認
        print(f"カラム: {df.columns.tolist()}")
        
        # カラム名が異なる場合の対応
        column_mapping = {
            'P': 'pressure',
            'AX': 'accel_x',
            'AY': 'accel_y',
            'AZ': 'accel_z',
            'GX': 'gyro_x',
            'GY': 'gyro_y',
            'GZ': 'gyro_z'
        }
        
        for old_col, new_col in column_mapping.items():
            if old_col in df.columns and new_col not in df.columns:
                df[new_col] = df[old_col]
        
        # 必要なカラムがあるか確認
        required_columns = ['pressure', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z']
        missing_columns = [col for col in required_columns if col not in df.columns]
        
        if missing_columns:
            print(f"警告: 以下のカラムがありません: {missing_columns}")
            return np.array([])
        
        # 特徴量の準備
        features = []
        
        # 窓ごとに特徴抽出
        for i in range(0, len(df) - WINDOW_SIZE + 1, OVERLAP):
            window = df.iloc[i:i+WINDOW_SIZE]
            
            # 特徴抽出
            feature_vector = []
            for sensor in required_columns:
                feature_vector.append(window[sensor].mean())
                feature_vector.append(window[sensor].std())
                feature_vector.append(window[sensor].max() - window[sensor].min())
                feature_vector.append(abs(window[sensor].diff().dropna()).mean())
                
                peaks = 0
                for j in range(1, len(window) - 1):
                    if (window[sensor].iloc[j] > window[sensor].iloc[j-1] and 
                        window[sensor].iloc[j] > window[sensor].iloc[j+1] and
                        abs(window[sensor].iloc[j] - window[sensor].iloc[j-1]) > 0.1 * window[sensor].std()):
                        peaks += 1
                feature_vector.append(peaks)
                
            features.append(feature_vector)
        
        # NumPy配列に変換
        if not features:
            print("警告: 特徴ベクトルが抽出できませんでした。")
            return np.array([])
            
        X = np.array(features, dtype=np.float32)
        print(f"抽出された特徴量: {len(features)}個, 形状: {X.shape}")
        
        # 特徴量の正規化
        X_scaled = self.scaler.transform(X)
        
        # 正規化された特徴量の統計情報
        print(f"正規化後の特徴量: 平均={X_scaled.mean():.4f}, 標準偏差={X_scaled.std():.4f}")
        
        return X_scaled
    
    def predict(self, data_file):
        """
        データファイルから号車を予測する
        """
        # データファイルが存在し、十分なデータが含まれているか確認
        try:
            df = pd.read_csv(data_file)
            if len(df) < WINDOW_SIZE:
                print(f"警告: 予測に必要なデータが不足しています ({len(df)}/{WINDOW_SIZE})")
                return None, 0, []
        except Exception as e:
            print(f"エラー: データファイルの読み込みに失敗しました: {e}")
            return None, 0, []
        
        # 特徴量抽出
        features = self.extract_features(data_file)
        
        if len(features) == 0:
            print("警告: 特徴量の抽出に失敗しました")
            return None, 0, []
        
        predictions = []
        raw_outputs = []
        
        # 各窓に対して予測
        for feature in features:
            # 入力テンソルに値をセット
            feature_reshaped = np.expand_dims(feature, axis=0).astype(np.float32)
            self.interpreter.set_tensor(self.input_details[0]['index'], feature_reshaped)
            
            # 推論の実行
            self.interpreter.invoke()
            
            # 出力テンソルから結果を取得
            output = self.interpreter.get_tensor(self.output_details[0]['index'])
            predictions.append(output[0])
            raw_outputs.append(output[0].copy())
        
        # NumPy配列に変換
        predictions = np.array(predictions)
        
        # 予測結果がない場合
        if len(predictions) == 0:
            return None, 0, []
        
        # 各クラスの確率の平均を計算
        avg_probs = np.mean(predictions, axis=0)
        print(f"クラスごとの平均確率: {avg_probs}")
        
        # 最も確率の高いクラスをカウント
        predicted_classes = np.argmax(predictions, axis=1)
        car_counts = np.bincount(predicted_classes, minlength=NUM_CLASSES)
        predicted_car = np.argmax(car_counts) + 1  # 0-based → 1-based
        
        # 各窓の予測クラスの内訳を表示
        print(f"窓ごとの予測: 1号車={car_counts[0]}回, 2号車={car_counts[1]}回, 3号車={car_counts[2]}回")
        
        # 信頼度の計算（多数決の結果の割合）
        confidence = car_counts[predicted_car - 1] / len(predicted_classes)
        
        # 最終予測レポート
        print(f"予測結果: {predicted_car}号車 (信頼度: {confidence:.2%})")
        
        return predicted_car, confidence, raw_outputs

# リアルタイムでデータを表示する関数
def print_data(prediction_info=None):
    """センサーデータを整形して表示する"""
    # 画面クリアが有効な場合のみクリア
    if CLEAR_SCREEN:
        os.system('clear' if os.name == 'posix' else 'cls')
        
    # 表示区切り線
    print("\n" + "="*50)
    
    current_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    print(f"=== センサーデータ {current_time} ===")
    
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
    
    # 予測情報の表示
    if prediction_info:
        predicted_car, confidence, timestamp, probabilities = prediction_info
        print("\n=== 列車号車予測 ===")
        print(f"予測時刻: {timestamp}")
        print(f"予測された車両: {predicted_car}号車")
        print(f"信頼度: {confidence:.2%}")
        
        # 確率分布の表示
        if probabilities is not None and len(probabilities) == 3:
            print(f"クラス確率: 1号車={probabilities[0]:.2%}, 2号車={probabilities[1]:.2%}, 3号車={probabilities[2]:.2%}")
        
        # 予測履歴の表示
        if prediction_history:
            print("\n予測履歴:")
            for i, (car, conf, _) in enumerate(prediction_history[-5:]):  # 最新5件のみ表示
                print(f"  {i+1}. {car}号車 (信頼度: {conf:.2%})")
        
        # 次の予測までの残り時間
        time_since_last = time.time() - last_prediction_time
        time_to_next = max(0, PREDICTION_INTERVAL - time_since_last)
        print(f"次の予測まで: {int(time_to_next)}秒")
    elif prediction_enabled:
        # 予測が有効だが、まだ結果がない場合
        time_since_start = time.time() - prediction_timer
        time_to_first = max(0, PREDICTION_INTERVAL - time_since_start)
        print("\n=== 列車号車予測 ===")
        print(f"最初の予測まで: {int(time_to_first)}秒")
    
    print("(Ctrl+Cで終了)")
    print("="*50)

# CSVファイルを初期化する関数
def init_csv():
    """CSVファイルを初期化し、ヘッダーを書き込む"""
    global csv_file, csv_writer, temp_csv_file, temp_csv_writer
    
    # メインCSVファイル
    file_exists = os.path.isfile(CSV_FILENAME)
    csv_file = open(CSV_FILENAME, 'a', newline='')
    csv_writer = csv.writer(csv_file)
    
    # 一時CSVファイル
    temp_csv_file = open(TEMP_CSV_FILENAME, 'w', newline='')
    temp_csv_writer = csv.writer(temp_csv_file)
    
    # ヘッダーを書き込む
    header = [
        'timestamp', 'pressure', 
        'accel_x', 'accel_y', 'accel_z', 
        'gyro_x', 'gyro_y', 'gyro_z'
    ]
    
    # メインCSVにはファイルが存在しない場合のみヘッダーを書き込む
    if not file_exists:
        csv_writer.writerow(header)
    
    # 一時CSVには常にヘッダーを書き込む
    temp_csv_writer.writerow(header)

# CSVにデータを書き込む関数
def write_to_csv():
    """データをCSVファイルに書き込む"""
    global prediction_timer, prediction_enabled
    
    if csv_writer is None or temp_csv_writer is None:
        return
    
    timestamp = datetime.datetime.now().strftime('%M:%S.%f')[:-4]  # 分:秒.ミリ秒 形式
    latest_data['timestamp'] = timestamp
    
    row_data = [
        timestamp,
        latest_data.get('P', ''),
        latest_data.get('AX', ''),
        latest_data.get('AY', ''),
        latest_data.get('AZ', ''),
        latest_data.get('GX', ''),
        latest_data.get('GY', ''),
        latest_data.get('GZ', '')
    ]
    
    # メインCSVに書き込む
    if SAVE_TO_CSV:
        csv_writer.writerow(row_data)
        csv_file.flush()  # すぐにディスクに書き込む
    
    # 一時CSVに書き込む
    temp_csv_writer.writerow(row_data)
    temp_csv_file.flush()  # すぐにディスクに書き込む
    
    # 予測タイマーの開始
    if not prediction_enabled:
        prediction_timer = time.time()
        prediction_enabled = True

# 予測実行関数
async def perform_prediction():
    """現在の蓄積データを使用して予測を実行"""
    global last_prediction_time, prediction_history
    
    # 予測が無効か、一時CSVファイルがない場合は何もしない
    if not prediction_enabled or temp_csv_file is None:
        return None
    
    last_prediction_time = time.time()
    
    try:
        # 一時ファイルを閉じて再度開くことで、最新の状態を確実に読み込む
        temp_csv_file.flush()
        
        # 予測器の初期化
        predictor = TrainCarPredictor()
        
        # 予測の実行
        predicted_car, confidence, raw_outputs = predictor.predict(TEMP_CSV_FILENAME)
        
        if predicted_car is None:
            print("予測できませんでした。十分なデータが揃うまでお待ちください。")
            return None
        
        # 確率分布を平均して計算
        avg_probs = np.mean(raw_outputs, axis=0) if raw_outputs else None
        
        # 予測結果を整形
        prediction_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        result = (predicted_car, confidence, prediction_time, avg_probs)
        
        # 予測履歴に追加
        prediction_history.append((predicted_car, confidence, prediction_time))
        
        return result
        
    except Exception as e:
        print(f"予測エラー: {e}")
        import traceback
        traceback.print_exc()
        return None

# 通知コールバック関数
def notification_handler(sender, data):
    """BLEデータ受信時のコールバック"""
    global latest_data
    
    try:
        # データをデコード
        data_str = data.decode('utf-8')
        # print(f"受信データ: {data_str}")  # デバッグ用
        
        # データの種類に応じて処理
        if data_str.startswith('P:'):
            # 圧力データ
            value = float(data_str.split(':')[1])
            latest_data['P'] = value
            
        elif data_str.startswith('A:'):
            # 加速度データ (形式: "A:x,y,z")
            values = data_str[2:].split(',')
            if len(values) >= 3:
                latest_data['AX'] = float(values[0])
                latest_data['AY'] = float(values[1])
                latest_data['AZ'] = float(values[2])
                
        elif data_str.startswith('G:'):
            # ジャイロデータ (形式: "G:x,y,z")
            values = data_str[2:].split(',')
            if len(values) >= 3:
                latest_data['GX'] = float(values[0])
                latest_data['GY'] = float(values[1])
                latest_data['GZ'] = float(values[2])
                
                # すべてのデータが更新されたのでCSVに保存し、表示を更新
                write_to_csv()
                print_data()
                
    except Exception as e:
        print(f"データ処理エラー: {e}")
        print(f"受信データ: {data}")

# デバイスをスキャンする関数
async def scan_for_device():
    """指定されたデバイス名のBLEデバイスをスキャンする"""
    print(f"{DEVICE_NAME}を検索中...")
    
    # スキャン時間を10秒に延長
    device = None
    devices = await BleakScanner.discover(timeout=10.0)
    
    for d in devices:
        if d.name == DEVICE_NAME:
            device = d
            break
            
    if device:
        print(f"デバイスが見つかりました: {device.name} ({device.address})")
        return device
    else:
        print(f"{DEVICE_NAME}が見つかりませんでした。")
        print("以下のデバイスが検出されました:")
        for d in devices:
            if d.name:
                print(f" - {d.name} ({d.address})")
        return None

# 予測ループ
async def prediction_loop():
    """45秒おきに予測を実行するループ"""
    current_prediction = None
    
    while True:
        await asyncio.sleep(1)  # 1秒ごとにチェック
        
        # 予測が有効で、最後の予測から45秒経過したか、まだ予測していない場合
        if prediction_enabled and (time.time() - last_prediction_time >= PREDICTION_INTERVAL or last_prediction_time == 0):
            # 予測を実行
            prediction_result = await perform_prediction()
            if prediction_result:
                current_prediction = prediction_result
                print_data(current_prediction)
        else:
            # 通常の表示更新
            print_data(current_prediction)

# メイン関数
async def main():
    """メインのBLE接続とデータ受信ループ"""
    try:
        # CSVファイルを初期化
        init_csv()
        
        # 予測ループを開始
        prediction_task = asyncio.create_task(prediction_loop())
        
        while True:
            # デバイスをスキャン
            device = await scan_for_device()
            if not device:
                print("5秒後に再スキャンします...")
                await asyncio.sleep(5)
                continue
                
            try:
                # デバイスに接続
                async with BleakClient(device) as client:
                    print(f"{device.name}に接続しました")
                    
                    # 通知を登録
                    await client.start_notify(CHAR_UUID_TX, notification_handler)
                    
                    print("データ受信中... (Ctrl+Cで終了)")
                    
                    # 接続が維持されている限り続ける
                    while client.is_connected:
                        await asyncio.sleep(1)
                        
                    print("デバイスとの接続が切断されました")
                    
            except Exception as e:
                print(f"接続エラー: {e}")
                print("5秒後に再接続します...")
                await asyncio.sleep(5)
                
    except KeyboardInterrupt:
        print("\nプログラムを終了します")
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    finally:
        # CSVファイルを閉じる
        if csv_file:
            csv_file.close()
            print(f"データは {CSV_FILENAME} に保存されました")
        if temp_csv_file:
            temp_csv_file.close()
            # 一時ファイルの削除（オプション）
            try:
                os.remove(TEMP_CSV_FILENAME)
            except:
                pass

# シミュレータモード
async def simulator_mode():
    """センサーデータをシミュレートするモード"""
    global latest_data
    
    print("=== シミュレータモード ===")
    print("BLEデバイスの代わりにダミーデータを生成します")
    
    # CSVファイルを初期化
    init_csv()
    
    # 予測ループを開始
    prediction_task = asyncio.create_task(prediction_loop())
    
    # シミュレーション用CSVファイルを指定
    simulate_file = input("シミュレーション用CSVファイル（空白でランダム生成）: ")
    
    if simulate_file and os.path.exists(simulate_file):
        # 既存のCSVファイルを使用
        print(f"{simulate_file} からデータをシミュレートします")
        df = pd.read_csv(simulate_file)
        rows = df.to_dict('records')
        
        for row in rows:
            # CSVの列名に合わせて調整
            if 'pressure' in row:
                latest_data['P'] = row['pressure']
            elif 'P' in row:
                latest_data['P'] = row['P']
                
            if 'accel_x' in row:
                latest_data['AX'] = row['accel_x']
            elif 'AX' in row:
                latest_data['AX'] = row['AX']
                
            if 'accel_y' in row:
                latest_data['AY'] = row['accel_y']
            elif 'AY' in row:
                latest_data['AY'] = row['AY']
                
            if 'accel_z' in row:
                latest_data['AZ'] = row['accel_z']
            elif 'AZ' in row:
                latest_data['AZ'] = row['AZ']
                
            if 'gyro_x' in row:
                latest_data['GX'] = row['gyro_x']
            elif 'GX' in row:
                latest_data['GX'] = row['GX']
                
            if 'gyro_y' in row:
                latest_data['GY'] = row['gyro_y']
            elif 'GY' in row:
                latest_data['GY'] = row['GY']
                
            if 'gyro_z' in row:
                latest_data['GZ'] = row['gyro_z']
            elif 'GZ' in row:
                latest_data['GZ'] = row['GZ']
            
            # データを記録
            write_to_csv()
            print_data()
            
            # 0.5秒待機
            await asyncio.sleep(0.5)
    
    else:
        # ランダムデータの生成
        print("ランダムデータを生成します")
        
        car_type = int(input("シミュレーションする車両タイプ (1-3): ") or "1")
        
        # 車両タイプごとの特性
        car_params = {
            1: {  # 1号車特性
                'P': {'base': 3245, 'range': 30},
                'AX': {'base': 0.9, 'range': 0.3},
                'AY': {'base': 0, 'range': 0.2},
                'AZ': {'base': -0.6, 'range': 0.2},
                'GX': {'base': -2, 'range': 20},
                'GY': {'base': 1.5, 'range': 20},
                'GZ': {'base': 0.5, 'range': 15}
            },
            2: {  # 2号車特性
                'P': {'base': 3145, 'range': 40},
                'AX': {'base': 1.1, 'range': 0.5},
                'AY': {'base': 0.1, 'range': 0.3},
                'AZ': {'base': -0.4, 'range': 0.4},
                'GX': {'base': -5, 'range': 25},
                'GY': {'base': 2.5, 'range': 30},
                'GZ': {'base': 1.2, 'range': 25}
            },
            3: {  # 3号車特性
                'P': {'base': 3345, 'range': 25},
                'AX': {'base': 0.7, 'range': 0.2},
                'AY': {'base': -0.1, 'range': 0.2},
                'AZ': {'base': -0.8, 'range': 0.2},
                'GX': {'base': -1, 'range': 15},
                'GY': {'base': 0.5, 'range': 15},
                'GZ': {'base': -0.3, 'range': 10}
            }
        }
        
        # 指定された車両タイプのパラメータを使用
        params = car_params.get(car_type, car_params[1])
        
        try:
            # データ生成ループ
            while True:
                for sensor, settings in params.items():
                    # ベース値にランダムな変動を加える
                    base = settings['base']
                    range_val = settings['range']
                    value = base + (np.random.random() - 0.5) * range_val
                    
                    # 値の更新
                    latest_data[sensor] = value
                
                # データを記録
                write_to_csv()
                print_data()
                
                # 0.5秒待機
                await asyncio.sleep(0.5)
                
        except KeyboardInterrupt:
            print("\nシミュレーションを終了します")

if __name__ == "__main__":
    # セットアップ方法を表示
    print("=== リアルタイム列車号車識別システム ===")
    print("Version 2.0 - 学習・診断機能付き")
    
    # ログ保持のお知らせ
    print("※ ログ保持モードが有効: 画面は消去されずログが蓄積されます")
    
    # 実行モードの選択
    print("\n実行モードを選択:")
    print("1. モデル学習モード")
    print("2. リアルタイム予測モード")
    print("3. シミュレータモード")
    
    mode = input("モードを選択 (1-3): ")
    
    if mode == '1':
        # モデル学習モード
        print("\n=== モデル学習モード ===")
        retrain = input("既存のモデルを再学習しますか？ (y/n): ").lower() == 'y'
        
        # 学習データファイルのパス確認
        for car, file_path in TRAINING_FILES.items():
            new_path = input(f"{car}号車データファイルのパス [{file_path}]: ")
            if new_path:
                TRAINING_FILES[car] = new_path
                
        # モデルの学習実行
        model, scaler = train_model(force_retrain=retrain)
        
        if model is not None and scaler is not None:
            print("\nモデルの学習が完了しました。")
            print(f"モデルファイル: {H5_MODEL_PATH}")
            print(f"TFLiteモデル: {MODEL_PATH}")
            print(f"スケーラーファイル: {SCALER_PATH}")
            
            # 最適化版の作成
            convert_to_optimized_tflite()
            
    elif mode == '3':
        # シミュレータモード
        asyncio.run(simulator_mode())
        
    else:
        # デフォルト: リアルタイム予測モード
        print("\n=== リアルタイム予測モード ===")
        print(f"デバイス名: {DEVICE_NAME}")
        print(f"データ保存: {'有効' if SAVE_TO_CSV else '無効'} ({CSV_FILENAME})")
        print(f"予測間隔: {PREDICTION_INTERVAL}秒")
        print("モデルとスケーラーを確認中...")
        
        # モデルとスケーラーの存在チェック
        model_exists = os.path.isfile(MODEL_PATH)
        scaler_exists = os.path.isfile(SCALER_PATH)
        
        if not model_exists or not scaler_exists:
            print("エラー: 必要なファイルが見つかりません")
            if not model_exists:
                print(f" - モデルファイルが見つかりません: {MODEL_PATH}")
            if not scaler_exists:
                print(f" - スケーラーファイルが見つかりません: {SCALER_PATH}")
                
            # モデル学習のオファー
            if input("モデルを学習しますか？ (y/n): ").lower() == 'y':
                model, scaler = train_model(force_retrain=True)
                if model is None or scaler is None:
                    print("モデルの学習に失敗しました。プログラムを終了します。")
                    exit(1)
            else:
                print("プログラムを終了します")
                exit(1)
        
        print("準備完了！BLEデバイスに接続します...")
        print("----------------------------------------------")
        
        # メインループを実行
        asyncio.run(main())