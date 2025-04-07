import os
import numpy as np
import pandas as pd
import tensorflow as tf
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler

# TensorFlow Lite変換用
import tensorflow as tf

# 定数設定
WINDOW_SIZE = 10  # 窓サイズ（10サンプル≒約5秒）
OVERLAP = 5       # オーバーラップ
NUM_CLASSES = 3   # 1号車、2号車、3号車

def load_and_preprocess_data(file_path, car_number):
    """
    CSVファイルを読み込み、前処理を行う
    """
    # CSVを読み込む
    df = pd.read_csv(file_path)
    
    # 特徴量とラベルの準備
    features = []
    labels = []
    
    # 窓ごとに特徴抽出
    for i in range(0, len(df) - WINDOW_SIZE + 1, OVERLAP):
        window = df.iloc[i:i+WINDOW_SIZE]
        
        # 基本統計量の特徴抽出
        feature_vector = []
        
        # センサーごとの特徴量抽出
        for sensor in ['pressure', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z']:
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
        labels.append(car_number - 1)  # 0-based indexingに変換（1号車→0, 2号車→1, 3号車→2）
    
    return features, labels

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

def train_model():
    """
    データの読み込み、前処理、モデルの学習を行う
    """
    # データセットの準備
    features = []
    labels = []
    
    # 1号車のデータ読み込み
    # car1_features, car1_labels = load_and_preprocess_data('1号車データ_ダミー.csv', 1)
    car1_features, car1_labels = load_and_preprocess_data('1号車データ2.csv', 1)
    features.extend(car1_features)
    labels.extend(car1_labels)
    
    # 2号車のデータ読み込み
    # car2_features, car2_labels = load_and_preprocess_data('2号車データ_ダミー.csv', 2)
    car2_features, car2_labels = load_and_preprocess_data('2号車データ2.csv', 2)
    features.extend(car2_features)
    labels.extend(car2_labels)
    
    # 3号車のデータ読み込み
    # car3_features, car3_labels = load_and_preprocess_data('3号車データ_ダミー.csv', 3)
    car3_features, car3_labels = load_and_preprocess_data('3号車データ2.csv', 3)
    features.extend(car3_features)
    labels.extend(car3_labels)
    
    # NumPy配列に変換
    X = np.array(features)
    y = np.array(labels)
    
    # 特徴量の正規化
    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(X)
    
    # 学習データとテストデータに分割
    X_train, X_test, y_train, y_test = train_test_split(X_scaled, y, test_size=0.2, random_state=42)
    
    # モデルの構築
    model = build_model(X_train.shape[1])
    
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
    
    # モデルの保存
    model.save('train_identification_model.h5')
    
    # TensorFlow Liteモデルへの変換
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    tflite_model = converter.convert()
    
    # TFLiteモデルの保存
    with open('train_identification_model.tflite', 'wb') as f:
        f.write(tflite_model)
    
    # スケーラーの保存
    import joblib
    joblib.dump(scaler, 'scaler.joblib')
    
    return model, scaler

def predict_car_number(model, scaler, data_file):
    """
    新しいデータの号車を予測する
    """
    # データの読み込みと前処理
    df = pd.read_csv(data_file)
    
    # 窓ごとに特徴抽出
    features = []
    for i in range(0, len(df) - WINDOW_SIZE + 1, OVERLAP):
        window = df.iloc[i:i+WINDOW_SIZE]
        
        # 特徴抽出（train_model関数内のものと同じロジック）
        feature_vector = []
        for sensor in ['pressure', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z']:
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
    
    # 特徴量の正規化
    X = np.array(features)
    X_scaled = scaler.transform(X)
    
    # 予測
    predictions = model.predict(X_scaled)
    
    # 最も確率の高いクラスをカウント
    predicted_classes = np.argmax(predictions, axis=1)
    car_counts = np.bincount(predicted_classes)
    predicted_car = np.argmax(car_counts) + 1  # 0-based → 1-based
    
    confidence = car_counts[predicted_car - 1] / len(predicted_classes)
    
    return predicted_car, confidence

# TensorFlow Liteモデルを使った推論用クラス
class TrainCarPredictor:
    def __init__(self, model_path='train_identification_model.tflite', scaler_path='scaler.joblib'):
        """
        TFLiteモデルと特徴量スケーラーを読み込む
        """
        import joblib
        
        # TFLiteモデルの読み込み
        self.interpreter = tf.lite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        
        # 入出力テンソルの取得
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        
        # スケーラーの読み込み
        self.scaler = joblib.load(scaler_path)
    
    def extract_features(self, data_file):
        """
        データファイルから特徴量を抽出する
        """
        # CSVを読み込む
        df = pd.read_csv(data_file)
        
        # 特徴量の準備
        features = []
        
        # 窓ごとに特徴抽出
        for i in range(0, len(df) - WINDOW_SIZE + 1, OVERLAP):
            window = df.iloc[i:i+WINDOW_SIZE]
            
            # 特徴抽出（上記と同じロジック）
            feature_vector = []
            for sensor in ['pressure', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z']:
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
        X = np.array(features, dtype=np.float32)
        
        # 特徴量の正規化
        X_scaled = self.scaler.transform(X)
        
        return X_scaled
    
    def predict(self, data_file):
        """
        データファイルから号車を予測する
        """
        # 特徴量抽出
        features = self.extract_features(data_file)
        
        predictions = []
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
        
        # NumPy配列に変換
        predictions = np.array(predictions)
        
        # 最も確率の高いクラスをカウント
        predicted_classes = np.argmax(predictions, axis=1)
        car_counts = np.bincount(predicted_classes)
        predicted_car = np.argmax(car_counts) + 1  # 0-based → 1-based
        
        # 信頼度の計算（多数決の結果の割合）
        confidence = car_counts[predicted_car - 1] / len(predicted_classes)
        
        return predicted_car, confidence, predictions


# Android/iOS アプリ用のTFLiteモデル変換（量子化版）
def convert_to_optimized_tflite(keras_model_path='train_identification_model.h5'):
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
    with open('train_identification_model_quantized.tflite', 'wb') as f:
        f.write(tflite_quant_model)
    
    print(f"量子化モデルサイズ: {len(tflite_quant_model) / 1024:.2f} KB")
    return tflite_quant_model


if __name__ == "__main__":
    # モデルのトレーニング
    model, scaler = train_model()
    
    # 最適化されたTFLiteモデルへの変換
    convert_to_optimized_tflite()
    
    # モデル予測テスト
    predictor = TrainCarPredictor()
    # predicted_car, confidence, _ = predictor.predict("新しいデータ_ダミー.csv")
    predicted_car, confidence, _ = predictor.predict("新しいデータ2.csv")
    print(f"予測された車両: {predicted_car}号車 (信頼度: {confidence:.2f})")