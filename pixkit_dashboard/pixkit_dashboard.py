#!/usr/bin/env python3
"""
Copyright 2025 NAKAMURA Shotaro

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Float32
from flask import Flask, render_template_string, jsonify, render_template, Response
from pix_hooke_driver_msgs.msg import V2aDriveStaFb, V2aBrakeStaFb, V2aVehicleStaFb, V2aVehicleWorkStaFb, V2aSteerStaFb, V2aPowerStaFb
from flask_socketio import SocketIO
import threading
import json
import time
import os
import cv2
import numpy as np

class PIXkitDashboard(Node):
    def __init__(self):
        super().__init__('pixkit_dashboard')
        
        # 車両状態データを保存
        self.vehicle_data = {
            'gear': 'N',               # ギア状態 (1:D, 2:N, 3:R)
            'speed': 0.0,              # 速度 (m/s)
            'throttle': 0.0,           # アクセル開度 (0-100%)

            'parking': 0,              # パーキングブレーキ (0:オフ, 1:オン, 2:解除中, 3:施錠中)

            'control': 0,              # コントロール (0:nobody, 1:自動運転, 2:リモコン, 3:ハンドル)
            'limit': 30,               # 制限速度
            'emargency': 0,            # 緊急停止 (0:オフ, 1,2,3:オン)

            'battery': 60,              # バッテリー残量
            
            'headlight': 0,            # ヘッドライト
            'left_turn_signal': 0,     # 左ウインカー
            'right_turn_signal': 0,    # 右ウインカー
            'hazard_soignal': 0,       # ハザードランプ

            'steer_mode': 'ackerman',  # ステアリングモード (ackerman, parallel, counter)
            
            'tire_angle_fl': 0.0,      # タイヤ角 (左前)
            'tire_angle_fr': 0.0,      # タイヤ角 (右前)
            'tire_angle_rl': 0.0,      # タイヤ角 (左後)
            'tire_angle_rr': 0.0,      # タイヤ角 (右後)
        }
        
        # サブスクライバーを作成
        try:
            self.drive_sub = self.create_subscription(  
                V2aDriveStaFb,  
                '/pix_hooke/v2a_drivestafb',  
                self.drive_callback,  
                10)
              
            self.brake_sub = self.create_subscription(  
                V2aBrakeStaFb,  
                '/pix_hooke/v2a_brakestafb',  
                self.brake_callback,  
                10)
              
            self.vehicle_work_sub = self.create_subscription(  
                V2aVehicleWorkStaFb,  
                '/pix_hooke/v2a_vehicleworkstafb',  
                self.vehicle_work_callback,  
                10)
            
            self.power_sub = self.create_subscription(  
                V2aPowerStaFb,
                '/pix_hooke/v2a_powerstafb',
                self.power_callback,
                10)
              
            self.vehicle_sta_sub = self.create_subscription(  
                V2aVehicleStaFb,  
                '/pix_hooke/v2a_vehiclestafb',  
                self.vehicle_sta_callback,  
                10)

            self.steer_angle_sub = self.create_subscription(  
                V2aSteerStaFb,
                '/pix_hooke/v2a_steerstafb',
                self.steer_angle_callback,  
                10)

        except Exception as e:
            self.get_logger().error(f'Error creating subscribers: {e}')
        
        self.get_logger().info('PIXkit Dashboard node has been initialized')
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    # コールバック関数
    def drive_callback(self, msg):
        gear_map = {1: 'D', 2: 'N', 3: 'R'}
        self.vehicle_data['gear'] = gear_map.get(msg.vcu_chassis_gear_fb, 'N')
        self.vehicle_data['speed'] = float(msg.vcu_chassis_speed_fb) * 3.6  # m/s から km/h に変換
        self.vehicle_data['throttle'] = float(msg.vcu_chassis_throttle_padl_fb)
      
    def brake_callback(self, msg):
        parking_map = {0: False, 1: True, 2: True, 3: False}
        self.vehicle_data['parking'] = parking_map.get(msg.vcu_chassis_epb_fb, False)
      
    def vehicle_work_callback(self, msg):
        control_map = {0: 'nobody', 1: 'auto', 2: 'rc', 3: 'handle'}
        self.vehicle_data['control'] = control_map.get(msg.vcu_driving_mode_fb, 'nobody')
        self.vehicle_data['limit'] = float(msg.vcu_chassis_speed_limited_val_fb) * 3.6  # m/s から km/h に変換
        emargency_map = {0: False, 1: True, 2: True, 3: True}  
        self.vehicle_data['emargency'] = emargency_map.get(msg.vcu_chassis_e_stop_sta_fb, False)

    def power_callback(self, msg):
        self.vehicle_data['battery'] = msg.vcu_chassis_power_soc_fb
      
    def vehicle_sta_callback(self, msg):
        self.vehicle_data['headlight'] = bool(msg.vcu_vehicle_head_lamp_fb)
        self.vehicle_data['left_turn_signal'] = bool(msg.vcu_vehicle_left_lamp_fb)
        self.vehicle_data['right_turn_signal'] = bool(msg.vcu_vehicle_right_lamp_fb)
        self.vehicle_data['hazard_signal'] = bool(msg.vcu_vehicle_hazard_war_lamp_fb)
      
    def steer_angle_callback(self, msg):
        steer_map = {0: 'ackerman', 1: 'parallel', 2: 'counter'}
        self.vehicle_data['steer_mode'] = steer_map.get(msg.vcu_chassis_steer_mode_fb, 'ackerman')
        self.vehicle_data['tire_angle_fl'] = float(msg.vcu_chassis_steer_angle_fb) * 27.5 / 500
        self.vehicle_data['tire_angle_fr'] = float(msg.vcu_chassis_steer_angle_fb) * 27.5 / 500
        self.vehicle_data['tire_angle_rl'] = float(msg.vcu_chassis_steer_angle_rear_fb) * 27.5 / 500
        self.vehicle_data['tire_angle_rr'] = float(msg.vcu_chassis_steer_angle_rear_fb) * 27.5 / 500
      
    def timer_callback(self):  
        self.get_logger().info(f'Vehicle Data: {self.vehicle_data}')  
      
    def get_vehicle_data(self):  
        data_copy = self.vehicle_data.copy()  
        data_copy['speed'] = float(data_copy['speed'])  
        data_copy['throttle'] = float(data_copy['throttle'])  
        return data_copy

# Flaskウェブアプリ
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading', ping_timeout=10, ping_interval=3)

@app.route('/')
def index():
    try:
        return render_template('index.html')
    except Exception as e:
        print(f"Error rendering template: {e}")
        return f"Error rendering template: {e}", 500
    
# カメラモジュールの初期化時に追加の設定
def initialize_camera():
    global camera
    try:
        camera = cv2.VideoCapture(0)
        
        if not camera.isOpened():
            print("Failed to open camera 0. Trying alternative...")
            camera = cv2.VideoCapture(1)  # 別のデバイス番号を試す
            if not camera.isOpened():
                print("Failed to open camera 1. Trying alternative...")
                camera = cv2.VideoCapture(2)  # 別のデバイス番号を試す
                if not camera.isOpened():
                    print("Failed to open camera 2. Trying alternative...")
                    camera = cv2.VideoCapture(3)  # 別のデバイス番号を試す
        
        if camera.isOpened():
            # カメラ設定
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            camera.set(cv2.CAP_PROP_FPS, 30)  # フレームレート設定
            camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # バッファを小さく
            print("Camera initialized successfully")
        else:
            print("Warning: Could not open any camera")
    except Exception as e:
        print(f"Camera initialization error: {e}")

# MJPEG生成関数の再実装
def gen_frames():
    global camera
    
    while True:
        if camera is None or not camera.isOpened():
            try:
                print("Reinitializing camera...")
                initialize_camera()
                time.sleep(0.5)
                continue
            except Exception as e:
                print(f"Camera reinitialization error: {e}")
                # ダミーフレームを返す
                dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(dummy_frame, "Camera Unavailable", (120, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                ret, buffer = cv2.imencode('.jpg', dummy_frame)
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                      b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                time.sleep(0.5)
                continue
        
        # 各リクエストに対して新しいフレームを取得
        try:
            # バッファをクリアするために複数回読み出す
            for _ in range(3):
                camera.grab()
            success, frame = camera.read()
            
            if not success:
                print("Failed to read camera frame")
                time.sleep(0.1)
                continue
                
            # フレームが取得できたら画像処理
            frame = cv2.resize(frame, (640, 480))
            
            # JPEG圧縮して返す
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            frame = buffer.tobytes()
            
            # MJPEG形式で送信
            yield (b'--frame\r\n'
                  b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            
            # フレームレート調整
            time.sleep(0.03)  # ~30fps
            
        except Exception as e:
            print(f"Error in frame generation: {e}")
            time.sleep(0.5)

# リアカメラエンドポイント
@app.route('/rear_camera_feed')
def rear_camera_feed():
    # キャッシュ防止ヘッダー
    response = Response(gen_frames(), 
                       mimetype='multipart/x-mixed-replace; boundary=frame')
    response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
    response.headers['Pragma'] = 'no-cache'
    response.headers['Expires'] = '0'
    return response

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    # 接続時に最新データを送信
    emit_vehicle_data()

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

@socketio.on('request_update')
def handle_update_request():
    emit_vehicle_data()

# グローバル変数としてダッシュボードノードを保持
dashboard_node = None

def emit_vehicle_data():
    global dashboard_node
    if dashboard_node:
        try:
            data = dashboard_node.get_vehicle_data()
            # データ型を確実に変換
            for key, value in data.items():
                if isinstance(value, float):
                    data[key] = float(value)  # 明示的にfloatに変換
                elif isinstance(value, int):
                    data[key] = int(value)    # 明示的にintに変換
                elif isinstance(value, bool):
                    data[key] = bool(value)   # 明示的にboolに変換
            
            socketio.emit('vehicle_data', data)
        except Exception as e:
            print(f"Error emitting data: {e}")

def emit_thread():
    """定期的にデータを送信するスレッド"""
    global dashboard_node
    error_count = 0
    while True:
        try:
            emit_vehicle_data()
            error_count = 0  # 成功したらエラーカウントリセット
            time.sleep(0.03)  # 30Hzで更新
        except Exception as e:
            error_count += 1
            print(f"Error in emit thread ({error_count}): {e}")
            if error_count > 10:
                print("Too many errors in emit thread, reducing frequency...")
                time.sleep(1.0)  # エラー多発時は頻度を下げる
            else:
                time.sleep(0.1)

def ros_spin_thread():
    global dashboard_node
    try:
        print("Starting ROS spin loop...")
        rclpy.spin(dashboard_node)
        print("ROS spin loop ended.")
    except Exception as e:
        print(f"Error in ROS spin thread: {e}")

# アプリケーション終了時の処理
def cleanup():
    global camera
    print("Cleaning up resources...")
    if camera is not None:
        camera.release()
    print("Camera released")

def main():
    global dashboard_node
    
    # ROSノードの初期化
    rclpy.init()
    dashboard_node = PIXkitDashboard()

    # カメラ初期化
    initialize_camera()
    
    threading.Thread(target=ros_spin_thread, daemon=True).start()
    threading.Thread(target=emit_thread, daemon=True).start()
    
    print("Starting Flask server...")
    
    # Flaskサーバー起動
    try:
        socketio.run(app, host='0.0.0.0', port=5001, allow_unsafe_werkzeug=True)
    except Exception as e:
        print(f"Error starting Flask server: {e}")
    finally:
        # 終了処理
        cleanup()
        if dashboard_node:
            dashboard_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()