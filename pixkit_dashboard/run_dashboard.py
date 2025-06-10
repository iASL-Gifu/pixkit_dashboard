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

import subprocess
import time
import signal
import sys
import os

# ブラウザ指定チェック
if len(sys.argv) != 2 or sys.argv[1] not in ["chrome", "firefox"]:
    print("引数 chrome または firefox を指定して下さい.")
    sys.exit(1)

browser = sys.argv[1]

# Flaskサーバー起動
server_process = subprocess.Popen(["python3", "pixkit_dashboard.py"])

# Flask起動まで待機（必要に応じて調整）
time.sleep(5)

# ブラウザ起動（kioskモード＆ディスプレイ2）
if browser == "chrome":
    browser_process = subprocess.Popen([
        "google-chrome",
        "http://localhost:5001",
        "--new-window",
        "--kiosk",
        "--noerrdialogs",
        "--disable-session-crashed-bubble",
        "--window-position=-1920,0",
        "--window-size=1920,1080",
    ])
elif browser == "firefox":
    browser_process = subprocess.Popen([
        "firefox",
        "--new-window",
        "http://localhost:5001",
        "--kiosk"
    ])

try:
    print(f"{browser} にてダッシュボード起動中。Ctrl+Cで終了します。")
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("\n終了処理中...")

finally:
    # Flaskサーバーを停止
    if server_process.poll() is None:
        server_process.terminate()
        try:
            server_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            server_process.kill()

    # ブラウザを強制終了
    if browser == "chrome":
        kill_cmd = ["pkill", "-f", "chrome.*localhost:5001"]
    elif browser == "firefox":
        kill_cmd = ["pkill", "-f", "firefox.*localhost:5001"]

    try:
        subprocess.run(kill_cmd, check=True)
    except subprocess.CalledProcessError:
        pass  # すでに終了している場合など

    print("すべて終了しました。")
    sys.exit(0)
