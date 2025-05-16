import subprocess
import time
import signal
import sys
import os

# Flaskサーバー起動
server_process = subprocess.Popen(["python3", "pixkit_dashboard.py"])

# Flask起動まで待機（必要に応じて調整）
time.sleep(5)

# Chrome起動（フルスクリーン＆ディスプレイ2）
chrome_process = subprocess.Popen([
    "google-chrome",
    "http://localhost:5001",
    "--new-window",
    "--kiosk",
    "--noerrdialogs",
    "--disable-session-crashed-bubble",
    "--window-position=-1920,0",  # ディスプレイ2（左側）
    "--window-size=1920,1080",
])

try:
    print("ダッシュボード起動中。Ctrl+Cで終了します。")
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

    # Chromeを強制終了（特定URLに絞る）
    try:
        subprocess.run(["pkill", "-f", "chrome.*localhost:5001"], check=True)
    except subprocess.CalledProcessError:
        pass  # プロセスがすでに終了している場合など

    print("すべて終了しました。")
    sys.exit(0)
