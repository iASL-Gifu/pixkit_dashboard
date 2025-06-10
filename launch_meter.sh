#!/bin/bash

# ターミナル1のコマンド
# CMD1='source ~/pix/pixkit/Autoware/install/setup.bash; ros2 launch autoware_launch autoware.launch.xml vehicle_model:=pixkit sensor_model:=pixkit_sensor_kit map_path:=/home/pixkit/GifuU-maps/tennis-court-maps'
CMD1='source ~/pix/pixkit/Autoware/install/setup.bash; ros2 launch pixkit_launch vehicle_interface.launch.xml'

# ターミナル2のコマンド
CMD2='source ~/pix/pixkit/Autoware/install/setup.bash; cd meter/src/pixkit_dashboard/pixkit_dashboard/; python3 run_dashboard.py'

# 新しいターミナルウィンドウで2つのタブを開く
gnome-terminal -- bash -c "$CMD1; exec bash"
  sleep 10
gnome-terminal -- bash -c "$CMD2; exec bash"

