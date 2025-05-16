import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Float32
import curses

class KeyPublisher(Node):
    def __init__(self):
        super().__init__('debug_key_publisher')

        # パブリッシャー一覧
        self.pub_speed = self.create_publisher(Float32, '/pix_hooke/v2a_drivestafb/vcu_chassis_speed_fb', 10)
        self.pub_throttle = self.create_publisher(Float32, '/pix_hooke/v2a_drivestafb/vcu_chassis_throttle_padl_fb', 10)
        self.pub_headlight = self.create_publisher(Int8, '/pix_hooke/v2a_vehiclestafb/vcu_vehicle_head_lamp_fb', 10)
        self.pub_left = self.create_publisher(Int8, '/pix_hooke/v2a_vehiclestafb/vcu_vehicle_left_lamp_fb', 10)
        self.pub_right = self.create_publisher(Int8, '/pix_hooke/v2a_vehiclestafb/vcu_vehicle_right_lamp_fb', 10)
        self.pub_gear = self.create_publisher(Int8, '/pix_hooke/v2a_drivestafb/vcu_chassis_gear_fb', 10)
        self.pub_parking = self.create_publisher(Int8, '/pix_hooke/v2a_brakestafb/vcu_chassis_epb_fb', 10)
        self.pub_emergency = self.create_publisher(Int8, '/pix_hooke/v2a_vehicleworkstafb/vcu_chassis_e_stop_sta_fb', 10)
        self.pub_control = self.create_publisher(Int8, '/pix_hooke/v2a_vehicleworkstafb/vcu_driving_mode_fb', 10)
        self.pub_steering_lf = self.create_publisher(Float32, '/pix_hooke/v2a_chassiswheelanglefb/vcu_chassis_wheel_angle_lf', 10)
        self.pub_steering_rf = self.create_publisher(Float32, '/pix_hooke/v2a_chassiswheelanglefb/vcu_chassis_wheel_angle_rf', 10)

        # 初期値
        self.speed = 0.0
        self.throttle = 0.0
        self.headlight = 0
        self.left = 0
        self.right = 0
        self.gear = 1
        self.parking = 0
        self.emergency = 0
        self.control = 0
        self.steering = 0.0

    def update_and_publish(self, key):
        if key == curses.KEY_UP:
            self.speed += 1.0
            self.throttle += 5.0
        elif key == curses.KEY_DOWN:
            self.speed = max(0.0, self.speed - 1.0)
            self.throttle = max(0.0, self.throttle - 5.0)
        elif key == curses.KEY_LEFT:
            self.steering = max(-30.0, self.steering - 5.0)
        elif key == curses.KEY_RIGHT:
            self.steering = min(30.0, self.steering + 5.0)
        elif key == ord('h'):
            self.headlight ^= 1
        elif key == ord('l'):
            self.left ^= 1
        elif key == ord('r'):
            self.right ^= 1
        elif key == ord('g'):
            self.gear = 1 if self.gear == 3 else self.gear + 1
        elif key == ord('p'):
            self.parking ^= 1
        elif key == ord('e'):
            self.emergency ^= 1
        elif key == ord('c'):
            self.control = (self.control + 1) % 4
        elif key == ord('q'):
            return False

        # 値制限
        self.throttle = min(100.0, max(0.0, self.throttle))
        self.speed = min(50.0, max(0.0, self.speed))

        # パブリッシュ
        self.pub_speed.publish(Float32(data=self.speed))
        self.pub_throttle.publish(Float32(data=self.throttle))
        self.pub_headlight.publish(Int8(data=self.headlight))
        self.pub_left.publish(Int8(data=self.left))
        self.pub_right.publish(Int8(data=self.right))
        self.pub_gear.publish(Int8(data=self.gear))
        self.pub_parking.publish(Int8(data=self.parking))
        self.pub_emergency.publish(Int8(data=self.emergency))
        self.pub_control.publish(Int8(data=self.control))
        self.pub_steering_lf.publish(Float32(data=self.steering))
        self.pub_steering_rf.publish(Float32(data=self.steering))

        print(f"[速度:{self.speed:.1f}m/s] [スロットル:{self.throttle:.1f}%] [舵角:{self.steering:.1f}°] [ギア:{self.gear}] [P:{self.parking}] [E:{self.emergency}] [C:{self.control}] [H:{self.headlight}] [L:{self.left}] [R:{self.right}]")
        return True

def main(stdscr):
    rclpy.init()
    node = KeyPublisher()

    stdscr.nodelay(True)
    stdscr.clear()
    stdscr.addstr(0, 0, "上下矢印:アクセル増減, a/d:ステアリング, h:ヘッドライト, l:左ウインカー, r:右ウインカー, g:ギア, p:パーキングブレーキ, e:緊急停止, c:操作方式, q:終了")
    stdscr.refresh()

    try:
        while rclpy.ok():
            key = stdscr.getch()
            if key != -1:
                keep_running = node.update_and_publish(key)
                if not keep_running:
                    break
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    curses.wrapper(main)
