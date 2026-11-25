import math
import sys
import time
from pathlib import Path

SRC_DIR = Path(__file__).resolve().parents[3] / "src"
SDK_ROOT = SRC_DIR / "FTServo_Python"

if str(SDK_ROOT) not in sys.path:
    sys.path.insert(0, str(SDK_ROOT))

from scservo_sdk import GroupSyncRead, PortHandler, COMM_SUCCESS
from scservo_sdk.sms_sts import (
    sms_sts as SmsStsPacketHandler, SMS_STS_PRESENT_POSITION_L,
    SMS_STS_PRESENT_SPEED_L,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from sensor_msgs.msg import JointState
from serial import SerialException


class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        self.port = self.declare_parameter('port','/dev/ttyUSB0').get_parameter_value().string_value
        self.baudrate = self.declare_parameter('baudrate',1000000).get_parameter_value().integer_value
        self.ids = self.declare_parameter('servo_ids',[1]).get_parameter_value().integer_array_value
        self.min_ticks = self.declare_parameter('min_ticks',0).get_parameter_value().integer_value
        self.max_ticks = self.declare_parameter('max_ticks',4095).get_parameter_value().integer_value
        self.zero_ticks = self.declare_parameter('zero_ticks',2048).get_parameter_value().integer_value
        self.speed = self.declare_parameter('speed',100).get_parameter_value().integer_value
        self.acc = self.declare_parameter('acc',30).get_parameter_value().integer_value
        self.poll_hz = self.declare_parameter('poll_hz',10.0).get_parameter_value().double_value
        if not math.isfinite(self.poll_hz) or self.poll_hz <= 0:
            raise ValueError("poll_hz must be finite and positive")
        feedback_timeout = self.declare_parameter('feedback_timeout', 0.5)
        self.timeout: float = feedback_timeout.get_parameter_value().double_value
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("feedback_timeout must be finite and positive")
        self.last_feedback: dict[int, float] = {}
        self.disable_on_exit = self.declare_parameter('disable_torque_on_exit',False).get_parameter_value().integer_value

        self.portHandler = PortHandler(self.port)
        self.packetHandler = SmsStsPacketHandler(self.portHandler)
        try:
            if not self.portHandler.openPort():
                raise RuntimeError("Failed to open the port")
            if not self.portHandler.setBaudRate(self.baudrate):
                raise RuntimeError("Failed to set the baudrate")

            self.publisher = self.create_publisher(JointState,'/joint_state',10)
            self.subscriber = self.create_subscription(JointState,'/joint_commands',self.command_callback,10)
            self.timer = self.create_timer((1.0/self.poll_hz),self.read_state)


        except Exception:
            if self.portHandler.is_open:
                self.portHandler.closePort()
            super().destroy_node()
            raise

    def command_callback(self, msg: JointState):
        if len(msg.name) != len(msg.position):
            self.get_logger().warning("コマンドの関節名と目標角度の要素数が一致しません")
            return

        servo_ids_by_name = {f"servo_{servo_id}": servo_id for servo_id in self.ids}
        ticks_per_rad = 4096.0 / math.tau
        for name, target_rad in zip(msg.name, msg.position):
            servo_id = servo_ids_by_name.get(name)
            if servo_id is None:
                continue
            if not math.isfinite(target_rad):
                self.get_logger().warning(f"{name} の目標角度が有限値ではありません")
                continue
            target_tick = self.zero_ticks + target_rad * ticks_per_rad
            if not math.isfinite(target_tick):
                self.get_logger().warning(f"{name} の目標角度を tick に変換できません")
                continue
            target_tick = round(target_tick)
            if not self.min_ticks <= target_tick <= self.max_ticks:
                self.get_logger().warning(f"{name} の目標位置が設定範囲外です ({target_tick} tick)")
                continue

            last_feedback = self.last_feedback.get(servo_id)
            if last_feedback is None or (time.monotonic() - last_feedback) > self.timeout:
                self.get_logger().warning(f"{name} の有効な位置フィードバックがありません")
                continue

            result, error = self.packetHandler.WritePosEx(
                servo_id, target_tick, self.speed, self.acc
            )
            if result != COMM_SUCCESS or error:
                self.get_logger().error(
                    f"{name} の目標位置送信に失敗しました (通信結果={result}, サーボエラー={error})"
                )


    def read_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        rad_per_tick = math.tau / 4096.0
        names: list[str] = []
        positions: list[float] = []

        for servo_id in self.ids:
            position , result , error = self.packetHandler.ReadPos(servo_id)
            if result != COMM_SUCCESS or error:
                self.get_logger().warning(f"servo_{servo_id} の位置取得に失敗しました "f"(通信結果={result}, サーボエラー={error})")
                continue
            self.last_feedback[servo_id] = time.monotonic()
            names.append(f"servo_{servo_id}")
            positions.append((position - self.zero_ticks)*rad_per_tick)
        if len(names) != len(positions):
            raise ValueError("要素数が合いません")
        if names:
            msg.name = names
            msg.position = positions
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ServoController()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except SerialException as error:
        get_logger('servo_controller').error(f"シリアル通信に失敗しました: {error}")
        return 1
    finally:
        if node is not None:
            try:
                if node.portHandler.is_open:
                    node.portHandler.closePort()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
