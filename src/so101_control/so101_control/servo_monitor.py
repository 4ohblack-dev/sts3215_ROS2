import math
import sys
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
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ServoController(Node):
    def __init__(self, servo_ids=None):
        super().__init__('servo_controller')
        self.servo_ids = list(servo_ids if servo_ids is not None else [1])
        self.names = {servo_id: f"servo_{servo_id}" for servo_id in self.servo_ids}
        self.zero_position = {servo_id: 2048 for servo_id in self.servo_ids}
        self.ticks_per_rev = 4096.0
        self.speed = 100
        self.acc = 50

        self.publisher = self.create_publisher(JointState, '/joint_state',10)
        self.subscriber = self.create_subscription(
            JointState, "/joint_commands", self.command_callback, 10
        )
        self.portHandler = PortHandler('/dev/ttyUSB0')
        self.packetHandler = SmsStsPacketHandler(self.portHandler)
        try:
            if not self.portHandler.openPort():
                raise RuntimeError("Failed to open serial port")
            if not self.portHandler.setBaudRate(1000000):
                raise RuntimeError("Failed to change the baudrate")

            self.groupSyncRead = GroupSyncRead(
                self.packetHandler, SMS_STS_PRESENT_POSITION_L, 4
            )
            for servo_id in self.servo_ids:
                if not self.groupSyncRead.addParam(servo_id):
                    raise RuntimeError(f"Failed to register ID {servo_id}")
        except Exception:
            if self.portHandler.is_open:
                self.portHandler.closePort()
            raise

        self.feedback_ready = set()
        self.timer = self.create_timer(0.1,self.readState)        


    def ticks_to_rad(self,servo_id,ticks):
        return (ticks - self.zero_position[servo_id])*2.0*math.pi/self.ticks_per_rev
    def rad_to_ticks(self,servo_id,rad):
        return round(self.zero_position[servo_id] + rad*self.ticks_per_rev/(2.0*math.pi))

    
        
    def readState(self):
        comm_result = self.groupSyncRead.txRxPacket()
        if comm_result != COMM_SUCCESS:
            self.get_logger().error(self.packetHandler.getTxRxResult(comm_result))
            return
        
        msg = JointState()
        joint_names = []
        joint_positions = []
        joint_velocities = []
        msg.header.stamp = self.get_clock().now().to_msg()

        for servo_id in self.servo_ids:
            data_result, error = self.groupSyncRead.isAvailable(
                servo_id, SMS_STS_PRESENT_POSITION_L, 4
            )
            if data_result and error == 0:
                position = self.groupSyncRead.getData(
                    servo_id, SMS_STS_PRESENT_POSITION_L, 2
                )
                speed = self.groupSyncRead.getData(
                    servo_id, SMS_STS_PRESENT_SPEED_L, 2
                )

                position = self.packetHandler.scs_tohost(position,15)
                speed = self.packetHandler.scs_tohost(speed,15)

                joint_names.append(self.names[servo_id])
                joint_positions.append(self.ticks_to_rad(servo_id, position))
                # SDK speed units are servo-specific; verify before publishing as rad/s.
                joint_velocities.append(speed * 2.0 * math.pi / self.ticks_per_rev)
                self.feedback_ready.add(servo_id)
            else:
                self.get_logger().warning(f"[ID:{servo_id}] feedback unavailable (error={error})")
        if joint_names:
            msg.name = joint_names
            msg.position = joint_positions
            msg.velocity = joint_velocities
            self.publisher.publish(msg) 

    def command_callback(self,msg):
        if len(msg.name) != len(msg.position):
            self.get_logger().warning("Invalid JointState command")
            return

        for name,target_rad in zip(msg.name, msg.position):
            matching_ids = [sid for sid in self.servo_ids if self.names[sid] == name]
            if not matching_ids:
                continue

            servo_id = matching_ids[0]
            if servo_id not in self.feedback_ready:
                self.get_logger().warning(f"No valid feedback from ID {servo_id}")
                continue
    
            if not math.isfinite(target_rad):
                continue
    
            target_ticks = self.rad_to_ticks(servo_id, target_rad)
            if not (0 <= target_ticks <= 4095):
                self.get_logger().warning("Target outside servo tick range")
                continue
    
            result, error = self.packetHandler.WritePosEx(servo_id, target_ticks, self.speed, self.acc)
            if result != COMM_SUCCESS or error:
                self.get_logger().error(f"Write failed: ID={servo_id}, result={result}, error={error}")

    def destroy_node(self):
        self.portHandler.closePort()
        return super().destroy_node()                


        
def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ServoController([1])
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    from rclpy.executors import ExternalShutdownException
    main()
