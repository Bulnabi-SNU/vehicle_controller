__author__ = "Juyong Shin"
__contact__ = "juyong3393@snu.ac.kr"

# import rclpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# import px4_msgs
from px4_msgs.msg import VehicleStatus

class VehicleController(Node):
    
    def __init__(self):
        super().__init__('vehicle_controller')

        # 0. Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.TRANSIENT_LOCAL,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1
        )

        # 1. Variables

        # vehicle states
        self.states = {
            "IDLE" : 0, "TAKEOFF" : 1, "LOITER" : 2, "OFFBOARD" : 3,
            "POSITION" : 4, "RETURN" : 5, "KILLED" : -1
        }
        self.kill = False
        self.state = self.states["IDLE"]

        # 2. Create Subscribers
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )

    def vehicle_status_callback(self, msg):
        """Callback function for vehicle_status topic"""
        if msg.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            """disarmed"""
            if self.kill:
                self.state = self.states["KILLED"]
            else:
                self.state = self.states["IDLE"]
        else:
            """armed"""
            if msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                self.state = self.states["TAKEOFF"]
            elif msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                self.state = self.states["LOITER"]
            elif msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.state = self.states["OFFBOARD"]
            elif msg.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL:
                self.state = self.states["POSITION"]
            elif msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_RTL:
                self.state = self.states["RETURN"]

        self.get_logger().info(f"state : {self.state}")

def main(args = None):
    rclpy.init(args = args)

    vehicle_controller = VehicleController()
    rclpy.spin(vehicle_controller)

    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)