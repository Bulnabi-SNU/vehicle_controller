__author__ = "Juyong Shin"
__contact__ = "juyong3393@snu.ac.kr"

# import rclpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# import px4_msgs
from px4_msgs.msg import VehicleStatus, VehicleCommand

class VehicleController(Node):
    
    def __init__(self):
        super().__init__('vehicle_controller')

        # 0. Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 1. Constants

        # 2. Variables

        ## vehicle states
        self.states = {
            "IDLE" : 0, "ARMING" : 1, "TAKEOFF" : 2, "LOITER" : 3, "OFFBOARD" : 4,
            "POSITION" : 5, "RETURN" : 6, "KILLED" : -1
        }
        self.kill = False
        self.takeoff = False
        self.state = self.states["IDLE"]

        ## time step counter for takeoff after arming
        self.counter = 0

        # 3. Create Subscribers
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )

        # 4. Create Publishers
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )
        self.timer = self.create_timer(0.5, self.timer_callback)

    def vehicle_status_callback(self, msg):
        """Callback function for vehicle_status topic subscriber."""
        if msg.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            """disarmed"""
            if self.kill:
                self.state = self.states["KILLED"]
            else:
                self.state = self.states["IDLE"]
        else:
            """armed"""
            if not self.takeoff:    # Before take-off
                self.state = self.states["ARMING"]
            elif msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
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

    def timer_callback(self):
        """Callback function for the timer."""
        if self.state == self.states["IDLE"]:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        elif self.state == self.states["ARMING"]:
            self.counter += 1
            if self.counter >= 10:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF)
                self.takeoff = True
        elif self.state == self.states["TAKEOFF"]:
            pass
    
    def publish_vehicle_command(self, command, **kwargs):
        """Publish a vehicle command.""" 
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = kwargs.get("param1", float('nan'))
        msg.param2 = kwargs.get("param2", float('nan'))
        msg.param3 = kwargs.get("param3", float('nan'))
        msg.param4 = kwargs.get("param4", float('nan'))
        msg.param5 = kwargs.get("param5", float('nan'))
        msg.param6 = kwargs.get("param6", float('nan'))
        msg.param7 = kwargs.get("param7", float('nan'))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

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