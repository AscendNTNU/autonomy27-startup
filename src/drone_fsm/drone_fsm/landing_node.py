import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, OffboardControlMode, VehicleLandDetected
from std_srvs.srv import Trigger


NaN = float('nan')

class LandingNode(Node):
    def __init__(self):
        super().__init__("landing_node")
        self.is_landing = False

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)

        self.trajectory_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.land_detected_subscriber = self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected', self.land_detected_callback, qos_profile)

        self.land_service = self.create_service(Trigger, '/land', self.land_service_callback)

        self.get_logger().info("Init ferdig")

    def land_service_callback(self, request, response):
        self.get_logger().info("Mottatt /land trigger. Starter landing.")
        self.start_landing()

        response.success = True
        response.message = ""

        return response

    def start_landing(self):
        self.is_landing = True
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Heartbeat laget")

    def timer_callback(self):
        self.publish_offboard()
        self.publish_setpoint()

    def publish_setpoint(self):
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = self.get_timestamp()

        # NED
        setpoint.position = [NaN, NaN, NaN]
        setpoint.velocity = [0.0, 0.0, 0.5]
        setpoint.yaw = NaN

        self.trajectory_publisher.publish(setpoint)

    def publish_offboard(self):
        msg = OffboardControlMode()
        msg.timestamp = self.get_timestamp()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        
        self.offboard_publisher.publish(msg)

    def land_detected_callback(self, msg):
        if self.is_landing and msg.landed:
            self.landing_finished()
        
    def landing_finished(self):
        self.is_landing = False
        if self.timer is not None:
            self.timer.destroy()
            self.timer = None
            self.get_logger().info("Stoppet timer")
            self.disarm()

    def disarm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 0.0
        msg.param2 = 0.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_timestamp()

        self.vehicle_command_publisher.publish(msg)
        self.get_logger().info("Publiserte disarm-melding")

    def get_timestamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)


def main(args=None):
    rclpy.init(args=args)

    landing_node = LandingNode()

    try:
        rclpy.spin(landing_node)
    except KeyboardInterrupt:
        pass
    finally:
        landing_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
