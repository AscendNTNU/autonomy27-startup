import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition, VehicleCommand
from std_srvs.srv import Trigger

import numpy as np

class LandingNode(Node):
    def __init__(self):
        super().__init__("landing_node")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.trajectory_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_callback, qos_profile)

        self.land_service = self.create_service(Trigger, '/land', self.land_service_callback)

    def land_service_callback(self, request, response):
        self.get_logger().info("Mottatt /land trigger. Starter landing.")
        self.start_landing()

        response.success = True
        response.message = ""

        return response

    def start_landing(self):
        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Heartbeat laget")

    def timer_callback(self):
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = self.get_timestamp()

        # NED
        setpoint.position = [0.0, 0.0, 0.0]

        self.trajectory_publisher.publish(setpoint)
        self.get_logger().info(
            f"Publiserer til TrajectorySetpoint: {setpoint.position}")

    def position_callback(self, msg):
        position = np.array([msg.x, msg.y, msg.z])
        distance = np.linalg.norm(position)

        self.get_logger().info(
            f"Mottatt VehicleLocalPosition: {position}. Avstand: {distance:.3f}")

        threshold = 0.25  # 25 cm
        if distance < threshold:
            self.landing_finished()
        
    def landing_finished(self):
        self.timer.destroy()
        self.get_logger().info("Stoppet timer")
        self.disarm()

    def disarm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 0.0
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
