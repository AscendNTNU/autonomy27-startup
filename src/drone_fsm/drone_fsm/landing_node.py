import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition


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
        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_callback, qos_profile)

    def timer_callback(self):
        setpoint = TrajectorySetpoint()

        # NED
        setpoint.position = [0.0, 0.0, 0.0]

        self.trajectory_publisher.publish(setpoint)
        self.get_logger().info(
            f"Publiserer til TrajectorySetpoint: {setpoint.position}")

    def position_callback(self, msg):
        position = [msg.x, msg.y, msg.z]
        self.get_logger().info(
            f"Mottatt VehicleLocalPosition: {position}")


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
