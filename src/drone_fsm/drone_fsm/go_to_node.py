import rclpy
import math
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from nav_msgs.msg import Path
from std_srvs.srv import Trigger

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition


class GoTo(Node):
    # Tilstandsmaskin
    IDLE = 0
    FLYING = 1
    DONE = 2

    def __init__(self):
        super().__init__("go_to")

        # Parameters
        self.declare_parameter("tolerance", 0.25)
        self.declare_parameter("timer_period", 0.1)
        self.tolerance = self.get_parameter("tolerance").value
        self.state = self.IDLE
        self.waypoints: list = []
        self.current_wp_index = 0
        self.local_position = None

        # QoS profiles
        qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.pub_setpoint = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_pub
        )
        self.pub_offboard_mode = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_pub
        )

        # Service-client for landing
        self.land_client = self.create_client(Trigger, "/land")

        # Subscribe to local position
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self.local_position_cb, qos_sub)

        # Subscriber to waypoints
        self.create_subscription(Path, "/nextwaypoint", self.waypoints_cb, 10)

        # Maintimer
        self.timer = self.create_timer(
            self.get_parameter("timer_period").value, self.control_loop
        )

        self.get_logger().info("Offboard waypoint-node started. Waiting for waypoints")

    # Callbacks
    def waypoints_cb(self, msg: Path):
        if self.state != self.IDLE:
            self.get_logger().warn("Received waypoints, but ignores")
            return  # ignores new waypoints while flying
        if not msg:
            return
        self.waypoints = msg.poses
        self.current_wp_index = 0
        self.state = self.FLYING
        self.get_logger().info(
            f"Received {len(self.waypoints)} waypoints, starting to fly"
        )

    def local_position_cb(self, msg: VehicleLocalPosition):
        self.local_position = msg

    def land_response_cb(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info(f"Landing: {response.message}")
        else:
            self.get_logger().error(f"Landing failed: {response.message}")

    # States
    def request_landing(self):
        if not self.land_client.service_is_ready():
            self.get_logger().error("/land is not available")
            return
        req = Trigger.Request()
        future = self.land_client.call_async(req)
        future.add_done_callback(self.land_response_cb)

    # Convert ENU to NED
    def current_target_wp(self):
        pose = self.waypoints[self.current_wp_index].pose
        return pose.position.y, pose.position.x, -pose.position.z, float("nan")

    def control_loop(self):
        now = int(self.get_clock().now().nanoseconds / 1000)

        # IDLE-check need to come BEFORE heartbeat. Takeoff controlls IDLE,
        # and go_to should not send anything to PX4, neither OffboardControlMode.
        if self.state == self.IDLE:
            return # IDLE: takeoff-node streams hold-position, and go_to-node does not send anything to PX4

        ocm = OffboardControlMode()
        ocm.timestamp = now
        ocm.position = True
        ocm.velocity = ocm.acceleration = ocm.attitude = ocm.body_rate = False

        # Offboard heart beat
        self.pub_offboard_mode.publish(ocm)

        # Trajectory setpoint heart beat
        if self.local_position is None:
            return
    
        if self.state != self.FLYING:
            return  # IDLE: takeoff-node streams hold-position.

        # State is FLYING and has recieved waypoints
        x, y, z, yaw = self.current_target_wp()
        sp = TrajectorySetpoint()
        sp.timestamp = now
        sp.position = [x, y, z]
        sp.yaw = yaw
        self.pub_setpoint.publish(sp)

        # Tolerance calculation
        lp = self.local_position
        dist = math.dist((lp.x, lp.y, lp.z), (x, y, z))
        if dist < self.tolerance:
            self.get_logger().info(
                f"Waypoint {self.current_wp_index + 1}/{len(self.waypoints)} nådd."
            )
            self.current_wp_index += 1
            if self.current_wp_index >= len(self.waypoints):
                self.state = self.DONE
                self.request_landing()
                self.get_logger().info("Ferdig! :3")


def main(args=None):
    rclpy.init(args=args)
    node = GoTo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
