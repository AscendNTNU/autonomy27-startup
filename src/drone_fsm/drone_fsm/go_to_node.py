import rclpy
import math
from rclpy.node import node
from rclpy.qos import QoSProfile, QoSReliability, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleLocalPosition,
    VehicleStatus,
)

class GoTo(Node):
    #Tilstandsmaskin
    IDLE = 0
    FLYING = 1
    DONE = 2

    def __init__(self):
        super().__init__('go_to')

        #Parameters
        self.declare_parameter('tolerance', 0.5)
        self.declare_parameter('timer_period', 0.1)
        self.tolerance = self.get_parameter('tolerance').value
        self.state = self.IDLE
        self.waypoints: list = []
        self.current_wp_index = 0 
        self.local_position = None 

        #QoS profiles
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        #Publishers
        self.pub_setpoint = self.create_publisher(
            TrajectorySetpoint,  '/fmu/in/trajectory_setpoint', qos_pub
        )
        self.pub_offboard_mode = self.create_publisher(
           OffboardControlMode, '/fmu/in/offboard_control_mode', qos_pub
        )

        #Service-client 
        self.land_client = self.create_client(Trigger, '/land')
        while not self.land_client.wait_for_service(timeout_sec=1.0):
            self.get_logget().info('Venter på /land-service')

        #Subscribers
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_cb, qos_sub
        )

        #Subscriber til waypoints
        self.create_subscription(
            Path, '/nextwaypoint', self.waypoints_cb, 10
        )

        #Hovedtimer
        self.timer = self.create_timer(
            self.get_parameter('timer_period').value, self.control_loop
        )

        self.get_logger().info(
            'Offboard waypoint-node started. Waiting for waypoints'
        )

    #Callbacks
    def waypoints_cb(self, msg: PoseStamped):
        if self.state != self.IDLE:
            self.get_logger().warn('Mottok waypoints, men ignorerer')
            return  #ignorer nye waypoints mens vi flyr
        if not msg:
            return
        self.waypoints = list(msg)
        self.current_wp_index = 0 
        self.state = self.FLYING
        self.get_logger().info(f'Mottok {len(self.waypoints)} waypoints, starter flyging')

    def local_position_cb(self, msg: VehicleLocalPosition):
        self.local_position = msg
    
    def land_response_cb(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info(f'Landing: {response.message}')
        else:
            self.get_logger().error(f'Landing feilet: {response.message}')

    #hendelser
    def request_landing(self):
        if not self.land_client.service_is_ready():
            self.get_logger().error('/land er ikke tilgjengelig')
            return
        req = Trigger.Request()
        future = self.land_client.call_async(req)
        future.add_done_callback(self.land_response_cb)

    @staticmethod
    def yaw_from_quaternion(w, x, y, z):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def current_target_wp(self):
        pose = self.waypoints[self.current_wp_index].pose
        q = pose.orientation
        yaw_enu = self.yaw_from_quaternion(q.w, q.x, q.y, q.z)
        return pose.position.x, -pose.position.y, -pose.position.z, -yaw_enu

    def control_loop(self):
        ocm = OffboardControlMode()
        ocm.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        ocm.position = True
        ocm.velocity = ocm.acceleration = ocm.attitude = ocm.body_rate = False
        self.pub_offboard_mode.publish(ocm)

        if self.state != self.FLYING or self.local_position is None:
            return # IDLE: takeoff-noden strømmer hold-posisjon

        x, y, z, yaw = self.current_target_wp()
        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        sp.position = [x, y, z]
        sp.yaw = yaw
        self.pub_setpoint.publish(sp)

        lp = self.local_position
        dist = math.dist((lp.x, lp.y, lp.z), (x, y, z))
        if dist < self.tolerance:
            self.get_logger().info(
                f'Waypoint {self.current_wp_index + 1}/{len(self.waypoints)} nådd.'
            )
            self.current_wp_index += 1
            if self.current_wp_index >= len(self.waypoints):
                self.state = self.DONE
                self.request_landing()
                self.get_logger().info('Ferdig!')

def main():
    rclpy.init(args=args)
    node = GoTo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

