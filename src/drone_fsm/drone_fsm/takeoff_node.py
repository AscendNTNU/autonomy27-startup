#from px4_msgs.msg import ,,,
import rclpy
import rclpy.node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition)
from std_msgs import msg

class TakeoffNode(rclpy.node.Node):
    
    def __init__(self):
        super().__init__('takeoff_node')
        self.get_logger().info('TakeoffNode has been initialized.')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboardControlPublisher_ = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile)
        self.trajectorySetpointPublisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicleCommandPublisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.takeoffFinishedPublisher_ = self.create_publisher(msg.Bool, 'takeoff_done', qos_profile)
        
        self.period_ = 0.1 # 10Hz timer

        self.offboardControlTimer = self.create_timer(self.period_, self.timer_callback)

        self.pos_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.update_vehicle_pos, qos_profile)
        self.status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.update_vehicle_status, qos_profile)
        
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED # Taken from github
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX # Taken from github

        self.last_pos_msg = None
        self.target = [0,0,-3]


    def timer_callback(self):
        # Heartbeat to put into offboard mode
        ocm_msg = OffboardControlMode()
        ocm_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        ocm_msg.position = True
        ocm_msg.velocity = False
        ocm_msg.acceleration=False
        self.offboardControlPublisher_.publish(ocm_msg)

        # if the drone has its local position, then got to this position if/when ready
        if self.last_pos_msg is not None:
            ts_msg = TrajectorySetpoint()
            ts_msg.timestamp = self.get_clock().now().nanoseconds // 1000
            ts_msg.position = [0.0, 0.0, -3.0]
            ts_msg.yaw = 0.0

            self.trajectorySetpointPublisher_.publish(ts_msg)
            self.get_logger().info('Publishing setpoint [0, 0, -3]')
            

            if (abs(self.last_pos_msg.x) < 0.2 and
                abs(self.last_pos_msg.y) < 0.2 and
                abs(self.last_pos_msg.z + 3) < 0.2):
                
                true_msg = msg.Bool()
                true_msg.data = True
                self.get_logger().info('TakeoffNode is at [0, 0, -3]')
                self.get_logger().info('TakeoffNode startes handoff')
                for i in range(100):
                    self.takeoffFinishedPublisher_.publish(true_msg)
                rclpy.shutdown()
            
           

    def update_vehicle_status(self, msg):
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state

    def update_vehicle_pos(self, msg):
        self.last_pos_msg = msg

        

def main(args=None):
    rclpy.init(args=args)

    takeoffNode = TakeoffNode()
    
    try:
        rclpy.spin(takeoffNode)
    except KeyboardInterrupt:
        pass

    takeoffNode.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()