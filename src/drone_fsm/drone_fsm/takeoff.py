#from px4_msgs.msg import ,,,
import rclpy
import rclpy.node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition)


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
        
        self.offboard_period = 0.3 # The heartbeat signal needs min 2Hz
        self.trajectory_period = 1 # Chose 1 at random
        self.vehicle_command_period = 1 # Chose 1 at random

        self.offboardControlTimer = self.create_timer(self.offboard_period, self.offboardControlMode_callback)
        self.trajectorySetpointTimer = self.create_timer(self.trajectory_period, self.trajectorySetpoint_callback)
        self.vehicleCommandTimer = self.create_timer(self.vehicle_command_period, self.vehicleCommand_callback)

        self.pos_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.update_vehicle_pos, qos_profile)
        self.status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.update_vehicle_status, qos_profile)
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED # Taken from github
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX # Taken from github

        self.last_pos_msg = None
        self.target = [0,0,-3]



    def offboardControlMode_callback(self):
        ocm_msg = OffboardControlMode()
        ocm_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        ocm_msg.position = True
        ocm_msg.velocity = False
        ocm_msg.acceleration=False
        self.offboardControlPublisher_.publish(ocm_msg)

    def trajectorySetpoint_callback(self):
        if self.last_pos_msg is None:
            return

        ts_msg = TrajectorySetpoint()
        ts_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        ts_msg.position = [0.0, 0.0, -3.0]
        ts_msg.yaw = 0.0

        self.trajectorySetpointPublisher_.publish(ts_msg)

    def vehicleCommand_callback(self):
        vc_msg = VehicleCommand()

        if (self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            vc_msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
            vc_msg.param1 = 1.0
            vc_msg.param2 = 6.0

            vc_msg.target_system = 1
            vc_msg.target_component = 1
            vc_msg.source_system = 1
            vc_msg.source_component = 1
            vc_msg.from_external = True
            vc_msg.timestamp = self.get_clock().now().nanoseconds // 1000

            self.vehicleCommandPublisher_.publish(vc_msg)
            return
        
        elif (self.arming_state == VehicleStatus.ARMING_STATE_DISARMED):            
            vc_msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            vc_msg.param1 = 1.0

            vc_msg.target_system = 1
            vc_msg.target_component = 1
            vc_msg.source_system = 1
            vc_msg.source_component = 1
            vc_msg.from_external = True
            vc_msg.timestamp = self.get_clock().now().nanoseconds // 1000

            self.vehicleCommandPublisher_.publish(vc_msg)
            return




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