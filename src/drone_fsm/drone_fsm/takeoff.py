#from px4_msgs.msg import ,,,
import rclpy
import rclpy.node
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition)


class TakeoffNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('takeoff_node')
        self.get_logger().info('TakeoffNode has been initialized.')

        self.offboardControlPublisher_ = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.trajectorySetpointPublisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicleCommandPublisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        
        self.offboard_period = 0.3 # The heartbeat signal needs min 2Hz
        self.trajectory_period = 1 # Chose 1 at random
        self.vehicle_command_period = 1 # Chose 1 at random

        self.offboardControlTimer = self.create_timer(self.offboard_period, self.offboardControlMode_callback)
        self.trajectorySetpointTimer = self.create_timer(self.trajectory_period, self.trajectorySetpoint_callback)
        self.vehicleCommandTimer = self.create_timer(self.vehicle_command_period, self.vehicleCommand_callback)

        self.pos_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.update_vehicle_pos)
        self.status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.update_vehicle_status)
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED # Taken from github
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX # Taken from github

        self.has_been_told_to_go_up = False
        self.last_pos_msg = None

        self.is_first_pos_msg = True
        self.target = [0,0,-3]



    def offboardControlMode_callback(self):
        ocm_msg = OffboardControlMode()
        ocm_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        ocm_msg.position = True
        ocm_msg.velocity = False
        ocm_msg.acceleration=False
        self.offboardControlPublisher_.publish(ocm_msg)

    def trajectorySetpoint_callback(self):
        if not (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED and (not self.has_been_told_to_go_up)):
            return
        if self.last_pos_msg is None:
            return
        ts_msg = TrajectorySetpoint()
        ts_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        ts_msg.position[0] = self.target[0]
        ts_msg.position[1] = self.target[1]
        ts_msg.position[2] = self.target[2]

        self.trajectorySetpointPublisher_.publish(ts_msg)

    def vehicleCommand_callback(self):
        vc_msg = VehicleCommand()

        if (self.arming_state == VehicleStatus.ARMING_STATE_DISARMED):
            vc_msg.param1 = 1 # 1 is arm, 0 is unarm
            vc_msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM # Constant 1
            self.vehicleCommandPublisher_.publish(vc_msg)
            return

        elif (self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
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
