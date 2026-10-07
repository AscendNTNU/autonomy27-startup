import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition
from std_srvs.srv import SetBool, Trigger


class TakeoffNode(Node):
    
    def __init__(self):
        super().__init__('takeoff_node')
        self.get_logger().info('TakeoffNode has been initialized.')

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.offboardControlPublisher_ = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile)
        self.trajectorySetpointPublisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicleCommandPublisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.takeoffFinishedClient_ = self.create_client(SetBool, '/takeoff_finished')
        self.landClient_ = self.create_client(Trigger, '/land')

        self.offboard_period = 0.3  # The heartbeat signal needs min 2Hz
        self.trajectory_period = 1  # Chose 1 at random

        self.last_trajectory_time = self.get_clock().now()
        self.timer = self.create_timer(self.offboard_period, self.timer_callback)

        # Topic-name have versionchange because VehicleLocalPosition have MESSAGE_VERSION = 1
        # in px4_msgs (PX4 b7e991cd8c). Check with `ros2 topic list | grep fmu` of PX4 updates.
        self.pos_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.update_vehicle_pos, qos_profile) # Change to '/fmu/out/vehicle_local_position_v1' for gazebo sim

        # VehicleStatus has MESSAGE_VERSION = 4 in our version of px4_msgs, so the topic is named vehicle_status_v4.
        
        self.status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.update_vehicle_status, qos_profile) # Change to '/fmu/out/vehicle_status_v4' for gazebo sim

        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX

        self.last_pos_msg = None
        self.target = [0, 0, -3]

        # States: takeoff, takeoff_finished, landing_service, emergency_return, done
        self.state = 'takeoff'

        self.service_future = None
        self.service_counter = 0
        self.last_counter_time = None
        self.done = False


    def timer_callback(self):

        # Normal takeoff
        if self.state == 'takeoff':
            ocm_msg = OffboardControlMode()
            ocm_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            ocm_msg.position = True
            ocm_msg.velocity = False
            ocm_msg.acceleration = False
            self.offboardControlPublisher_.publish(ocm_msg)

            # External system must arm and put PX4 into OFFBOARD.
            if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD or self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                return

            now = self.get_clock().now()

            if (now - self.last_trajectory_time).nanoseconds / 1e9 < self.trajectory_period:
                return

            self.last_trajectory_time = now

            if self.last_pos_msg is None:
                return

            ts_msg = TrajectorySetpoint()
            ts_msg.timestamp = self.get_clock().now().nanoseconds // 1000
            ts_msg.position = [0.0, 0.0, -3.0]
            ts_msg.yaw = 0.0
            self.trajectorySetpointPublisher_.publish(ts_msg)

            if abs(self.last_pos_msg.x) < 0.2 and abs(self.last_pos_msg.y) < 0.2 and abs(self.last_pos_msg.z + 3) < 0.2:
                self.get_logger().info('Takeoff done. Contacting /takeoff_finished.')
                self.state = 'takeoff_finished'
                self.service_counter = 0
                self.last_counter_time = self.get_clock().now()
                self.service_future = None

            return


        # Contact /takeoff_finished
        elif self.state == 'takeoff_finished':
            # Keep heartbeat alive while waiting for handover.
            ocm_msg = OffboardControlMode()
            ocm_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            ocm_msg.position = True
            ocm_msg.velocity = False
            ocm_msg.acceleration = False
            self.offboardControlPublisher_.publish(ocm_msg)

            # Hold the takeoff position while waiting.
            ts_msg = TrajectorySetpoint()
            ts_msg.timestamp = self.get_clock().now().nanoseconds // 1000
            ts_msg.position = [0.0, 0.0, -3.0]
            ts_msg.yaw = 0.0
            self.trajectorySetpointPublisher_.publish(ts_msg)

            # Send request if one has not already been sent.
            if self.service_future is None and self.takeoffFinishedClient_.service_is_ready():
                request = SetBool.Request()
                request.data = True
                self.service_future = self.takeoffFinishedClient_.call_async(request)
                self.get_logger().info('Sent True to /takeoff_finished.')

            # Check for response.
            if self.service_future is not None and self.service_future.done():
                try:
                    response = self.service_future.result()

                    if response.success:
                        self.get_logger().info('/takeoff_finished responded with 1. Giving control away.')
                        self.state = 'done'
                        self.done = True
                        self.timer.cancel()
                        return

                except Exception as e:
                    self.get_logger().error(f'/takeoff_finished call failed: {e}')

                self.service_future = None

            # Increase counter once every second.
            now = self.get_clock().now()

            if (now - self.last_counter_time).nanoseconds / 1e9 >= 1.0:
                self.service_counter += 1
                self.last_counter_time = now
                self.get_logger().info(f'/takeoff_finished wait: {self.service_counter}/10')

            # No valid response after 10 seconds.
            if self.service_counter >= 10:
                self.get_logger().warning('/takeoff_finished did not respond with 1. Trying /land.')
                self.state = 'landing_service'
                self.service_counter = 0
                self.last_counter_time = self.get_clock().now()
                self.service_future = None

            return


        # Try landing node
        elif self.state == 'landing_service':
            # Keep heartbeat and position while waiting for the landing node to accept control.
            ocm_msg = OffboardControlMode()
            ocm_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            ocm_msg.position = True
            ocm_msg.velocity = False
            ocm_msg.acceleration = False
            self.offboardControlPublisher_.publish(ocm_msg)

            ts_msg = TrajectorySetpoint()
            ts_msg.timestamp = self.get_clock().now().nanoseconds // 1000
            ts_msg.position = [0.0, 0.0, -3.0]
            ts_msg.yaw = 0.0
            self.trajectorySetpointPublisher_.publish(ts_msg)

            # Contact the landing node.
            if self.service_future is None and self.landClient_.service_is_ready():
                request = Trigger.Request()
                self.service_future = self.landClient_.call_async(request)
                self.get_logger().info('Sent request to /land.')

            # Check landing-node response.
            if self.service_future is not None and self.service_future.done():
                try:
                    response = self.service_future.result()

                    if response.success:
                        self.get_logger().info('/land accepted the request. Giving control to landing node.')
                        self.state = 'done'
                        self.done = True
                        self.timer.cancel()
                        return

                except Exception as e:
                    self.get_logger().error(f'/land call failed: {e}')

                self.service_future = None

            # Increase counter once per second.
            now = self.get_clock().now()

            if (now - self.last_counter_time).nanoseconds / 1e9 >= 1.0:
                self.service_counter += 1
                self.last_counter_time = now
                self.get_logger().info(f'/land wait: {self.service_counter}/10')

            # Landing node failed too.
            if self.service_counter >= 10:
                self.get_logger().warning('/land did not respond. Starting fallback return to [0, 0, 0].')
                self.state = 'emergency_return'
                self.service_counter = 0
                self.service_future = None

            return


        # Final fallback
        elif self.state == 'emergency_return':
            if self.last_pos_msg is None:
                return

            ocm_msg = OffboardControlMode()
            ocm_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            ocm_msg.position = True
            ocm_msg.velocity = False
            ocm_msg.acceleration = False
            self.offboardControlPublisher_.publish(ocm_msg)

            # Go back to [0, 0, 0].
            ts_msg = TrajectorySetpoint()
            ts_msg.timestamp = self.get_clock().now().nanoseconds // 1000
            ts_msg.position = [0.0, 0.0, 0.0]
            ts_msg.yaw = 0.0
            self.trajectorySetpointPublisher_.publish(ts_msg)

            # Only disarm once the drone is effectively at [0, 0, 0].
            if abs(self.last_pos_msg.x) < 0.2 and abs(self.last_pos_msg.y) < 0.2 and abs(self.last_pos_msg.z) < 0.2:
                vc_msg = VehicleCommand()
                vc_msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
                vc_msg.param1 = 0.0
                vc_msg.target_system = 1
                vc_msg.target_component = 1
                vc_msg.source_system = 1
                vc_msg.source_component = 1
                vc_msg.from_external = True
                vc_msg.timestamp = self.get_clock().now().nanoseconds // 1000

                self.vehicleCommandPublisher_.publish(vc_msg)

                self.get_logger().warning('Fallback reached [0, 0, 0]. Sent disarm command.')
                self.state = 'done'
                self.done = True
                self.timer.cancel()

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