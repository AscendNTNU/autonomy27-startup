import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy)

from std_msgs.msg  import String
from nav_msgs.msg import Path
from px4_msgs.msg import VehicleLocalPosition

FIGURES = {'square': [
            (0.0, 0.0, -3.0),  # Waypoint 1: Takeoff to 3 meters altitude, above start point
            (3.0, 0.0, -3.0),  # Waypoint 2: Move to (3, 0) at 3 meters altitude
            (3.0, 3.0, -3.0),  # Waypoint 3: Move to (3, 3) at 3 meters altitude
            (0.0, 3.0, -3.0),  # Waypoint 4: Move to (0, 3) at 3 meters altitude
            (0.0, 0.0, -3.0)   # Waypoint 5: Return to start point at 3 meters altitude
        ],}

class Waypoint(Node):

    def __init__(self):
        super().__init__('waypoint')

        self.state = "waiting"
        self.current_figure = None
        self.waypoints = []
        self.waypoint_index = 0
        self.tolerance = 0.2  # Tolerance for reaching a waypoint in meters

        # Publisher for the whole figure as a PATH (Subscriber GO_TO - Alex copy topic name)
        self.waypoint_pub = self.create_publisher(Path, '/figure/path', 10)

        # Publisher for finished figure (Subscribers GO_TO, LANDING)
        self.command_pub = self.create_publisher(String, '/finished', 10)

        # Subscriber for updates when to start a new figure (Publisher GO_TO - Alex have you made a topic?)
        self.start_figure_sub = self.create_subscription(String, '/figure/start', self.start_figure_callback, 10)

        #TODO: Initialize subscription for takeoff updates
        #self.state = self.create_subscription()
        
        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history= QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        # Subscriber to Alex's local (GO_TO) position topic
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.waypoint_callback,
            px4_qos)

        self.get_logger().info('Waypoint node has been started. Figure: {list(FIGURES.keys())}')

    # Runs when someone asks the node to start a new figure
    def start_figure_callback(self, msg):
        if self.state != "waiting":
            self.get_logger().warn('Cannot start a new figure while another is in progress. ' \
                'Current: {self.current_figure}, ignoring {msg.data}')
            return
        if msg.data not in FIGURES:
            self.get_logger().error(f'Unknown figure: {msg.data}')
            return

        self.current_figure = msg.data
        self.waypoints = FIGURES[msg.data]
        self.waypoint_index = 0
        self.state = "flying"
        self.get_logger().info(f'Starting figure: {self.current_figure}')
        self.send_path()

    def send_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'local_ned'  # PX4 local NED coordinates

        for x, y, z in self.waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0  # No rotation

            path.poses.append(pose)

        self.waypoint_pub.publish(path)
        self.get_logger().info(f']{self.current_figure}] send path with {len(path.poses)} waypoints.')


    # Runs every time PX4 sends a new position. 
    # Tracks progress through waupoints in order so we know when figure is done
    def position_callback(self, msg):
        if self.state != "flying":
            return # check only position when the drone flies a figure

        tx, ty, tz = self.waypoints[self.waypoint_index]
        distance = math.sqrt((msg.x - tx)**2 + (msg.y - ty)**2 + (msg.z - tz)**2)

        if distance < self.tolerance:
            self.get_logger().info(f'Waypoint {self.waypoint_index + 1} reached.')
            self.waypoint_index += 1

            if self.waypoint_index < len(self.waypoints):
                self.send_next_waypoint()
            else: 
                self.finish_figure()

    # Tells the others the figure is done, and get ready for the next one
    def finish_figure(self):
     self.get_logger().info(f'Figure {self.current_figure} completed.')
     self.command_pub.publish(String(data=self.current_figure))
     self.state = "waiting"
     self.current_figure = None

def main(args=None):
    rclpy.init(args=args)
    waypoint_node = Waypoint()
    rclpy.spin(waypoint_node)
    waypoint_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
