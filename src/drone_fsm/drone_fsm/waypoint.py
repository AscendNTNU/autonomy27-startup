import rclpy
from rclpy.node import Node

from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSDurabilityPolicy, QoSHistoryPolicy)

from std_msgs.msg import Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Waypoint(Node):

    def __init__(self, figure_name, waypoints):
        super().__init__(f'{figure_name}_node')

        self.figure = figure_name
        self.waypoints = waypoints
        self.path_sent = False
        
        path_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 1)

        # Publisher for the whole figure as a PATH
        self.waypoint_pub = self.create_publisher(Path, '/nextwaypoint', path_qos)

        takeoff_qos = QoSProfile(
                            reliability = QoSReliabilityPolicy.RELIABLE,
                            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
                            history = QoSHistoryPolicy.KEEP_LAST,
                            depth = 1)

        # Subscription for takeoff updates from Takeoff node
        self.state_takeoff = self.create_subscription(Bool, '/takeoff_done', self.takeoff_callback, takeoff_qos)

        
    # Runs when take off done message is received. Sends the path to the Go-To node.
    def takeoff_callback(self, msg):
        if not msg.data:
            return  # takeoff not done yet
 
        if self.path_sent:
            self.get_logger().warn('Path already sent, ignoring extra takeoff message')
            return
 
        self.send_path()
        self.path_sent = True
 
    # Sends the path to the Go-To node as a Path message.
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
        self.get_logger().info(
            f'[{self.figure}] sent path with {len(path.poses)} waypoints.')
 
 
def run_figure(figure_name, waypoints, args=None):
    rclpy.init(args=args)
    node = Waypoint(figure_name, waypoints)
 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()