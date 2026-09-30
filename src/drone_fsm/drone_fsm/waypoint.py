import rclpy
from rclpy.node import Node

from std_msgs.msg  import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Waypoint(Node):

    def __init__(self):
        super().__init__(f'{figure_name}_node')

        self.figure = figure_name
        self.waypoints = waypoints
        self.path_sent = False

        # Publisher for the whole figure as a PATH (Subscriber GO_TO - Alex copy topic name)
        self.waypoint_pub = self.create_publisher(Path, '/nextwaypoint', 10)

        # Subscription for takeoff updates from Takeoff node
        self.state_takeoff = self.create_subscription(bool, '/takeoff_done', 10)

        self.get_logger().info('Waypoint node has been started. Figure: {self.figure}' 
                               'Waiting for takeoff..')

    # Runs when take off done message is received. Sends the path to the Go-To node.
    def takeoff_callback(self, msg):
        if msg.data != "TAKEOFF_DONE":
            return
    
        if self.path_sent:
            self.get_logger().warn(f'Path already sent, ignoring extra takeoff message')
            return
    
        self.path_sent()
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
        self.get_logger().info(f'[{self.current_figure}] send path with {len(path.poses)} waypoints.')

    # Runs the figure node, initializing ROS2 and spinning until shutdown.
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

