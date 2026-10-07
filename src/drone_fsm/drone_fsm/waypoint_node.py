import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import SetBool


class Waypoint(Node):

    def __init__(self, figure_name, waypoints):
        super().__init__(f'{figure_name}_node')

        self.figure = figure_name
        self.waypoints = waypoints
        self.path_sent = False

        path_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher for the whole figure as a PATH
        self.waypoint_pub = self.create_publisher(
            Path,
            '/nextwaypoint',
            path_qos
        )

        takeoff_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Service for takeoff updates from Takeoff node
        self.state_takeoff = self.create_service(
            SetBool,
            '/takeoff_finished',
            self.takeoff_callback
        )

        self.get_logger().info(
            f'[{self.figure}] started with {len(self.waypoints)} waypoints. '
            'Waiting for /takeoff_finished...')

    # Runs when take off done service is called. Sends the path to the Go-To node.
    def takeoff_callback(self, request, response):
        self.get_logger().info(f'[{self.figure}] received /takeoff_finished: {request.data}')

        if not request.data:
            response.success = False
            response.message = ''
            return response

        if self.path_sent:
            self.get_logger().warning('Path already sent, ignoring extra takeoff message')
            response.success = True
            response.message = ''
            return response

        if self.send_path():
            self.path_sent = True
            response.success = True
        else:
            response.success = False

        response.message = ''
        return response

    # Sends the path to the Go-To node as a Path message.
    def send_path(self):
        if not self.waypoints:
            self.get_logger().error(f'[{self.figure}] cannot send an empty waypoint path.')
            return False

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'local_enu'  # ENU for ROS 2 and Gazebo

        for east, north, up in self.waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = east
            pose.pose.position.y = north
            pose.pose.position.z = up
            pose.pose.orientation.w = 1.0  # No rotation

            path.poses.append(pose)

        self.waypoint_pub.publish(path)

        # Choose where the figure starts and ends so its easy to see if the right points are sent.
        first = self.waypoints[0]
        last = self.waypoints[-1]

        self.get_logger().info(
                    f'[{self.figure}] sent path with {len(path.poses)} waypoints '
                    f'(ENU, first: {first}, last: {last})')
        
        return True


def run_figure(figure_name, waypoints, args=None):
    rclpy.init(args=args)
    node = Waypoint(figure_name, waypoints)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
