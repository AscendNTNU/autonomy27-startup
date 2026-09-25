import rclpy
from rclpy.node import Node

class Square(Node):

    def __init__(self):
        super().__init__('waypoint')
        self.state = "takeoff"
        self.waypoint_index = 0

        self.waypoints = [
            (0.0, 0.0, -3.0),  # Waypoint 1: Takeoff to 3 meters altitude
            (3.0, 0.0, -3.0),  # Waypoint 2: Move to (3, 0) at 3 meters altitude
            (3.0, 3.0, -3.0),  # Waypoint 3: Move to (3, 3) at 3 meters altitude
            (0.0, 3.0, -3.0),  # Waypoint 4: Move to (0, 3) at 3 meters altitude
        ]
        self.get_logger().info('Waypoint 1 node has been started.')

def main(args=None):
    rclpy.init(args=args)
    waypoint_node = Square()
    rclpy.spin(waypoint_node)
    waypoint_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
