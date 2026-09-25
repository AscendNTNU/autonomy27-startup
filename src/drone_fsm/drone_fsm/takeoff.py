#from px4_msgs.msg import ,,,
import rclpy
import rclpy.node


def main():
    print('The takeoff node is now running.')
    # 


class TakeoffNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('takeoff_node')
        self.get_logger().info('TakeoffNode has been initialized.')
        # TODO: Create publisher here, using MAVROS?
        # TODO: Find the topics and values that needs to be published in order for the drone
        # to enter and stay in Offboardmode

        self.timer_period = 0.5

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        # 



if __name__ == '__main__':
    main()
