import rclpy
from rclpy.node import Node

from delta_interfaces.msg import RingObjects
from delta_interfaces.msg import CylinderObjects


class PubDemo(Node):

    def __init__(self):
        super().__init__('pub_demo')
        
        # creating the publishers and a timer
        self.ring_publisher = self.create_publisher(RingObjects, 'ring_objects', 1)
        self.cylinder_publisher = self.create_publisher(CylinderObjects, 'cylinder_objects', 1)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publishStuff)
        self.get_logger().info('started publisher demo')
        
    def publishStuff(self):
        self.get_logger().info('publishing')
    
        ring_msg = RingObjects()
        ring_msg.position_x = [-1.1, -1.1, 1.1, 2.7]
        ring_msg.position_y = [0.25, 3.0, 1.0, -1.7]
        ring_msg.position_z = [1.0, 1.0, 1.0, 1.0]
        ring_msg.color = ["black", "blue", "red", "green"]
        ring_msg.id = ["ring_1", "ring_2", "ring_3", "ring_4"]
        self.ring_publisher.publish(ring_msg)


def main(args=None):
    rclpy.init(args=args)
    pubDemo = PubDemo()
    rclpy.spin(pubDemo)
    
    pubDemo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
