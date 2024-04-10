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
    
        ring_msg = RingObjects() # rings at (1,1,1), (2,2,2) and (3,3,3)
        ring_msg.position_x = [1.0, 2.0, 3.0]
        ring_msg.position_y = [1.0, 2.0, 3.0]
        ring_msg.position_z = [1.0, 2.0, 3.0]
        ring_msg.color = ["red", "blue", "green"]
        ring_msg.id = ["ring_1", "ring_2", "ring_3"]
        self.ring_publisher.publish(ring_msg)
        
        cylinder_msg = CylinderObjects() # cylinders at (10,10,10), (20,20,20) and (30,30,30)
        cylinder_msg.position_x = [10.0, 20.0, 30.0]
        cylinder_msg.position_y = [10.0, 20.0, 30.0]
        cylinder_msg.position_z = [10.0, 20.0, 30.0]
        cylinder_msg.color = ["red", "blue", "green"]
        cylinder_msg.id = ["cylinder_1", "cylinder_2", "cylinder_3"]
        self.cylinder_publisher.publish(cylinder_msg)


def main(args=None):
    rclpy.init(args=args)
    pubDemo = PubDemo()
    rclpy.spin(pubDemo)
    
    pubDemo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
