import rclpy
from rclpy.node import Node

from delta_interfaces.msg import GreeterJob
from delta_interfaces.msg import JobStatus
from threading import Thread

import time


# robot controller imports
from geometry_msgs.msg import Quaternion, PoseStamped
from nav2_msgs.action import Spin, NavigateToPose
from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler
from rclpy.action import ActionClient

class RobotController:

    def __init__(self, node):
    
        self._arrived = False
        self._rotation_complete = False
        self._node = node
        
        # ROS2 Action clients
        self._nav_to_pose_client = ActionClient(self._node, NavigateToPose, 'navigate_to_pose')
        self._spin_client = ActionClient(self._node, Spin, 'spin')
        
        
    def YawToQuaternion(self, angle_z = 0.):
        quat_tf = quaternion_from_euler(0, 0, angle_z)

        # Convert a list to geometry_msgs.msg.Quaternion
        quat_msg = Quaternion(x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3])
        return quat_msg


    def move_to_position(self, x, y, rot):
        
        self._arrived = False
          
        # building the message
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self._node.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation = self.YawToQuaternion(rot)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        goal_msg.behavior_tree = ''
        
        while not self._nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self._node.get_logger().info("'NavigateToPose' action server not available, waiting...")
        
        self._node.get_logger().info('Navigating to goal (x,y,rot): ' + str(goal_pose.pose.position.x) + ' ' +
                  str(goal_pose.pose.position.y) + ' ' + str(rot))
                  
        self._send_move_goal_future = self._nav_to_pose_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        
        self._send_move_goal_future.add_done_callback(self.move_goal_response_callback)
        
    def rotate(self, spin_dist_in_degree):
    
        self._rotation_complete = False
    
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = spin_dist_in_degree
        
        while not self._spin_client.wait_for_server(timeout_sec=1.0):
            self._node.get_logger().info("'Spin' action server not available, waiting...")
        self._node.get_logger().info(f'Spinning to angle {goal_msg.target_yaw}....')
        
        self._send_rotate_goal_future = self._spin_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        
        self._send_rotate_goal_future.add_done_callback(self.rotate_goal_response_callback)
        

    def move_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().info('Goal rejected :(')
            self._arrived = True
            return

        self._node.get_logger().info('Goal accepted :)')

        self._get_move_result_future = goal_handle.get_result_async()
        self._get_move_result_future.add_done_callback(self.get_move_result_callback)
        
    def rotate_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().info('Goal rejected :(')
            self._rotation_complete = True
            return

        self._node.get_logger().info('Goal accepted :)')

        self._get_rotate_result_future = goal_handle.get_result_async()
        self._get_rotate_result_future.add_done_callback(self.get_rotate_result_callback)


    def get_move_result_callback(self, future):
        result = future.result()
        self._arrived = True
        
    def get_rotate_result_callback(self, future):
        result = future.result()
        self._rotation_complete = True
        

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback


class Greeter(Node):

    def __init__(self):
        super().__init__('greeter')
        
        # information about the currently executed job
        self.currently_executing_job = False # is the job still beeing processed
        self.id_of_current_job = ""
        
        # publishing the jobs status
        self.publisher_ = self.create_publisher(JobStatus, 'job_status', 1)
        timer_period = 1.0  # seconds
        self.publish_status_timer = self.create_timer(timer_period, self.publish_status)
        
        # listen to incoming jobs
        self.subscription = self.create_subscription(GreeterJob, 'greeter_job', self.process_incoming_job, 1)
        self.subscription  # prevent unused variable warning
        
        # robot controller
        self.rc = RobotController(self)
        

    def publish_status(self):
        msg = JobStatus()
        msg.acting = self.currently_executing_job
        msg.job_id = self.id_of_current_job
        self.publisher_.publish(msg)
        
        
    def process_incoming_job(self, msg):
        if self.id_of_current_job == msg.job_id or self.currently_executing_job == True:
            return
        else:
            self.id_of_current_job = msg.job_id
            self.currently_executing_job = True
            self.publish_status()

        thread = Thread(target=self.greet_a_person, args=(msg.position_x, msg.position_y, msg.position_z, msg.rotation))
        thread.start()
        
    
    def greet_a_person(self, position_x, position_y, position_z, rotation):
            
        # moving to person
        self.get_logger().info('moving to greet person at (x: %f  y: %f  rot: %f)' % (position_x, position_y, rotation))
        self.rc.move_to_position(position_x, position_y, rotation)
        while not self.rc._arrived:
                time.sleep(1)
                self.get_logger().info('waiting until robot arrives at person')
        # greeting the person
        self.get_logger().info('.__________________________________________.')
        self.get_logger().info('|                                          |')
        self.get_logger().info('|                                          |')
        self.get_logger().info('|                                          |')
        self.get_logger().info('|        Dober dan, person object :)       |')
        self.get_logger().info('|                                          |')
        self.get_logger().info('|                                          |')
        self.get_logger().info('|                                          |')
        self.get_logger().info('.__________________________________________.')
        
        # IMPORTANT: after greeting has finished, set currently_executing_job to False
        self.currently_executing_job = False
        self.publish_status()
        
    def destroyNode(self):
        self.rc._nav_to_pose_client.destroy()
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    greeter = Greeter()
    rclpy.spin(greeter)
    
    greeter.destroyNode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
