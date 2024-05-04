import rclpy
from rclpy.node import Node

import time

from delta_interfaces.msg import ParkingJob
from delta_interfaces.msg import JobStatus
from threading import Thread

# robot controller imports
from geometry_msgs.msg import Quaternion, PoseStamped
from nav2_msgs.action import Spin, NavigateToPose, DriveOnHeading
from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler
from rclpy.action import ActionClient

# publishing markers
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from builtin_interfaces.msg import Duration
from irobot_create_msgs.msg import AudioNoteVector, AudioNote


class RobotController:

    def __init__(self, node):
    
        self._arrived = False
        self._rotation_complete = False
        self._move_forward_complete = False
        self._node = node
        
        # ROS2 Action clients
        self._nav_to_pose_client = ActionClient(self._node, NavigateToPose, 'navigate_to_pose')
        self._spin_client = ActionClient(self._node, Spin, 'spin')
        self._drive_on_heading_client = ActionClient(self._node, DriveOnHeading, "drive_on_heading")
        
        self._move_x = None
        self._move_y = None
        self._move_rot = None
        self._rotate_rot = None
        
        
    def YawToQuaternion(self, angle_z = 0.):
        quat_tf = quaternion_from_euler(0, 0, angle_z)

        # Convert a list to geometry_msgs.msg.Quaternion
        quat_msg = Quaternion(x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3])
        return quat_msg


    def move_to_position(self, x, y, rot):
        self._move_x = x
        self._move_y = y
        self._move_rot = rot
        
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
        
    def rotate(self, spin_dist_in_rad):
        self._rotate_rot = spin_dist_in_rad
    
        self._rotation_complete = False
    
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = spin_dist_in_rad
        
        while not self._spin_client.wait_for_server(timeout_sec=1.0):
            self._node.get_logger().info("'Spin' action server not available, waiting...")
        self._node.get_logger().info(f'Spinning to angle {goal_msg.target_yaw}....')
        
        self._send_rotate_goal_future = self._spin_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        
        self._send_rotate_goal_future.add_done_callback(self.rotate_goal_response_callback)
        
    def drive_forward(self, distance = 0.15, speed = 0.5):
        self._move_forward_complete = False
        
        drive_msg = DriveOnHeading.Goal()
        targetPoint = Point()
        targetPoint.x = distance
        drive_msg.target = targetPoint
        drive_msg.speed = speed
        
        while not self._drive_on_heading_client.wait_for_server(timeout_sec=1.0):
            self._node.get_logger().info("'DriveOnHeading' action server not available, waiting...")
        self._node.get_logger().info(f'Moving forward ....')
        
        self._send_move_forward_goal_future = self._drive_on_heading_client.send_goal_async(drive_msg, feedback_callback=self.feedback_callback)
        
        self._send_move_forward_goal_future.add_done_callback(self.drive_forward_response_callback)
    

    def move_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().info('Goal rejected :(')
            self.move_to_position(self._move_x, self._move_y, self._move_rot)
            return

        self._node.get_logger().info('Goal accepted :)')

        self._get_move_result_future = goal_handle.get_result_async()
        self._get_move_result_future.add_done_callback(self.get_move_result_callback)
        
    def rotate_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().info('Goal rejected :(')
            self.rotate(self._rotate_rot)
            return

        self._node.get_logger().info('Goal accepted :)')

        self._get_rotate_result_future = goal_handle.get_result_async()
        self._get_rotate_result_future.add_done_callback(self.get_rotate_result_callback)
        
    def drive_forward_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().info('Goal rejected :(')
            self.drive_forward()
            return
            
        self._node.get_logger().info('Goal accepted :)')
        
        self._get_move_forward_result_future = goal_handle.get_result_async()
        self._get_move_forward_result_future.add_done_callback(self.get_move_forward_callback)
        

    def get_move_result_callback(self, future):
        result = future.result()
        self._arrived = True
        
    def get_rotate_result_callback(self, future):
        result = future.result()
        self._rotation_complete = True
        
    def get_move_forward_callback(self, future):
        result = future.result()
        self._move_forward_complete = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback


class Parking(Node):

    def __init__(self):
        super().__init__('parking')
        
        # information about the currently executed job
        self.currently_executing_job = False # is the job still beeing processed
        self.id_of_current_job = ""
        
        # publishing the jobs status
        self.publisher_ = self.create_publisher(JobStatus, 'job_status', 1)
        timer_period = 1.0  # seconds
        self.publish_status_timer = self.create_timer(timer_period, self.publish_status)
        
        # listen to incoming jobs
        self.subscription = self.create_subscription(ParkingJob, 'parking_job', self.process_incoming_job, 1)
        self.subscription  # prevent unused variable warning
        
        # robot controller
        self.rc = RobotController(self)
        
        # For publishing the markers
        self.marker_pub = self.create_publisher(Marker, "/delta_nav_marker", QoSReliabilityPolicy.BEST_EFFORT)


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

        thread = Thread(target=self.park_at_position, args=(msg.position_x, msg.position_y, msg.position_z))
        thread.start()
        
    def park_at_position(self, position_x, position_y, position_z):

        # moving to parking spot
        self.get_logger().info('parking at (x: %f  y: %f)' % (position_x, position_y))
        self.rc.move_to_position(position_x, position_y, 0.0)
        while not self.rc._arrived:
                time.sleep(1)
                self.get_logger().info('waiting until robot arrives at parking location')
                # Publish a marker
                self.send_marker(position_x, position_y)
                self.send_marker(position_x - 0.1, position_y, 1, 0.15, "parking_nav_goal")
                
                
        # parking user infos
        self.get_logger().info('arrived at parking spot. beginning with parking')        
        self.send_marker(position_x - 0.1, position_y, 1, 0.15, "parking_in_progress")
        
        # just testing the robots movement commands
        self.rotate(3.14) # rotation: positive value -> anti clock wise. 6.3 = 2 pi = one full turn
        self.move_forward(.1) # move 0.1 meters
        
        
        # IMPORTANT: after greeting has finished, set currently_executing_job to False
        self.currently_executing_job = False
        self.publish_status()
        
    def move_forward(self, distanceInMeters):
        self.rc.drive_forward(distanceInMeters)
        while not self.rc._move_forward_complete:
                time.sleep(1)
                self.get_logger().info('moving forward')
                
    def rotate(self, angleInRad):
        self.rc.rotate(angleInRad)
        while not self.rc._rotation_complete:
                time.sleep(1)
                self.get_logger().info('rotating')

    def send_marker(self, x, y, marker_id = 0, scale = 0.1, text = ""):
        point_in_map_frame = PointStamped()
        point_in_map_frame.header.frame_id = "/map"
        point_in_map_frame.header.stamp = self.get_clock().now().to_msg()

        point_in_map_frame.point.x = x
        point_in_map_frame.point.y = y
        point_in_map_frame.point.z = 1.0
        
        marker = self.create_marker(point_in_map_frame, marker_id, scale, text)
        self.marker_pub.publish(marker)
            
            
    def create_marker(self, point_stamped, marker_id, scale, text):
        marker = Marker()

        marker.header = point_stamped.header
        
        if text == "":
            marker.type = marker.SPHERE
        else:
            marker.type = marker.TEXT_VIEW_FACING
            
        marker.action = marker.ADD
        marker.id = marker_id
        marker.lifetime = Duration(sec=2)
        marker.text = text

        # Set the scale of the marker
        scale = scale
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 0.1
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = point_stamped.point.x
        marker.pose.position.y = point_stamped.point.y
        marker.pose.position.z = point_stamped.point.z

        return marker
        
        
    def destroyNode(self):
        self.rc._nav_to_pose_client.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    parking = Parking()
    rclpy.spin(parking)
    
    parking.destroyNode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
