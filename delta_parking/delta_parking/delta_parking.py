import rclpy
from rclpy.node import Node

import time
import numpy as np
import math

from delta_interfaces.msg import ParkingJob
from delta_interfaces.msg import JobStatus
from threading import Thread

# for transforming between coordinate frames
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs as tfg

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

# for moving arm
from std_msgs.msg import String as String_msg


class RobotController:

    def __init__(self, node):
    
        self._arrived = False
        self._rotation_complete = False
        self._move_forward_complete = False
        self._task_canceled = False
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
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self._node.get_logger().info('Goal rejected :(')
            self.move_to_position(self._move_x, self._move_y, self._move_rot)
            return

        self._node.get_logger().info('Goal accepted :)')

        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_move_result_callback)
        
    def rotate_goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self._node.get_logger().info('Goal rejected :(')
            self.rotate(self._rotate_rot)
            return

        self._node.get_logger().info('Goal accepted :)')

        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_rotate_result_callback)
        
    def drive_forward_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self._node.get_logger().info('Goal rejected :(')
            self.drive_forward()
            return
            
        self._node.get_logger().info('Goal accepted :)')
        
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_move_forward_callback)
        

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
        
    def get_cancel_task_callback(self, future):
         result = future.result()
         self._task_canceled = True
        
    def cancelTask(self):
        self._task_canceled = False
        self._node.get_logger().info('Canceling current task.')
        if self.result_future:
            self.cancel_result_future = self.goal_handle.cancel_goal_async()
            self.cancel_result_future.add_done_callback(self.get_cancel_task_callback)
        else:
            self._task_canceled = True
     
     


class Parking(Node):

    def __init__(self):
        super().__init__('parking')
        
        # parking job variables
        self.currently_parking = False
        self.parking_goal_x = 0.0
        self.parking_goal_y = 0.0
        self.spotted_ring = False
        self.spotted_ring_x = 0.0
        self.spotted_ring_y = 0.0
        
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
        
        # for transforming between coordinate frames
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # publisher to move the arm
        self.arm_publisher = self.create_publisher(String_msg, '/arm_command', 1)
        
        # testing
        thread = Thread(target=self.park_at_position, args=(-1.1, 1.7, 0.0))
        thread.start()
        
    def get_angle_to_detected_ring(self):
        #if not self.spotted_ring:
        #    return None
            
        robot_map_position = self.get_robot_world_position()
        robot_forward = self.transform_from_robot_to_map_frame_safe(1.0, 0.0)
        vector_robot_forward_x = robot_forward[0] - robot_map_position[0]
        vector_robot_forward_y = robot_forward[1] - robot_map_position[1]
        vector_robot_ring_x = self.spotted_ring_x - robot_map_position[0]
        vector_robot_ring_y = self.spotted_ring_y - robot_map_position[1]
        
        angle = self.angle_between(vector_robot_forward_x, vector_robot_forward_y, vector_robot_ring_x, vector_robot_ring_y)
        
        # finding out if angle is positive or negative
        p1 = self.transform_from_robot_to_map_frame_safe(0.0, 1.0)
        p2 = self.transform_from_robot_to_map_frame_safe(0.0, -1.0)
        
        p1_ring_x = self.spotted_ring_x - p1[0]
        p1_ring_y = self.spotted_ring_y - p1[1]
        p2_ring_x = self.spotted_ring_x - p2[0]
        p2_ring_y = self.spotted_ring_y - p2[1]
        
        dist1_squared = p1_ring_x * p1_ring_x + p1_ring_y * p1_ring_y
        dist2_squared = p2_ring_x * p2_ring_x + p2_ring_y * p2_ring_y
        
        if dist1_squared > dist2_squared:
            angle = -1.0 * angle
        
        return angle


    def publish_status(self):
        msg = JobStatus()
        msg.acting = self.currently_executing_job
        msg.job_id = self.id_of_current_job
        self.publisher_.publish(msg)
        
    def publish_arm_command(self):
        msg = String_msg()
        msg.data = 'manual:[0.,0.3,0.0,2.5]'
        self.arm_publisher.publish(msg)
        
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
        # testing
        while True:
            self.get_logger().info('angle:')
            angle = self.get_angle_to_detected_ring()
            self.get_logger().info(str(angle))
            
        return
        
        time.sleep(1)
        
        self.parking_goal_x = position_x
        self.parking_goal_y = position_y
        self.currently_parking = True
    
        # moving the arm to the correct position
        self.publish_arm_command()
        
        # waiting for transforms to be availaible
        self.is_close_enough_for_parking(position_x, position_y)

        # moving to parking spot
        self.get_logger().info('parking at (x: %f  y: %f)' % (position_x, position_y))
        self.rc.move_to_position(position_x, position_y, 0.0)
        
        while not self.rc._arrived:
                self.publish_arm_command()
                self.get_logger().info('waiting until robot arrives at parking location')
                # Publish a marker
                self.send_marker(position_x, position_y)
                self.send_marker(position_x - 0.1, position_y, 1, 0.15, "parking_nav_goal")
                
                if self.is_close_enough_for_parking(position_x, position_y): # replace by self.spotted_ring when receiving markers works + increase close enough distance
                    self.cancel_task()
                    
                time.sleep(0.2)
                
        # parking user infos
        self.get_logger().info('arrived at parking spot. beginning with parking')        
        self.send_marker(position_x - 0.1, position_y, 1, 0.15, "parking_in_progress")
        
        # just testing the robots movement commands
        #self.rotate(3.14) # rotation: positive value -> anti clock wise. 6.3 = 2 pi = one full turn
        #self.move_forward(.1) # move 0.1 meters
        
        
        self.currently_parking = False
        # IMPORTANT: after greeting has finished, set currently_executing_job to False
        self.currently_executing_job = False
        self.publish_status()
        
        
    def receive_marker():
        x = -1.1
        y = 1.7
        
        if not self.currently_parking:
            return
        if not is_close_enough_for_parking(self.parking_goal_x, self.parking_goal_y):
            return
        
        self.spotted_ring = True
        self.spotted_ring_x = x
        self.spotted_ring_y = y
        
        
    def is_close_enough_for_parking(self, target_x, target_y, close_enough_distance = 0.4):
        x1 = target_x
        y1 = target_y
        
        # getting the robot map position in a way to complicated way
        robot_map_position = self.get_robot_world_position()
        
        x2 = robot_map_position [0]
        y2 = robot_map_position [1]
        
        dx = x1 - x2
        dy = y1 - y2
        dist_squared = dx * dx + dy * dy
        close_enough_distance_squared = close_enough_distance * close_enough_distance
        
        if (dist_squared < close_enough_distance_squared):
            return True
        return False
        
    def transform_from_robot_to_map_frame_safe(self, x, y):
        robot_map_position = None
        while robot_map_position is None:
            robot_map_position = self.transform_from_robot_to_map_frame(x, y, 0.0)
            if robot_map_position is None:
                self.get_logger().info('failed to get robot map position. trying again')
                time.sleep(1)
        return robot_map_position
        
    def get_robot_world_position(self):
        robot_map_position = self.transform_from_robot_to_map_frame_safe(0.0, 0.0)
        return robot_map_position
        
        
    def unit_vector(self, vector):
        return vector / np.linalg.norm(vector)

    def angle_between(self, x1, y1, x2, y2):
        v1 = (x1, y1)
        v2 = (x2, y2)
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
        
        
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
                
    def cancel_task(self):
        self.rc.cancelTask()
        while not self.rc._task_canceled:
                time.sleep(1)
                self.get_logger().info('waiting for task to be canceled')

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
        
    def transform_from_robot_to_map_frame(self, robot_frame_x, robot_frame_y, robot_frame_z):
        
        point_in_robot_frame = PointStamped()
        point_in_robot_frame.header.frame_id = "/base_link"
        point_in_robot_frame.header.stamp = self.get_clock().now().to_msg()
        point_in_robot_frame.point.x = robot_frame_x
        point_in_robot_frame.point.y = robot_frame_y
        point_in_robot_frame.point.z = robot_frame_z
        
        time_now = rclpy.time.Time()
        timeout = rclpy.duration.Duration(seconds=0.1)

        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", time_now, timeout)
            point_in_map_frame = tfg.do_transform_point(point_in_robot_frame, trans)
            map_frame_x = point_in_map_frame.point.x
            map_frame_y = point_in_map_frame.point.y
            map_frame_z = point_in_map_frame.point.z
            return [map_frame_x, map_frame_y, map_frame_z]
        except TransformException as te:
            self.get_logger().info(f"Cound not get the transform: {te}")
            return None
        
        
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
