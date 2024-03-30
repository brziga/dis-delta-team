import rclpy
from rclpy.node import Node

from delta_interfaces.msg import ExplorerJob
from delta_interfaces.msg import JobStatus
from threading import Thread

import time



# robot commander ----------------------------------------------------------------------------------
from enum import Enum
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Quaternion, PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import Spin, NavigateToPose
from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler

from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as rclpyDuration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3

amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

class RobotCommander(Node):

    def __init__(self, node_name='robot_commander', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)
        
        self.pose_frame_id = 'map'
        
        # Flags and helper variables
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.initial_pose_received = False
        self.is_docked = None

        # ROS2 subscribers
        self.create_subscription(DockStatus,
                                 'dock_status',
                                 self._dockCallback,
                                 qos_profile_sensor_data)
        
        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                              'amcl_pose',
                                                              self._amclPoseCallback,
                                                              amcl_pose_qos)
        
        # ROS2 publishers
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)
        
        # ROS2 Action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.undock_action_client = ActionClient(self, Undock, 'undock')
        self.dock_action_client = ActionClient(self, Dock, 'dock')

        self.get_logger().info(f"Robot commander has been initialized!")
        
    def destroyNode(self):
        self.nav_to_pose_client.destroy()
        super().destroy_node()     

    def goToPose(self, pose, behavior_tree=''):
        """Send a `NavToPose` action request."""
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                       str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def spin(self, spin_dist=1.57, time_allowance=10):
        self.debug("Waiting for 'Spin' action server")
        while not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.info("'Spin' action server not available, waiting...")
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = spin_dist
        goal_msg.time_allowance = Duration(sec=time_allowance)

        self.info(f'Spinning to angle {goal_msg.target_yaw}....')
        send_goal_future = self.spin_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Spin request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def undock(self):
        """Perform Undock action."""
        self.info('Undocking...')
        self.undock_send_goal()

        while not self.isUndockComplete():
            time.sleep(0.1)

    def undock_send_goal(self):
        goal_msg = Undock.Goal()
        self.undock_action_client.wait_for_server()
        goal_future = self.undock_action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, goal_future)

        self.undock_goal_handle = goal_future.result()

        if not self.undock_goal_handle.accepted:
            self.error('Undock goal rejected')
            return

        self.undock_result_future = self.undock_goal_handle.get_result_async()

    def isUndockComplete(self):
        """
        Get status of Undock action.

        :return: ``True`` if undocked, ``False`` otherwise.
        """
        if self.undock_result_future is None or not self.undock_result_future:
            return True

        rclpy.spin_until_future_complete(self, self.undock_result_future, timeout_sec=0.1)

        if self.undock_result_future.result():
            self.undock_status = self.undock_result_future.result().status
            if self.undock_status != GoalStatus.STATUS_SUCCEEDED:
                self.info(f'Goal with failed with status code: {self.status}')
                return True
        else:
            return False

        self.info('Undock succeeded')
        return True

    def cancelTask(self):
        """Cancel pending task request of any type."""
        self.info('Canceling current task.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isTaskComplete(self):
        """Check if the task request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug(f'Task with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug('Task succeeded!')
        return True

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def waitUntilNav2Active(self, navigator='bt_navigator', localizer='amcl'):
        """Block until the full navigation system is up and running."""
        self._waitForNodeToActivate(localizer)
        if not self.initial_pose_received:
            time.sleep(1)
        self._waitForNodeToActivate(navigator)
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            self.debug(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug(f'Result of get_state: {state}')
            time.sleep(2)
        return
    
    def YawToQuaternion(self, angle_z = 0.):
        quat_tf = quaternion_from_euler(0, 0, angle_z)

        # Convert a list to geometry_msgs.msg.Quaternion
        quat_msg = Quaternion(x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3])
        return quat_msg

    def _amclPoseCallback(self, msg):
        self.debug('Received amcl pose')
        self.initial_pose_received = True
        self.current_pose = msg.pose
        return

    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return
    
    def _dockCallback(self, msg: DockStatus):
        self.is_docked = msg.is_docked

    def setInitialPose(self, pose):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = pose
        msg.header.frame_id = self.pose_frame_id
        msg.header.stamp = 0
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return
        
# ---------------------------------------------------------------------------------------------------



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

        


class Explorer(Node):

    def __init__(self):
        super().__init__('explorer')
        self.currently_executing_job = False
        self.id_of_current_job = ""
        self.currently_exploring = False
        
        self.keepExploring = False
        self.stoppedExploring = True
        
        # point: x: +1 means one grid zell up
        #           -1 means one grid zell down
        #        y: +1 means one grid zell left
        #           -1 means one grid zell right
        #        startRotation: 0 = 2pi = up; pi/2 = - 3/2 pi = left; pi = -pi = down; 3/2 pi = - pi/2 = right
        up = 0
        leftUp = 0.785
        left = 1.57
        leftDown = 2.35
        down = 3.14
        downRight = -2.35
        right = -1.57
        rightUp = -0.785
        # rotation: positive value -> anti clock wise. 6.3 = 2 pi = one full turn
        ac_half = 3.14
        ac_quater = 1.57
        c_half = -3.14
        c_quater = -1.57
        # each exploration point contains one point [x, y, rotation] where the robot will go to and
        # one rotation, that will be applied after the point is reached. rotation = 'None' means apply no rotation after point is reached.
        # [x,y,startRotation], rotationToApply
        # USE FLOATS!! 0.0 INSTEAD OF 0 FOR EXAMPLE
        self.explorationPoints = [[[-0.3, 0.0, left], c_half], [[-0.3, -1.0, leftUp], c_quater], [[-0.3, -1.5, rightUp], c_half], [[-1.5, -0.5, rightUp], c_half+c_quater]]
        
        self.explorationPointIndex = 0
        
        # publishing the jobs status
        self.publisher_ = self.create_publisher(JobStatus, 'job_status', 1)
        timer_period = 1.0  # seconds
        self.publish_status_timer = self.create_timer(timer_period, self.publish_status)
        
        # listen to incoming jobs
        self.subscription = self.create_subscription(ExplorerJob, 'explorer_job', self.process_incoming_job, 1)
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
            if msg.explore:
                self.publish_status()
                thread = Thread(target = self.continue_exploring)
                thread.start()
            else:
                self.publish_status()
                thread = Thread(target=self.stop_exploring)
                thread.start()
    
    def continue_exploring(self):
        # already exploring, job done
        if self.currently_exploring:
            self.currently_executing_job = False
            self.publish_status()
            return
        
        # just to simulate delay
        time.sleep(1)
        
        # TODO: start exploring!
        self.keepExploring = True
        self.stoppedExploring = False
        thread = Thread(target = self.exploreNextPoint)
        thread.start()
        self.get_logger().info('starting to explore! Finished job with id: '+self.id_of_current_job)
        
        # IMPORTANT: when job is complete do:
        self.currently_exploring= True
        self.currently_executing_job = False
        self.publish_status()
        
    def stop_exploring(self):
        # not exploring anyway, job done
        if not self.currently_exploring:
            self.currently_executing_job = False
            self.publish_status()
            return
        
        #just to simulate delay
        time.sleep(1)
        
        # TODO: stop exploring!
        self.keepExploring = False
        while not self.stoppedExploring:
            self.get_logger().info('waiting for exploration to stop')
            time.sleep(0.5)
        self.get_logger().info('stopped exploring! Finished job with id: '+self.id_of_current_job)
        
        # IMPORTANT: when job is complete do:
        self.currently_exploring= False
        self.currently_executing_job = False
        self.publish_status()
        
    def exploreNextPoint(self):
        self.get_logger().info('exploring next point')
        if self.keepExploring:
            # move to next position
            nextPoint = self.explorationPoints[self.explorationPointIndex][0]
            self.get_logger().info('moving to point (x: %f  y: %f  rot: %f)' % (nextPoint[0], nextPoint[1], nextPoint[2]))
            self.rc.move_to_position(nextPoint[0], nextPoint[1], nextPoint[2])
            while not self.rc._arrived:
                time.sleep(1)
                self.get_logger().info('waiting until robot arrives at goal position')
            
            # rotate at that point
            nextRotation = self.explorationPoints[self.explorationPointIndex][1]
            if nextRotation is not None:
                self.get_logger().info('rotating by %f rad' % nextRotation)
                self.rc.rotate(nextRotation)
                while not self.rc._rotation_complete:
                    time.sleep(1)
                    self.get_logger().info('waiting until robot rotated')
            
            # check here if movement goal and rotation goal was successfull before incrementing?
            self.increment_exploration_point_index()
            
            #time.sleep(3) # just to simulate delay
            #TODO: move to position like this
            #moveToPosition(x,y,rotation)
            
            
            # moved to point. moving to next point
            thread = Thread(target = self.exploreNextPoint)
            thread.start()
        else:
            self.stoppedExploring = True
            return
            
    def increment_exploration_point_index(self):
        self.explorationPointIndex = self.explorationPointIndex + 1
        if self.explorationPointIndex >= len(self.explorationPoints):
            self.explorationPointIndex = 0
            
    def destroyNode(self):
        self.rc._nav_to_pose_client.destroy()
        super().destroy_node()
            


def main(args=None):
    rclpy.init(args=args)
    
    # robot commander: only for undocking from station --------------------------
    rc = RobotCommander()
    # Wait until Nav2 and Localizer are available
    rc.waitUntilNav2Active()
    # Check if the robot is docked, only continue when a message is recieved
    while rc.is_docked is None:
        rclpy.spin_once(rc, timeout_sec=0.5)
    # If it is docked, undock it first and turn away from docking station
    if rc.is_docked:
        rc.undock()
    rc.destroyNode()
    # ---------------------------------------------------------------------------
        
    
    explorer = Explorer()
    rclpy.spin(explorer)
    
    explorer.destroyNode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
