import rclpy
from rclpy.node import Node

from delta_interfaces.msg import LevelObjects

# for receiving marker from detect_people node
from visualization_msgs.msg import Marker

# for transforming between coordinate frames
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs as tfg

# structure to store the currently known level objects
class CurrentLevelObjects:
    def __init__(self, position_x, position_y, position_z, rotation, object_id, number_of_objects):
        self.position_x = position_x
        self.position_y = position_y
        self.position_z = position_z
        self.rotation = rotation
        self.object_id = object_id
        self.number_of_objects = number_of_objects

class LevelObjectIdentifier(Node):

    def __init__(self):
        super().__init__('level_object_identifier')
        # initialize member variables
        self.current_level_objects_ = CurrentLevelObjects([], [], [], [], [], 0)
        self.personId = 1
        
        # creating the publisher and a timer to publish level objects regulary
        self.publisher_ = self.create_publisher(LevelObjects, 'level_objects', 1)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_level_objects)
        
        # subscriber to receive the markers that the detect_people.py script that was given to us publishes
        self.marker_subscription = self.create_subscription(Marker, "/people_marker", self.receive_marker, 1)
        self.marker_subscription  # prevent unused variable warning
        
        # for transforming between coordinate frames
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        #testing
        #self.insert_level_object(.1, .2, .3, 0.0,"person_1")
        

    # gets called every time the subscriber receives a face marker
    def receive_marker(self, msg):
        robot_frame_x = msg.pose.position.x
        robot_frame_y = msg.pose.position.y
        robot_frame_z = msg.pose.position.z
        
        # transforming marker position from robot frame to map frame
        map_position = self.transform_from_robot_to_map_frame(robot_frame_x, robot_frame_y, robot_frame_z)
        if map_position is not None:
            self.process_face_position_in_map_frame(map_position[0], map_position[1] ,map_position[2])


    # processes the position of a face that was spottet in the map coordinate frame at face_x, face_y, face_z
    def process_face_position_in_map_frame(self, face_x, face_y ,face_z):
    
        # check if the new face is close to any of the known faces
        distance_threshold = 0.5
        distance_threshold_squared = distance_threshold * distance_threshold
        for i in range(self.current_level_objects_.number_of_objects):
            x = self.current_level_objects_.position_x[i]
            y = self.current_level_objects_.position_y[i]
            z = self.current_level_objects_.position_z[i]
            
            dx = x - face_x
            dy = y - face_y
            dz = z - face_z
            
            dist_squared = dx * dx + dy * dy + dz * dz
            
            if (dist_squared < distance_threshold_squared):
                # this face already exists!
                self.get_logger().info('ALREADY KNOWN person detected at (map_frame): (x: %f  y: %f  z: %f)' % (face_x, face_y, face_z))
                return
                
        # face is not close to any of the known faces -> it must be a new face!
        self.insert_level_object(face_x, face_y, face_z, 0.0,"person_"+str(self.personId))
        self.personId = self.personId + 1
        self.get_logger().info('FOUND A NEW person detected at (map_frame): (x: %f  y: %f  z: %f)' % (face_x, face_y, face_z))
        
        # TODO: - This node is currenty using the marker messages from the detect_people script of dis_tutorial3. I guess it would be
        #         better to implement an other msg-topic instead of using the marker-msg. Maybe migrate the detect_people script into our project?
        #       - Best might be not to set the face position itself as the object position, but a position about 1 meter before the face and
        #         with a rotation so that the robot faces the person when moving to the x,y,z,rot of the stored object


    # trys to transfer the given point from robot frame to map frame.
    # Returns the array [map_frame_x, map_frame_y, map_frame_z]. If it can not succeede it returns None
    def transform_from_robot_to_map_frame(self, robot_frame_x, robot_frame_y, robot_frame_z):
        point_in_robot_frame = PointStamped()
        point_in_robot_frame.header.frame_id = "/base_link"
        point_in_robot_frame.header.stamp = self.get_clock().now().to_msg()
        point_in_robot_frame.point.x = robot_frame_x
        point_in_robot_frame.point.y = robot_frame_y
        point_in_robot_frame.point.z = robot_frame_z
        
        time_now = rclpy.time.Time()
        timeout = Duration(seconds=0.1)

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


    # inserts the given level object into the set of known level objects
    def insert_level_object(self, object_position_x, object_position_y, object_position_z, object_rotation, object_id):
        # getting pointer to current level objects
        old_current_level_objects = self.current_level_objects_
        # creating a new CurrentLevelObjects object for the member variable 'current_level_objects_'
        self.current_level_objects_ = CurrentLevelObjects(
            old_current_level_objects.position_x + [object_position_x],
            old_current_level_objects.position_y + [object_position_y],
            old_current_level_objects.position_z + [object_position_z],
            old_current_level_objects.rotation + [object_rotation],
            old_current_level_objects.object_id + [object_id],
            old_current_level_objects.number_of_objects + 1
        )
        # publish current level objects whenever something changes about them (plus periodically via timer)
        self.publish_level_objects()


    # is called regulary by a timer to publish all known level objects
    def publish_level_objects(self):
        # getting pointer to current_level_objects before publishing, in case the object is exchanged while building the message
        current_level_objects = self.current_level_objects_
        msg = LevelObjects() # creating msg of type LevelObjects interface (see import above: from delta_interfaces.msg import LevelObjects)
        msg.position_x = current_level_objects.position_x
        msg.position_y = current_level_objects.position_y
        msg.position_z = current_level_objects.position_z
        msg.rotation = current_level_objects.rotation
        msg.id = current_level_objects.object_id
        msg.number_of_objects = current_level_objects.number_of_objects
        
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing %d level objects' % msg.number_of_objects)



def main(args=None):
    rclpy.init(args=args)
    level_object_identifier = LevelObjectIdentifier()
    rclpy.spin(level_object_identifier)
    
    level_object_identifier.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
