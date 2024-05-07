import rclpy
from rclpy.node import Node

from delta_interfaces.msg import CylinderObjects

# for receiving marker from detect_people node
from visualization_msgs.msg import Marker

# for transforming between coordinate frames
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs as tfg

# publishing markers
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from builtin_interfaces.msg import Duration


# structure to store the currently known level objects
class CurrentCylinderObjects:
    def __init__(self, position_x, position_y, position_z, color, object_id, number_of_objects):
        self.position_x = position_x
        self.position_y = position_y
        self.position_z = position_z
        self.color = color
        self.object_id = object_id
        self.number_of_objects = number_of_objects

class CylinderObjectIdentifier(Node):

    def __init__(self):
        super().__init__('level_object_identifier')
        # initialize member variables
        self.current_cylinder_objects_ = CurrentCylinderObjects([], [], [], [], [], 0)
        self.cylinderId = 1
        
        # creating the publisher and a timer to publish level objects regulary
        self.publisher_ = self.create_publisher(CylinderObjects, 'cylinder_objects', 1)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_level_objects)
        
        # subscriber to receive the markers that the detect_people.py script that was given to us publishes
        self.marker_subscription = self.create_subscription(Marker, "/detected_cylinder", self.receive_marker, 1)
        self.marker_subscription  # prevent unused variable warning
        
        # for transforming between coordinate frames
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # For publishing the markers
        self.marker_pub = self.create_publisher(Marker, "/delta_cylinder_marker", QoSReliabilityPolicy.RELIABLE)
        

    # gets called every time the subscriber receives a cylinder marker
    def receive_marker(self, msg):
        robot_frame_x = msg.pose.position.x
        robot_frame_y = msg.pose.position.y
        robot_frame_z = msg.pose.position.z
        r = msg.color.r
        g = msg.color.g
        b = msg.color.b
        rgb = str(r) + str(g) + str(b)
        
        # transforming marker position from robot frame to map frame
        map_position = self.transform_from_robot_to_map_frame(robot_frame_x, robot_frame_y, robot_frame_z, msg.header.stamp)
        if map_position is not None:
            #self.process_cylinder_position_in_map_frame(map_position[0], map_position[1] ,map_position[2])
            self.process_cylinder_position_in_map_frame(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, (r,g,b))

    def color_detect(self, rgb):
        r = float(rgb[0])
        g = float(rgb[1])
        b = float(rgb[2])

        if b < 5 and r > 75 and g > 75:
            return "yellow"
        elif max([r,g,b]) == r:
            return "red"
        elif max([r,g,b]) == g:
            return "green"
        elif max([r,g,b]) == b:
            return "blue"
        return ""


    # processes the position of a cylinder that was spottet in the map coordinate frame at cylinder_x, cylinder_y, cylinder_z
    def process_cylinder_position_in_map_frame(self, cylinder_x, cylinder_y ,cylinder_z, colorrgb):
    
        # check if the new cylinder is close to any of the known cylinders
        distance_threshold = 0.75
        distance_threshold_squared = distance_threshold * distance_threshold
        for i in range(self.current_cylinder_objects_.number_of_objects):
            x = self.current_cylinder_objects_.position_x[i]
            y = self.current_cylinder_objects_.position_y[i]
            z = self.current_cylinder_objects_.position_z[i]
            
            dx = x - cylinder_x
            dy = y - cylinder_y
            dz = z - cylinder_z
            
            dist_squared = dx * dx + dy * dy + dz * dz
            
            if (dist_squared < distance_threshold_squared):
                # this cylinder already exists!
                #self.get_logger().info('ALREADY KNOWN cylinder detected at (map_frame): (x: %f  y: %f  z: %f)' % (cylinder_x, cylinder_y, cylinder_z))
                return
                
        color = self.color_detect(colorrgb)
        if color == "":
            return
        

        # cylinder is not close to any of the known cylinders -> it must be a new cylinder!
        self.insert_level_object(cylinder_x, cylinder_y, cylinder_z, color,"cylinder_"+str(self.cylinderId))
        self.cylinderId = self.cylinderId + 1
        self.get_logger().info('FOUND A NEW cylinder detected at (map_frame): (x: %f  y: %f  z: %f) of a color %s' % (cylinder_x, cylinder_y, cylinder_z, color))


    # trys to transfer the given point from robot frame to map frame.
    # Returns the array [map_frame_x, map_frame_y, map_frame_z]. If it can not succeede it returns None
    def transform_from_robot_to_map_frame(self, robot_frame_x, robot_frame_y, robot_frame_z, header_stamp):
        point_in_robot_frame = PointStamped()
        point_in_robot_frame.header.frame_id = "/base_link"
        point_in_robot_frame.header.stamp = header_stamp
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


    # inserts the given level object into the set of known level objects
    def insert_level_object(self, object_position_x, object_position_y, object_position_z, object_color, object_id):
        # getting pointer to current level objects
        old_current_cylinder_objects = self.current_cylinder_objects_
        # creating a new CurrentCylinderObjects object for the member variable 'current_cylinder_objects_'
        self.current_cylinder_objects_ = CurrentCylinderObjects(
            old_current_cylinder_objects.position_x + [object_position_x],
            old_current_cylinder_objects.position_y + [object_position_y],
            old_current_cylinder_objects.position_z + [object_position_z],
            old_current_cylinder_objects.color + [object_color],
            old_current_cylinder_objects.object_id + [object_id],
            old_current_cylinder_objects.number_of_objects + 1
        )
        # publish current level objects whenever something changes about them (plus periodically via timer)
        self.publish_level_objects()


    # is called regulary by a timer to publish all known level objects
    def publish_level_objects(self):
        # getting pointer to current_cylinder_objects before publishing, in case the object is exchanged while building the message
        current_cylinder_objects = self.current_cylinder_objects_
        msg = CylinderObjects() # creating msg of type CylinderObjects inter¸ (see import above: from delta_interfaces.msg import CylinderObjects)
        msg.position_x = current_cylinder_objects.position_x
        msg.position_y = current_cylinder_objects.position_y
        msg.position_z = current_cylinder_objects.position_z
        msg.color = current_cylinder_objects.color
        msg.id = current_cylinder_objects.object_id
        msg.number_of_objects = current_cylinder_objects.number_of_objects
        
        self.publisher_.publish(msg)
        self.publish_level_object_markers()
        #self.get_logger().info('Publishing %d level objects' % msg.number_of_objects)
        
    def publish_level_object_markers(self):
        for i in range(self.current_cylinder_objects_.number_of_objects):
            x = self.current_cylinder_objects_.position_x[i]
            y = self.current_cylinder_objects_.position_y[i]
            object_id = self.current_cylinder_objects_.object_id[i]
            color = self.current_cylinder_objects_.color[i]
            
            self.send_marker(x, y, color, 2 * i + 1000)
            self.send_marker(x - 0.15, y, color, 2 * i + 1000 + 1, 0.15, object_id)
        
        
        
        # send marker when a new level object is discovered
    def send_marker(self, x, y, color, marker_id, scale = 0.1, text = ""):
        point_in_map_frame = PointStamped()
        point_in_map_frame.header.frame_id = "/map"
        point_in_map_frame.header.stamp = self.get_clock().now().to_msg()

        point_in_map_frame.point.x = x
        point_in_map_frame.point.y = y
        point_in_map_frame.point.z = 1.0
        
        marker = self.create_marker(point_in_map_frame, color, marker_id, scale, text)
        self.marker_pub.publish(marker)
            
            
    def create_marker(self, point_stamped, color, marker_id, scale, text):
        marker = Marker()

        marker.header = point_stamped.header
        
        if text == "":
            marker.type = marker.CYLINDER
        else:
            marker.type = marker.TEXT_VIEW_FACING
            
        marker.action = marker.ADD
        marker.id = marker_id
        marker.lifetime = Duration(sec=10000)
        marker.text = text

        # Set the scale of the marker
        scale = scale
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        # Set the color

        r = 0.0
        g = 0.0
        b = 0.0

        if color == "yellow":
            r = 255.0
            g = 255.0
            b = 0.0
        elif color == "red":
            r = 255.0
            g = 0.0
            b = 0.0
        elif color == "green":
            r = 0.0
            g = 255.0
            b = 0.0
        elif color == "blue":
            r = 0.0
            g = 0.0
            b = 255.0

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = point_stamped.point.x
        marker.pose.position.y = point_stamped.point.y
        marker.pose.position.z = point_stamped.point.z

        return marker



def main(args=None):
    rclpy.init(args=args)
    cylinder_object_identifier = CylinderObjectIdentifier()
    rclpy.spin(cylinder_object_identifier)
    
    cylinder_object_identifier.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
