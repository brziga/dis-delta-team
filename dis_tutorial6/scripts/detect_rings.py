#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs as tfg

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from visualization_msgs.msg import Marker

qos_profile = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

PCL_Z_THRESH = 0.25
DIST_EXST_THRESH = 50
WIDTH_DIFF_THRESH = 5

def rgb_to_color_name(rgb):
    r, g, b = rgb
    if r < 30 and g < 30 and b < 30:
        return "Black"
    elif max([r,g,b]) == r:
        return "Red"
    elif max([r,g,b]) == g:
        return "Green"
    elif max([r,g,b]) == b:
        return "Blue"
    
    return "Unknown"

# def point_below_center(center, axes, angle):
#     # Unpack center and axes
#     cx, cy = center
#     a, b = axes

#     # Calculate the non-rotated point directly below the center
#     x = 0
#     y = b

#     # Convert angle to radians
#     theta = np.radians(angle)

#     # Apply rotation
#     x_rotated = x * np.cos(theta) - y * np.sin(theta)
#     y_rotated = x * np.sin(theta) + y * np.cos(theta)

#     # Adjust point based on the center location
#     point_below = [int(cx + x_rotated), int(cy + y_rotated)]

#     return point_below

class RingObject:
    def __init__(self, id, center, ref_point, corners, ellipses, masks, color_num, color_name) -> None:
        self.id = id
        self.center = center # center of the ring
        self.ref_point = ref_point
        self.ellipses = ellipses # the ellipses objects
        self.masks = masks # ring mask, large mask, small (center) mask
        self.color_num = color_num # color of the ring - numeric
        self.color_name = color_name # name of the color of the ring
        self.corners = corners # bounding corners (left top and bottom right)
        self.pcl_coords = None
        self.hollow = None
        self.color_voting = {}


class RingDetector(Node):
    def __init__(self):
        super().__init__('transform_point')

        # Basic ROS stuff
        timer_frequency = 2
        timer_period = 1/timer_frequency

        marker_topic = "ring_marker" #TODO

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # Marker array object used for visualizations
        self.marker_array = MarkerArray()
        self.marker_num = 1

        # Subscribe to the image and/or depth topic
        self.image_sub = self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.image_callback, 1)
        self.depth_sub = self.create_subscription(Image, "/oakd/rgb/preview/depth", self.depth_callback, 1)
        self.pcl_sub = self.create_subscription(PointCloud2, "/oakd/rgb/preview/depth/points", self.pcl_callback, 1)

        self.marker_pub = self.create_publisher(Marker, marker_topic, QoSReliabilityPolicy.BEST_EFFORT)

        self.next_ring_id = 0
        self.rings_candidates = []
        self.rings_detected = []

        # Publiser for the visualization markers
        # self.marker_pub = self.create_publisher(Marker, "/ring", QoSReliabilityPolicy.BEST_EFFORT)

        # Object we use for transforming between coordinate frames
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        cv2.namedWindow("Binary Image", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Detected contours", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Detected rings", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Depth window", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Detected ring mask", cv2.WINDOW_NORMAL)


    def image_callback(self, data): #sig for use with ROS2

        # ROS2 overhead #
        # self.get_logger().info(f"I got a new image! Will try to find rings...")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        ################

        blue = cv_image[:,:,0]
        green = cv_image[:,:,1]
        red = cv_image[:,:,2]

        # Tranform image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # gray = red

        # Apply Gaussian Blur
        # gray = cv2.GaussianBlur(gray,(3,3),0)

        # Do histogram equalization
        # gray = cv2.equalizeHist(gray)

        # Binarize the image, there are different ways to do it
        #ret, thresh = cv2.threshold(img, 50, 255, 0)
        #ret, thresh = cv2.threshold(img, 70, 255, cv2.THRESH_BINARY)
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, 30)
        cv2.imshow("Binary Image", thresh)
        cv2.waitKey(1)

        # Extract contours
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Example of how to draw the contours, only for visualization purposes
        cv2.drawContours(gray, contours, -1, (255, 0, 0), 3)
        cv2.imshow("Detected contours", gray)
        cv2.waitKey(1)

        # Fit elipses to all extracted contours
        elps = []
        for cnt in contours:
            #     print cnt
            #     print cnt.shape
            if cnt.shape[0] >= 11:
                ellipse = cv2.fitEllipse(cnt)
                elps.append(ellipse)


        # Find two elipses with same centers
        candidates = []
        for n in range(len(elps)):
            for m in range(n + 1, len(elps)):
                # e[0] is the center of the ellipse (x,y), e[1] are the lengths of major and minor axis (major, minor), e[2] is the rotation in degrees
                
                e1 = elps[n]
                e2 = elps[m]
                dist = np.sqrt(((e1[0][0] - e2[0][0]) ** 2 + (e1[0][1] - e2[0][1]) ** 2))
                angle_diff = np.abs(e1[2] - e2[2])

                # The centers of the two elipses should be within 5 pixels of each other (is there a better treshold?)
                if dist >= 5:
                    continue

                # The rotation of the elipses should be whitin 4 degrees of eachother
                if angle_diff>4:
                    continue

                e1_minor_axis = e1[1][0]
                e1_major_axis = e1[1][1]

                e2_minor_axis = e2[1][0]
                e2_major_axis = e2[1][1]

                if e1_major_axis>=e2_major_axis and e1_minor_axis>=e2_minor_axis: # the larger ellipse should have both axis larger
                    le = e1 # e1 is larger ellipse
                    se = e2 # e2 is smaller ellipse
                elif e2_major_axis>=e1_major_axis and e2_minor_axis>=e1_minor_axis:
                    le = e2 # e2 is larger ellipse
                    se = e1 # e1 is smaller ellipse
                else:
                    continue # if one ellipse does not contain the other, it is not a ring
                
                # # The widths of the ring along the major and minor axis should be roughly the same
                # border_major = (le[1][1]-se[1][1])/2
                # border_minor = (le[1][0]-se[1][0])/2
                # border_diff = np.abs(border_major - border_minor)

                # if border_diff>WIDTH_DIFF_THRESH:
                #     continue
                    
                candidates.append((e1,e2))

        # print("Processing is done! found", len(candidates), "candidates for rings")

        vis_all_masks = np.zeros((cv_image.shape[0], cv_image.shape[1]))

        # Plot the rings on the image
        for c in candidates:

            # the centers of the ellipses
            e1 = c[0]
            e2 = c[1]

            # drawing the ellipses on the image
            cv2.ellipse(cv_image, e1, (0, 255, 0), 2)
            cv2.ellipse(cv_image, e2, (0, 255, 0), 2)

            # larger and smaller ellipse
            e1_minor_axis = e1[1][0]
            e1_major_axis = e1[1][1]

            e2_minor_axis = e2[1][0]
            e2_major_axis = e2[1][1]

            if e1_major_axis>=e2_major_axis and e1_minor_axis>=e2_minor_axis: # the larger ellipse should have both axis larger
                le = e1 # e1 is larger ellipse
                se = e2 # e2 is smaller ellipse
            elif e2_major_axis>=e1_major_axis and e2_minor_axis>=e1_minor_axis:
                le = e2 # e2 is larger ellipse
                se = e1 # e1 is smaller ellipse
            else:
                le = se = None

            # ellipse masks
            mask_large = np.zeros((cv_image.shape[0], cv_image.shape[1]))
            cv2.ellipse(mask_large, le, 1, -1)
            mask_small = np.zeros((cv_image.shape[0], cv_image.shape[1]))
            cv2.ellipse(mask_small, se, 1, -1)

            mask_ring = cv2.subtract(mask_large, mask_small)

            vis_all_masks += mask_ring


            # Get a bounding box, around the first ellipse ('average' of both elipsis)
            size = (le[1][0]+le[1][1])/2
            center = (le[0][1], le[0][0])

            x1 = int(center[0] - size / 2)
            x2 = int(center[0] + size / 2)
            x_min = x1 if x1>0 else 0
            x_max = x2 if x2<cv_image.shape[0] else cv_image.shape[0]

            y1 = int(center[1] - size / 2)
            y2 = int(center[1] + size / 2)
            y_min = y1 if y1 > 0 else 0
            y_max = y2 if y2 < cv_image.shape[1] else cv_image.shape[1]

            cv2.circle(cv_image, (y_min, x_min), radius=3, color=(255, 0, 0), thickness=-1)
            cv2.circle(cv_image, (y_max, x_max), radius=3, color=(255, 0, 0), thickness=-1)
            cv2.rectangle(cv_image, (y_min, x_min), (y_max, x_max), (0, 0, 250), 2)

            # print(f"\nDetected ring [{self.next_ring_id}] with center {center}")

            # colours
            bool_mask = mask_ring.astype(bool)
            average_color = np.mean(cv_image[bool_mask], axis=0)
            avg_color_name = rgb_to_color_name(average_color)
            # print(f"Ring color: {avg_color_name} {average_color}")
            if(avg_color_name != "Unknown"):
                pass


            label = avg_color_name
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            font_color = (0, 0, 255)  # Red
            thickness = 1
            line_type = cv2.LINE_AA
            text_x = x_min + 25
            text_y = y_min - 100
            cv2.putText(cv_image, label, (text_y, text_x), font, font_scale, font_color, thickness, line_type)

            # refp_axes = [
            #     (le[1][0] + se[1][0]) / 2,
            #     (le[1][1] + se[1][1]) / 2
            # ]
            # ref_point = point_below_center(center, refp_axes, se[2])
            # ref_point[0] = ref_point[0] if ref_point[0] > 0 else 0
            # ref_point[1] = ref_point[1] if ref_point[1] > 0 else 0
            # ref_point[0] = ref_point[0] if ref_point[0] < cv_image.shape[0]-1 else cv_image.shape[0]
            # ref_point[1] = ref_point[1] if ref_point[1] < cv_image.shape[1]-1 else cv_image.shape[1]
            # print(f"Ref point: {ref_point}")
            # cv2.circle(cv_image, (ref_point[1], ref_point[0]), radius=3, color=(255, 0, 255), thickness=-1)

            rows = np.where(mask_ring[int(center[0])::, int(center[1])] != 0)
            ref_point = [np.median(rows), int(center[0])]

            self.rings_candidates.append(
                RingObject(
                    self.next_ring_id, #id
                    center, #center
                    ref_point,
                    ((y_min, x_min), (y_max, x_max)), #corners
                    c, #ellipses
                    (mask_ring, mask_large, mask_small), #masks
                    average_color,
                    avg_color_name
                )
            )
            self.next_ring_id += 1


        if len(candidates)>0:
                cv2.imshow("Detected rings",cv_image)
                cv2.imshow("Detected ring mask", vis_all_masks)
                cv2.waitKey(1)


    def depth_callback(self,data):

        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

        depth_image[depth_image==np.inf] = 0
        
        # Do the necessairy conversion so we can visuzalize it in OpenCV
        image_1 = depth_image / 65536.0 * 255
        image_1 = image_1/np.max(image_1)*255

        #image_viz = np.array(image_1, dtype= np.uint8)

        #cv2.imshow("Depth window", image_viz)
        #cv2.waitKey(1)
        

        for ring_candidate in self.rings_candidates:

            contains_inf = np.any(depth_image[ring_candidate.masks[2].astype(bool)]) # mask with inner mask of the ring and check if there is an infinite depth pixel
            ring_candidate.hollow = contains_inf

            # print(f"Ring {ring_candidate.id} [{ring_candidate.center}] hollow is {contains_inf}")



    '''def pcl_callback(self,data):

        height = data.height
        width = data.width
        point_step = data.point_step
        row_step = data.row_step

        for ring_candidate in self.rings_candidates:

            # print(ring_candidate.center)
            #x, y = ring_candidate.ref_point
            #y, x = int(y), int(x)
            # print(y)
            # print(x)

            x_min, y_min = ring_candidate.corners[0]
            x_max, y_max = ring_candidate.corners[1]

            x_min = max(x_min, 0)
            y_min = max(y_min, 0)
            x_max = min(x_max, width - 1)
            y_max = min(y_max, height - 1)
            

            pcl =  pc2.read_points_numpy(data, field_names= ("x", "y", "z"))
            # print(pcl)
            pcl = pcl.reshape((height,width,3))
            # print(pcl)
    
            region_pcl = pcl[y_min:y_max + 1, x_min:x_max + 1, :]

            valid_points = region_pcl[(region_pcl[:, :, 2] != np.inf) & (region_pcl[:, :, 2] != 0)]
            if valid_points.size == 0:
                continue

            closest_point = valid_points[np.argmin(valid_points[:, 2])]

            ring_candidate.pcl_coords = closest_point

            #y = min(y, pcl.shape[0]-1)
            #y = min(x, pcl.shape[1]-1)

            #pcl_center = pcl[y,x,:]
            # print(pcl_center)

            #ring_candidate.pcl_coords = pcl_center

            # print(f"PCL of ring {ring_candidate.id}: {pcl_center}")

            if closest_point[2] > PCL_Z_THRESH and ring_candidate.hollow:
                for det_ring in self.rings_detected:
                    distance = np.linalg.norm(
                        ring_candidate.pcl_coords 
                        - 
                        det_ring.pcl_coords
                    )

                    print(f"Distance between {ring_candidate.pcl_coords} and {det_ring.pcl_coords} is {distance}")

                    if not np.isinf(distance) and not np.isnan(distance) and distance > DIST_EXST_THRESH:
                        self.rings_detected.append(ring_candidate)
                        print(f"Confirmed ring {ring_candidate.id} with: \n\t center: {ring_candidate.center} \n\t color: {ring_candidate.color_name} {ring_candidate.color_num} \n\t hollow: {ring_candidate.hollow} \n\t pcl coords: {ring_candidate.pcl_coords}")
                    else:
                        if ring_candidate.color_name == "Unknown": continue
                        elif ring_candidate.color_name not in det_ring.color_voting.keys():
                            det_ring.color_voting[ring_candidate.color_name] = 1
                        else:
                            det_ring.color_voting[ring_candidate.color_name] += 1
                    
                if len(self.rings_detected) == 0:
                    if ring_candidate.color_name != "Unknown":
                        ring_candidate.color_voting[ring_candidate.color_name] = 1
                    self.rings_detected.append(ring_candidate)
                    print(f"Confirmed ring {ring_candidate.id} with: \n\t center: {ring_candidate.center} \n\t color: {ring_candidate.color_name} {ring_candidate.color_num} \n\t hollow: {ring_candidate.hollow} \n\t pcl coords: {ring_candidate.pcl_coords}")


            # # create marker
            # marker = Marker()

            # marker.header.frame_id = "/base_link" #TODO
            # marker.header.stamp = data.header.stamp

            # marker.type = 2
            # marker.id = 0

            # # Set the scale of the marker
            # scale = 0.1
            # marker.scale.x = scale
            # marker.scale.y = scale
            # marker.scale.z = scale

            # # Set the color
            # marker.color.r = 0.0
            # marker.color.g = 0.0
            # marker.color.b = 1.0
            # marker.color.a = 1.0

            # # Set the pose of the marker
            # marker.pose.position.x = float(pcl_center[0])
            # marker.pose.position.y = float(pcl_center[1])
            # marker.pose.position.z = float(pcl_center[2])

            # self.marker_pub.publish(marker)

        self.rings_candidates = []'''
    
    def pcl_callback(self, data):

        height = data.height
        width = data.width
        point_step = data.point_step
        row_step = data.row_step

        for ring_candidate in self.rings_candidates:
            x_min, y_min = ring_candidate.corners[0]
            x_max, y_max = ring_candidate.corners[1]

            x, y = ring_candidate.center
            y, x = int(y), int(x)

            pcl = pc2.read_points_numpy(data, field_names=("x", "y", "z"))
            pcl = pcl.reshape((height, width, 3))

            y = min(y, pcl.shape[0] - 1)
            x = min(x, pcl.shape[1] - 1)

            pcl_center = pcl[y, x, :]

            if np.isinf(pcl_center).any():
                ring_candidate.hollow = True
            else: continue

            x_min = int(x_min + 0.1 * x_min)
            y_min = int(y_min + 0.1 * y_min)
            x_max = int(x_max + 0.1 * x_max)
            y_max = int(y_max + 0.1 * y_max)

            # Ensure the bounding box is within the point cloud dimensions
            x_min = max(x_min, 0)
            y_min = max(y_min, 0)
            x_max = min(x_max, width - 1)
            y_max = min(y_max, height - 1)

            # Extract the point cloud data within the bounding box
            region_pcl = pcl[y_min:y_max + 1, x_min:x_max + 1, :]

            # Filter out points with z as inf or 0
            valid_points = region_pcl[(region_pcl[:, :, 2] != np.inf) & (region_pcl[:, :, 2] != 0)]
            
            if valid_points.size == 0:
                continue  # No valid points, skip this candidate

            # Find the point with the minimum Z value
            closest_point = valid_points[np.argmin(valid_points[:, 2])]

            ring_candidate.pcl_coords = closest_point

            newring = False

            if closest_point[2] > PCL_Z_THRESH and ring_candidate.hollow:
                for det_ring in self.rings_detected:
                    distance = np.linalg.norm(
                        ring_candidate.pcl_coords 
                        - 
                        det_ring.pcl_coords
                    )

                    print(f"Distance between {ring_candidate.pcl_coords} and {det_ring.pcl_coords} is {distance}")

                    if not np.isinf(distance) and not np.isnan(distance) and distance > DIST_EXST_THRESH:
                        self.rings_detected.append(ring_candidate)
                        newring = True
                        print(f"Confirmed ring {ring_candidate.id} with: \n\t center: {ring_candidate.center} \n\t color: {ring_candidate.color_name} {ring_candidate.color_num} \n\t hollow: {ring_candidate.hollow} \n\t pcl coords: {ring_candidate.pcl_coords}")
                    else:
                        if ring_candidate.color_name == "Unknown": continue
                        elif ring_candidate.color_name not in det_ring.color_voting.keys():
                            det_ring.color_voting[ring_candidate.color_name] = 1
                        else:
                            det_ring.color_voting[ring_candidate.color_name] += 1
                    
                if len(self.rings_detected) == 0:
                    if ring_candidate.color_name != "Unknown":
                        ring_candidate.color_voting[ring_candidate.color_name] = 1
                    self.rings_detected.append(ring_candidate)
                    newring = True
                    print(f"Confirmed ring {ring_candidate.id} with: \n\t center: {ring_candidate.center} \n\t color: {ring_candidate.color_name} {ring_candidate.color_num} \n\t hollow: {ring_candidate.hollow} \n\t pcl coords: {ring_candidate.pcl_coords}")


                if newring:
                    print("putting ring on map", ring_candidate.color_num)
                    time_now = rclpy.time.Time()
                    timeout = rclpy.duration.Duration(seconds=0.1)

                    point_on_ring = PointStamped()
                    point_on_ring.header.frame_id = "/base_link"
                    point_on_ring.header.stamp = time_now.to_msg()
                    point_on_ring.point.x = float(closest_point[0])
                    point_on_ring.point.y = float(closest_point[1])
                    point_on_ring.point.z = float(closest_point[2])

                    try:
                        trans = self.tf_buffer.lookup_transform("map", "base_link", time_now, timeout)
                        point_in_map_frame = tfg.do_transform_point(point_on_ring, trans)
                        map_frame_x = point_in_map_frame.point.x
                        map_frame_y = point_in_map_frame.point.y
                        map_frame_z = point_in_map_frame.point.z
                    except TransformException as te:
                        self.get_logger().info(f"Cound not get the transform: {te}")
                        return
                    
                    # Create marker for visualization
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.type = Marker.SPHERE
                    marker.id = ring_candidate.id

                    # Set the scale of the marker
                    marker.scale.x = 0.1
                    marker.scale.y = 0.1
                    marker.scale.z = 0.1

                    # Set the color
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                    marker.color.a = 1.0

                    # Set the pose of the marker
                    marker.pose.position.x = float(map_frame_x)
                    marker.pose.position.y = float(map_frame_y)
                    marker.pose.position.z = float(map_frame_z)

                    self.marker_pub.publish(marker)

        self.rings_candidates = []






def main():

    rclpy.init(args=None)
    rd_node = RingDetector()

    rclpy.spin(rd_node)

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()