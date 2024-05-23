#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from visualization_msgs.msg import Marker

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from ultralytics import YOLO

# from rclpy.parameter import Parameter
# from rcl_interfaces.msg import SetParametersResult

class detect_faces(Node):

	def __init__(self):
		super().__init__('detect_faces')

		self.declare_parameters(
			namespace='',
			parameters=[
				('device', ''),
		])

		marker_topic = "/people_marker"

		self.detection_color = (0,0,255)
		self.device = self.get_parameter('device').get_parameter_value().string_value

		self.bridge = CvBridge()
		self.scan = None

		self.rgb_image_sub = self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.rgb_callback, qos_profile_sensor_data)
		#self.pointcloud_sub = self.create_subscription(PointCloud2, "/oakd/rgb/preview/depth/points", self.pointcloud_callback, qos_profile_sensor_data)

		self.marker_pub = self.create_publisher(Marker, marker_topic, QoSReliabilityPolicy.BEST_EFFORT)

		self.model = YOLO("yolov8n.pt")

		self.faces = []

		self.get_logger().info(f"Node has been initialized! Will publish face markers to {marker_topic}.")
		
	# returns the height (in the world space) of the camera view at the distance of the face
	def get_camera_height_in_meters(self, face_height_in_meters, camera_height_in_pixels, face_height_in_pixels):
		return float(face_height_in_meters) * float(camera_height_in_pixels) / float(face_height_in_pixels)
		
	# returns the distance to the spotted face
	def get_distance_in_meters(self, camera_height_in_meters, camera_opening_angle_in_rad_UP_DOWN):
		return (float(camera_height_in_meters) / 2.0) / np.tan(float(camera_opening_angle_in_rad_UP_DOWN) / 2.0)
		
	# NOT IN USE, because it is very unprecise: when looking at image from the side, the image width can vary greatly :(
	def get_camera_width_in_meters_via_fixed_face_width(self, face_width_in_meters, camera_width_in_pixels, face_width_in_pixels):
		return float(face_width_in_meters) * float(camera_width_in_pixels) / float(face_width_in_pixels)
	
	# returns the width (in the world space) of the camera view for a given distance to the face
	def get_camera_width_in_meters(self, camera_opening_angle_in_rad_LEFT_RIGHT, distance_in_meters):
		return np.tan(float(camera_opening_angle_in_rad_LEFT_RIGHT / 2)) * distance_in_meters * 2.0
	
	# returns the side offset the face has to the camera center in meters
	def get_side_offset_in_meters(self, side_offset_in_pixel, camera_width_in_meters, camera_width_in_pixels):
		return float(side_offset_in_pixel) * float(camera_width_in_meters) / float(camera_width_in_pixels)
	
	# returns the height offset the face has to the camera center in meters
	def get_height_offset_in_meters(self, height_offset_in_pixel, camera_height_in_meters, camera_height_in_pixels):
		return float(height_offset_in_pixel) * float(camera_height_in_meters) / float(camera_height_in_pixels)
		
	def deg_to_rad(self, deg):
		return (deg / 360.0) * (2 * np.pi)

	def rgb_callback(self, data):

		self.faces = []

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

			self.get_logger().info(f"Running inference on image...")

			# run inference
			res = self.model.predict(cv_image, imgsz=(256, 320), show=False, verbose=False, classes=[0], device=self.device)

			# iterate over results
			for x in res:
				bbox = x.boxes.xyxy
				if bbox.nelement() == 0: # skip if empty
					continue


				bbox = bbox[0]

				cx = int((bbox[0]+bbox[2])/2)
				cy = int((bbox[1]+bbox[3])/2)

				if (cy < 125.0):
					continue


				self.faces.append((cx,cy))
				
				
				# --- estimating distance ---
				
				# tuneable parameters
				face_height_in_meters = 0.21 # meters
				camera_opening_angle_in_deg_UP_DOWN = 40.0 # degree (NOT rad). It is the UP/DOWN angle (NOT the left/right angle)
				camera_opening_angle_in_deg_LEFT_RIGHT = 40.0
				
				# fixed parameters
				camera_height_in_pixels = 250.0 # how many pixels HIGH the camera image is (y coordinate of image)
				camera_width_in_pixels = 250.0 # how many pixels WIDTH the camera image is (x coordinate of image)
				
				# distance calculation
				camera_opening_angle_in_rad_UP_DOWN = self.deg_to_rad(camera_opening_angle_in_deg_UP_DOWN) # NOW its in rad ;)
				face_height_in_pixels = int(bbox[3] - bbox[1])
				camera_height_in_meters =  self.get_camera_height_in_meters(face_height_in_meters, camera_height_in_pixels, face_height_in_pixels)
				distance_in_meters = self.get_distance_in_meters(camera_height_in_meters, camera_opening_angle_in_rad_UP_DOWN)
				
				# side offset calculation
				camera_opening_angle_in_rad_LEFT_RIGHT = self.deg_to_rad(camera_opening_angle_in_deg_LEFT_RIGHT)
				side_offset_in_pixel = (camera_width_in_pixels / 2.0) - cx
				camera_width_in_meters = self.get_camera_width_in_meters(camera_opening_angle_in_rad_LEFT_RIGHT, distance_in_meters)
				side_offset_in_meters = self.get_side_offset_in_meters(side_offset_in_pixel, camera_width_in_meters, camera_width_in_pixels)
				
				# height offset calculation
				height_offset_in_pixel = (camera_height_in_pixels / 2.0) - cy
				height_offset_in_meters = self.get_height_offset_in_meters(height_offset_in_pixel, camera_height_in_meters, camera_height_in_pixels)

				if (distance_in_meters > 2.0):
					continue
				
				self.get_logger().info(f"Person has been detected!")

				# draw rectangle
				cv_image = cv2.rectangle(cv_image, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), self.detection_color, 3)

				# draw the center of bounding box
				cv_image = cv2.circle(cv_image, (cx,cy), 5, self.detection_color, -1)

				# create marker
				marker = Marker()

				marker.header.frame_id = "/base_link"
				marker.header.stamp = data.header.stamp

				marker.type = 2
				marker.id = 0

				# Set the scale of the marker
				scale = 0.1
				marker.scale.x = scale
				marker.scale.y = scale
				marker.scale.z = scale

				# Set the color
				marker.color.r = 1.0
				marker.color.g = 1.0
				marker.color.b = 1.0
				marker.color.a = 1.0

				# Set the pose of the marker
				marker.pose.position.x = distance_in_meters # forward /backward
				marker.pose.position.y = side_offset_in_meters # left / right
				marker.pose.position.z = height_offset_in_meters # up / down

				self.marker_pub.publish(marker)	
				

			cv2.imshow("image", cv_image)
			
			key = cv2.waitKey(1)
			if key==27:
				print("exiting")
				exit()
			
		except CvBridgeError as e:
			print(e)

	'''
	def pointcloud_callback(self, data):
	
		self.get_logger().info(f"pointcloud_callback")

		# get point cloud attributes
		height = data.height
		width = data.width
		point_step = data.point_step
		row_step = data.row_step		

		# iterate over face coordinates
		for x,y in self.faces:

			# get 3-channel representation of the poitn cloud in numpy format
			a = pc2.read_points_numpy(data, field_names= ("x", "y", "z"))
			a = a.reshape((height,width,3))

			# read center coordinates
			d = a[y,x,:]

			# create marker
			marker = Marker()

			marker.header.frame_id = "/base_link"
			marker.header.stamp = data.header.stamp

			marker.type = 2
			marker.id = 0

			# Set the scale of the marker
			scale = 0.1
			marker.scale.x = scale
			marker.scale.y = scale
			marker.scale.z = scale

			# Set the color
			marker.color.r = 1.0
			marker.color.g = 1.0
			marker.color.b = 1.0
			marker.color.a = 1.0

			# Set the pose of the marker
			marker.pose.position.x = float(d[0])
			marker.pose.position.y = float(d[1])
			marker.pose.position.z = float(d[2])

			self.marker_pub.publish(marker)
	'''

def main():
	print('Face detection node starting.')

	rclpy.init(args=None)
	node = detect_faces()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
