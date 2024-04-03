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
		self.pointcloud_sub = self.create_subscription(PointCloud2, "/oakd/rgb/preview/depth/points", self.pointcloud_callback, qos_profile_sensor_data)

		self.marker_pub = self.create_publisher(Marker, marker_topic, QoSReliabilityPolicy.BEST_EFFORT)

		self.model = YOLO("yolov8n.pt")

		self.faces = []
		
		self.bounding_box = []

		self.get_logger().info(f"Node has been initialized! Will publish face markers to {marker_topic}.")

	def rgb_callback(self, data):

		self.faces = []
		
		self.bounding_box = []

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

				self.get_logger().info(f"Person has been detected!")

				bbox = bbox[0]

				# draw rectangle
				cv_image = cv2.rectangle(cv_image, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), self.detection_color, 3)

				cx = int((bbox[0]+bbox[2])/2)
				cy = int((bbox[1]+bbox[3])/2)

				# draw the center of bounding box
				cv_image = cv2.circle(cv_image, (cx,cy), 5, self.detection_color, -1)

				self.faces.append((cx,cy))
				
				# rectangle points (x,y):
				#
				# bbox[0],bbox[3]----------------bbox[2],bbox[3]
				# |                                           |
				# bbox[0],bbox[1]----------------bbox[2],bbox[1]
				bb1x = int(bbox[0])
				bb1y = int(bbox[1])
				bb2x = int(bbox[0])
				bb2y = int(bbox[3])
				bb3x = int(bbox[2])
				bb3y = int(bbox[3])
				bb4x = int(bbox[2])
				bb4y = int(bbox[1])
				self.bounding_box.append((bb1x, bb1y, bb2x, bb2y, bb3x, bb3y, bb4x, bb4y))

			cv2.imshow("image", cv_image)
			key = cv2.waitKey(1)
			if key==27:
				print("exiting")
				exit()
			
		except CvBridgeError as e:
			print(e)

	def pointcloud_callback(self, data):

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

			#self.marker_pub.publish(marker)
			
		for bb1x, bb1y, bb2x, bb2y, bb3x, bb3y, bb4x, bb4y in self.bounding_box:
		        # get 3-channel representation of the poitn cloud in numpy format
			a = pc2.read_points_numpy(data, field_names= ("x", "y", "z"))
			a = a.reshape((height,width,3))
			
			# read corner coordinates
			p1 = a[bb1y,bb1x,:]
			p2 = a[bb2y,bb2x,:]
			p3 = a[bb3y,bb3x,:]
			p4 = a[bb4y,bb4x,:]
			
			# now creating arrays of the points
			p1_array = [float(p1[0]), float(p1[1]), float(p1[2])]
			p2_array = [float(p2[0]), float(p2[1]), float(p2[2])]
			p3_array = [float(p3[0]), float(p3[1]), float(p3[2])]
			p4_array = [float(p4[0]), float(p4[1]), float(p4[2])]
			
			# creating numpy vectors
			vector1 = np.array(p1_array)
			vector2 = np.array(p2_array)
			vector3 = np.array(p3_array)
			vector4 = np.array(p4_array)
			
			# calculating the vectors we need
			v1 = vector1 - vector3
			v2 = vector2 - vector4
			
			perpendicular_vector = np.cross(v1, v2)
			# making it length 1
			perpendicular_unit_vector = perpendicular_vector / np.linalg.norm(perpendicular_vector)
			
			# center of image
			v_center = 0.5 * (vector1 + vector3)
			
			# getting result
			scale = 0.3
			v_result1 = v_center + (scale * perpendicular_unit_vector)
			v_result2 = v_center - (scale * perpendicular_unit_vector)
			# choosing correct result (the one closer to robot -> the shorter one bc we are in robot frame)
			v_final_result = v_result1
			if np.linalg.norm(v_result2) <= np.linalg.norm(v_result1):
			    v_final_result = v_result2
			
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
			marker.color.g = 0.5
			marker.color.b = 0.0
			marker.color.a = 1.0

			# Set the pose of the marker
			marker.pose.position.x = v_final_result[0]
			marker.pose.position.y = v_final_result[1]
			marker.pose.position.z = v_final_result[2]
			
			self.marker_pub.publish(marker)
			
			

def main():
	print('Face detection node starting.')

	rclpy.init(args=None)
	node = detect_faces()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
