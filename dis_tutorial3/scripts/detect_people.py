#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from builtin_interfaces.msg import Duration

import os
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
		self.monalisas = []
		self.marker_id = 0

		script_dir = os.path.dirname(__file__)
		print(script_dir)
		print(script_dir)
		print(script_dir)
		print(script_dir)
		print(script_dir)
		rel_path = "../../../../src/dis-delta-team/dis_tutorial3/scripts/mona.png"
		abs_file_path = os.path.join(script_dir, rel_path)
		self.reference_image = cv2.imread(abs_file_path)
		self.reference_hist = self.calculate_histogram(self.reference_image)

		self.get_logger().info(f"Node has been initialized! Will publish face markers to {marker_topic}.")


	def calculate_histogram(self, image):
		hist = cv2.calcHist([image], [0, 1, 2], None, [8, 8, 8], [0, 256, 0, 256, 0, 256])
		cv2.normalize(hist, hist)
		return hist

	def compare_histograms(self, hist1, hist2):
		return cv2.compareHist(hist1, hist2, cv2.HISTCMP_CORREL)



	def rgb_callback(self, data):

		self.faces = []
		self.monalisas = []

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

				
				roi = cv_image[int(bbox[1]):int(bbox[3]), int(bbox[0]):int(bbox[2])]
				print(roi.shape)
				
				if roi.size == 0:
					self.get_logger().warning("Empty ROI, skipping this detection.")
					continue

				self.get_logger().info(f"ROI shape: {roi.shape}")
				if roi.shape[0] / roi.shape[1] > 4:
					continue

				roi_hist = self.calculate_histogram(roi)
				if roi_hist is None:
					self.get_logger().warning("Failed to calculate histogram for ROI, skipping this detection.")
					continue
				similarity = self.compare_histograms(self.reference_hist, roi_hist)
				self.get_logger().info(f"Histogram similarity: {similarity}")

				avg_b = np.mean(roi[:, :, 0])
				avg_g = np.mean(roi[:, :, 1])
				avg_r = np.mean(roi[:, :, 2])

				avg_rgb = (avg_r + avg_g + avg_b) / 3
				self.get_logger().info(f"Average R: {avg_r}, G: {avg_g}, B: {avg_b}, RGB: {avg_rgb}")

				cv2.imshow("ROI", roi)

				if similarity > 0.77:
					self.monalisas.append((cx,cy))
				elif similarity < 0.60:
					self.faces.append((cx,cy))
		
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
			marker.id = self.marker_id
			self.marker_id += 1
			marker.lifetime = Duration(sec=10000, nanosec=0)

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
		
		for x,y in self.monalisas:

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
			marker.id = self.marker_id
			self.marker_id += 1
			marker.lifetime = Duration(sec=10000, nanosec=0)

			# Set the scale of the marker
			scale = 0.1
			marker.scale.x = scale
			marker.scale.y = scale
			marker.scale.z = scale

			# Set the color
			marker.color.r = 1.0
			marker.color.g = 0.0
			marker.color.b = 0.0
			marker.color.a = 1.0

			# Set the pose of the marker
			marker.pose.position.x = float(d[0])
			marker.pose.position.y = float(d[1])
			marker.pose.position.z = float(d[2])
			

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
