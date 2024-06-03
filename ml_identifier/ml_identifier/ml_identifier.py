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
from pyzbar import pyzbar
import requests

from ultralytics import YOLO

from delta_interfaces.msg import MonalisaJob
from delta_interfaces.msg import JobStatus
from threading import Thread
import time

class ml_identifier(Node):

	def __init__(self):
		super().__init__('ml_identifier')

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
		self.top_rgb_image_sub = self.create_subscription(Image, "/top_camera/rgb/preview/image_raw", self.top_rgb_callback, qos_profile_sensor_data)
		self.pointcloud_sub = self.create_subscription(PointCloud2, "/oakd/rgb/preview/depth/points", self.pointcloud_callback, qos_profile_sensor_data)

		self.marker_pub = self.create_publisher(Marker, marker_topic, QoSReliabilityPolicy.BEST_EFFORT)

		self.model = YOLO("yolov8n.pt")

		self.faces = []
		self.qr_monalisa = None
		
		# information about the currently executed job
		self.currently_executing_job = False # is the job still beeing processed
		self.id_of_current_job = ""
		self.is_real_monalisa = False
		# publishing the jobs status
		self.job_publisher_ = self.create_publisher(JobStatus, 'job_status', 1)
		timer_period = 1.0  # seconds
		self.publish_status_timer = self.create_timer(timer_period, self.publish_job_status)
		# listen to incoming jobs
		self.job_subscription = self.create_subscription(MonalisaJob, 'monalisa_job', self.process_incoming_job, 1)
		self.job_subscription  # prevent unused variable warning
		
		self.startScanning = False


		self.get_logger().info(f"Node has been initialized! Will publish face markers to {marker_topic}.")
		
	def publish_job_status(self):
		msg = JobStatus()
		msg.acting = self.currently_executing_job
		msg.job_id = self.id_of_current_job
		msg.result_bool = self.is_real_monalisa
		self.job_publisher_.publish(msg)
		
	def process_incoming_job(self, msg):
		if self.id_of_current_job == msg.job_id or self.currently_executing_job == True:
			return
		
		self.id_of_current_job = msg.job_id
		self.currently_executing_job = True
		self.publish_job_status()
		
		if msg.scan_qr:
			thread = Thread(target = self.scan_qr_code)
			thread.start()
		else:
			thread = Thread(target = self.check_mona_lisa)
			thread.start()

	def scan_qr_code(self):
	
		# TODO: scan qr code here
		self.startScanning = True
		while self.startScanning:
			time.sleep(1)
		
		# when qr code scan has finished:
		self.currently_executing_job = False
		self.publish_job_status()
        
	def check_mona_lisa(self):
	
		# TODO: check if mona lisa is real and store it like this:
		self.is_real_monalisa = True # wow its the real mona lisa!
		# or
		self.is_real_monalisa = True # oh no, just another fake mona lisa
		
		# when mona lisa check is finished and answer stored in self.is_real_monalisa, do:
		self.currently_executing_job = False
		self.publish_job_status()

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

				self.get_logger().info(f"Person has been detected!")

				bbox = bbox[0]

				# draw rectangle
				cv_image = cv2.rectangle(cv_image, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), self.detection_color, 3)

				cx = int((bbox[0]+bbox[2])/2)
				cy = int((bbox[1]+bbox[3])/2)

				# draw the center of bounding box
				cv_image = cv2.circle(cv_image, (cx,cy), 5, self.detection_color, -1)

				self.faces.append((cx,cy))

			#cv2.imshow("image", cv_image)
			key = cv2.waitKey(1)
			if key==27:
				print("exiting")
				exit()
			
		except CvBridgeError as e:
			print(e)


	def download_image(self, url):
		response = requests.get(url)
		response.raise_for_status()

		image_array = np.asarray(bytearray(response.content), dtype=np.uint8)
		image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
		return image


	def top_rgb_callback(self, data):
		if not self.startScanning:
			return
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			qr_codes = pyzbar.decode(cv_image)
			for obj in qr_codes:
				obj_data = obj.data.decode("utf-8")
				#self.get_logger().info(f"Data:  {obj_data}")
				if obj_data.startswith("http") and obj_data.endswith(".png"):
					if self.qr_monalisa is None:
						self.get_logger().info(f"Saving mona lisa image")
						self.qr_monalisa = self.download_image(obj_data)
						cv2.imshow("Downloaded monalisa", self.qr_monalisa)
						self.startScanning = False
						
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
			"""marker = Marker()

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

			self.marker_pub.publish(marker)"""

def main():
	print('ml_identifier node starting.')

	rclpy.init(args=None)
	node = ml_identifier()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
