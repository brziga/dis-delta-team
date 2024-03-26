import rclpy
from rclpy.node import Node

from delta_interfaces.msg import ExplorerJob
from delta_interfaces.msg import JobStatus
from threading import Thread

import time

class Explorer(Node):

    def __init__(self):
        super().__init__('explorer')
        self.currently_executing_job = False
        self.id_of_current_job = ""
        self.currently_exploring = False
        
        # publishing the jobs status
        self.publisher_ = self.create_publisher(JobStatus, 'job_status', 1)
        timer_period = 1.0  # seconds
        self.publish_status_timer = self.create_timer(timer_period, self.publish_status)
        
        # listen to incoming jobs
        self.subscription = self.create_subscription(ExplorerJob, 'explorer_job', self.process_incoming_job, 1)
        self.subscription  # prevent unused variable warning
        
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
        time.sleep(5)
        
        # TODO: start exploring!  
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
        
        # just to simulate delay
        time.sleep(5)
        
        # TODO: stop exploring!  
        self.get_logger().info('stopped exploring! Finished job with id: '+self.id_of_current_job)
        
        # IMPORTANT: when job is complete do:
        self.currently_exploring= False
        self.currently_executing_job = False
        self.publish_status()
        

def main(args=None):
    rclpy.init(args=args)
    explorer = Explorer()
    rclpy.spin(explorer)
    
    explorer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
