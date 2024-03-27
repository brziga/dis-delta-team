import rclpy
from rclpy.node import Node

from delta_interfaces.msg import GreeterJob
from delta_interfaces.msg import JobStatus
from threading import Thread

import time


class Greeter(Node):

    def __init__(self):
        super().__init__('greeter')
        
        # information about the currently executed job
        self.currently_executing_job = False # is the job still beeing processed
        self.id_of_current_job = ""
        
        # publishing the jobs status
        self.publisher_ = self.create_publisher(JobStatus, 'job_status', 1)
        timer_period = 1.0  # seconds
        self.publish_status_timer = self.create_timer(timer_period, self.publish_status)
        
        # listen to incoming jobs
        self.subscription = self.create_subscription(GreeterJob, 'greeter_job', self.process_incoming_job, 1)
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
            self.publish_status()

        thread = Thread(target=self.greet_a_person, args=(msg.position_x, msg.position_y, msg.position_z, msg.rotation))
        thread.start()
        
    
    def greet_a_person(self, position_x, position_y, position_z, rotation):
    
        #just bc greeting not implemented yet, to simulate delay of greeting task:
        time.sleep(3)
        self.get_logger().info('Finished greeting job with id: '+self.id_of_current_job)
        
        # TODO: Perform greeting here! something like this:
        # move_to_position(position_x,position_y,rotation)
        # say("dober dan, person object")
        
        # IMPORTANT: after greeting has finished, do: self.currently_executing_job = False and publish_status() like this:
        self.currently_executing_job = False
        self.publish_status()
        # dont forget! only once greeting has finished!! or robo brain will think greeting is finished too early =O



def main(args=None):
    rclpy.init(args=args)
    greeter = Greeter()
    rclpy.spin(greeter)
    
    greeter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
