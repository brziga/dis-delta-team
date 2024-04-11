import rclpy
from rclpy.node import Node

from delta_interfaces.msg import SayText

import pyttsx3

class Speaker(Node):

    def __init__(self):
        super().__init__('greeter')
        
        self.subscription = self.create_subscription(SayText, 'say_text', self.process_msg, 10)
        self.subscription  # prevent unused variable warning
        
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty("rate", 160) # default rate is 200; subtracting 40 seems to sound better
    
    def process_msg(self, msg):
        self.tts_engine.say(msg.text)
        self.tts_engine.runAndWait()
        

def main(args=None):
    rclpy.init(args=args)
    speaker = Speaker()
    rclpy.spin(speaker)
    
    speaker.destroyNode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
