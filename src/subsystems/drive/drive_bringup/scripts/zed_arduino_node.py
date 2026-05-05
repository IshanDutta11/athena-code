#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
#from std_msgs.msg import String,Int32, Float64MultiArray
from panorama_interfaces.srv import TakePanorama
from std_msgs.msg import Int32,Float32
import pyzed.sl as sl
import cv2 as cv
import math
import numpy as np
from pyfirmata import Arduino, util
import time

class Zed_Servo_Subscriber(Node):
    
    #Node Parameters: take_panorama:bool <-- Do we take the panorama? 
    #angle: 0.0 - 360.0 | frames = 100 - 2000
    # ros2 run node 360, 2000 0 0 
    def __init__(self):
        super().__init__(node_name='Zed_Servo_Subscriber')
        self.board = Arduino("COM4")#Whichever port it is, can edit
        self.servo =  self.board.get_pin('d:9:s') #Servo on pin 9 Also can edit
        time.sleep(2) # Wait for board to initialize
        self.board.samplingOn()
        self.subscriber = self.create_subscription(
            Float32,
            'temp_topic',
            self.servo_callback,
            10
        )
    def servo_callback(self,msg):
        angle = msg.data
        try:
            self.servo.write(angle)#[0]
        except Exception as e:
            self.get_logger().info(f"Failed to rotate servo:{e}")
def main(args=None):
    rclpy.init(args=args) # Initializes the ros2 communication
    node = Zed_Servo_Subscriber()
    rclpy.spin(node=node) # Node spin = Keep this Node alive until we kill it
    #Maybe spin it up once then kill the subscription to get the location
    rclpy.shutdown() #Shutsdown a Node

#Considerable if we find black borders
# def crop_black_borders(self, image):
#     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#     _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
#     contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     x, y, w, h = cv2.boundingRect(contours[0])
#     return image[y:y+h, x:x+w]
    

if __name__ == "__main__":
    main()