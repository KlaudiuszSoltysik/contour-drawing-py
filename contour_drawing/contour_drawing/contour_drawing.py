#!/usr/bin/python
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

import cv2


class ContourDrawing(Node):
    
    def __init__(self):
        super().__init__('contour_drawing')
        
        # VARIABLE INITIALIZATION
        self.pose = Pose()
        
        self.is_first_run = True
        self.skip = False
        
        self.coordinate_number = 0
        self.contour_number = 0

        # CREATE SUBSCRIBER TO POSE TOPIC
        self.pose_sub = self.create_subscription(Pose, 'turtle1/pose', self.pose_sub_callback, 10)
        self.pose_sub

        # CREATE PUBLISHER ON CMD_VEL TOPIC
        self.pose_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)                   
        
        # EXTRACT CONTOURS FROM IMAGE
        image = cv2.imread("/home/klaudiusz/ros_contour_drawing_py/src/contour_drawing/img/img2.png")
        self.height, self.width, _ = image.shape
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray_inverted = cv2.bitwise_not(gray)
        _, binary = cv2.threshold(gray_inverted, 50, 255, cv2.THRESH_BINARY)
        self.contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        self.get_coordinate()
        
        # PROMPT USER
        self.get_logger().info('---------------------------------')
        self.get_logger().info('Contour drawing has been started.')
        
        self.timer = self.create_timer(1 / 30, self.draw)

    
    def draw(self):
        self.error_x = self.pose.x - self.target_x
        self.error_y = self.pose.y - self.target_y
        
        if abs(self.error_x) > 0.05:
            self.move_x(-5 * self.error_x)
        else:
            msg = Twist()
            msg.linear.x = 0.0
            self.pose_pub.publish(msg)
        
        if abs(self.error_y) > 0.05:
            self.move_y(-5 * self.error_y)
        else:
            msg = Twist()
            msg.linear.y = 0.0
            self.pose_pub.publish(msg)

        if abs(self.error_x) <= 0.05 and abs(self.error_y) <= 0.05:
            self.get_coordinate()
        
    
    def move_x(self, control):
        msg = Twist()
        msg.linear.x = control
        self.pose_pub.publish(msg)
    

    def move_y(self, control):
        msg = Twist()
        msg.linear.y = control    
        self.pose_pub.publish(msg)
    
    
    def get_coordinate(self):
        if self.skip:
            if self.contour_number < len(self.contours) - 1:
                self.skip = False
                self.contour_number += 1 
                self.coordinate_number = 0
            else:
                self.timer.cancel()
                self.get_logger().info('---------------------------------')
                self.get_logger().info('Drawing done.')
                self.get_logger().info('---------------------------------')
            
        if self.coordinate_number == len(self.contours[self.contour_number]):
            self.coordinate_number = 0
            self.skip = True        
        
        x, y = self.contours[self.contour_number][self.coordinate_number][0]
        
        self.target_x = 0.5 + x * 10 / self.width
        self.target_y = 10.5 - (y * 10 / self.height)
        
        self.coordinate_number += 1
        
        
    def pose_sub_callback(self, msg):
        self.pose = msg


def main(args=None):
    rclpy.init(args=args)

    contour_drawing = ContourDrawing()

    rclpy.spin(contour_drawing)
    contour_drawing.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
