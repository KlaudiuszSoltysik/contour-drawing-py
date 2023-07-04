#!/usr/bin/python
import rclpy
from rospkg import RosPack
from rclpy.node import Node
from turtlesim.srv import SetPen, TeleportAbsolute
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

import cv2
import numpy as np
from math import sqrt
from simple_pid import PID


class ContourDrawing(Node):
    
    def __init__(self):
        super().__init__('contour_drawing')
        
        # VARIABLE INITIALIZATION
        self.pose = Pose()
        self.pid_x = PID(1, 0.15, 0.05, setpoint=1)
        self.pid_y = PID(1, 0.15, 0.05, setpoint=1)
        self.prepare = True
        
        # CONNECT WITH SET_PEN SERVICE
        self.set_pen_cli = self.create_client(SetPen, 'turtle1/set_pen')

        while not self.set_pen_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service SetPen not available, waiting again...')
            
        # CONNECT WITH TELEPORT SERVICE
        self.teleport_cli = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')

        while not self.teleport_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service TeleportAbsolute not available, waiting again...')

        # CREATE SUBSCRIBER TO POSE TOPIC
        self.pose_sub = self.create_subscription(Pose, 'turtle1/pose', self.pose_sub_callback, 10)
        self.pose_sub

        # CREATE PUBLISHER ON CMD_VEL TOPIC
        self.pose_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)                   
        
        # PROMPT USER
        self.get_logger().info('----------------------')
        self.get_logger().info('Contour drawing has been started.')
        
        image = cv2.imread("/home/klaudiusz/ros_contour_drawing_py/src/contour_drawing/img/img4.png")

        # Get the resolution
        self.height, self.width, _ = image.shape

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply adaptive thresholding
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)

        # Apply morphological operations to remove noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        # Find contours in the binary image
        self.contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Create a blank image to draw the contours on
        contour_image = np.zeros_like(image)

        # Draw the contours on the blank image
        cv2.drawContours(contour_image, self.contours, -1, (0, 255, 0), 3)

        x, y = self.contours[0][0][0]
        
        self.target_x = x * 11 / self.width
        self.target_y = 11 - (y * 11 / self.height)
        
        self.error_x = self.pose.x - self.target_x
        self.error_y = self.pose.y - self.target_y
        
        self.timer = self.create_timer(1 / 30, self.draw)


    def get_contours(self):       
        pass
        # Display the original image and the contour image
        # cv2.imshow("Original Image", image)
        # cv2.imshow("Contour Image", contour_image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        # for contour in contours:
        #     for coordinate in contour:
        #         x, y = coordinate[0]
                
        #         self.get_logger().info(f"Image x: {x}    y: {y}")
                
        #         x = x * 11 / width
        #         y = 11 - (y * 11 / height)

        #         self.move_turtle_to_coordinate(x, y)

    
    def draw(self):
        if self.prepare:
            control_x = self.pid_x(self.error_x)
            self.error_x = self.move_x(control_x)
            control_y = self.pid_y(self.error_y)
            self.error_y = self.move_y(control_y)
            
            self.prepare = False
        else:
            if abs(self.error_x) > 0.1:
                control_x = self.pid_x(self.error_x)
                self.error_x = self.move_x(control_x)
            
            if abs(self.error_y) > 0.1:   
                control_y = self.pid_y(self.error_y)
                self.error_y = self.move_y(control_y)
                
            if abs(self.error_x) < 0.5 and abs(self.error_y) < 0.5:
                self.get_coordinates()
        
    
    def move_x(self, control):
        msg = Twist()
             
        msg.linear.x = control
                
        self.pose_pub.publish(msg)
        
        return self.pose.x - self.target_x
    

    def move_y(self, control):
        msg = Twist()
             
        msg.linear.y = control
                
        self.pose_pub.publish(msg)
        
        return self.pose.y - self.target_y
    

    def pose_sub_callback(self, msg):
        self.pose = msg


    def get_coordinates(self):
        x, y = self.contours[0][1][0]
        
        self.target_x = x * 11 / self.width
        self.target_y = 11 - (y * 11 / self.height)
        
        self.error_x = self.pose.x - self.target_x
        self.error_y = self.pose.y - self.target_y


def main(args=None):
    rclpy.init(args=args)

    contour_drawing = ContourDrawing()

    rclpy.spin(contour_drawing)
    contour_drawing.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
