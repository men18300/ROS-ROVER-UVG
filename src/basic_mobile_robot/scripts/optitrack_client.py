#!/usr/bin/env python3
from warnings import catch_warnings
import rclpy
from rclpy.node import Node
import socket
import json
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class OptiTrackNode(Node): 
    def __init__(self):
        super().__init__("optitrack_connection") 
        # Start sochet connection with Robotat
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect(('192.168.50.200',1883))
        self.fullMsg = self.s.recv(1024).decode("utf-8")
        self.get_logger().info(self.fullMsg) #Verify Robotat has connected

        #Create publisher to publish the data in a topic
        self.publisher_ = self.create_publisher(Odometry, "/wheel/odometry", 50)
        self.timer_ = self.create_timer(0.05,self.publish_coordinates)
        self.get_logger().info("The number publisher has been started")

    def publish_coordinates(self):
        #Request the server the coordinates from markers 1 and 2
        self.markerNumbers = [8,4]
        self.s.sendall(json.dumps(self.markerNumbers).encode())

        #Receive the coordinates
        self.coordinates = self.s.recv(1024).decode("utf-8")
       # self.get_logger().info(self.coordinates)
        self.jsonCoordinates = json.loads(self.coordinates)
        self.M1_Pos = self.jsonCoordinates[0:3]
        self.M1_Ori = self.jsonCoordinates[3:7]
        self.M2_Pos = self.jsonCoordinates[7:10]
        self.M2_Ori = self.jsonCoordinates[10:13]

        #Start Odometry
        odom=Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id= "base_footprint"

        # set the position
        odom.pose.pose.position.x = self.M1_Pos[0]
        odom.pose.pose.position.y = self.M1_Pos[2]
        odom.pose.pose.position.z = self.M1_Pos[1]
        odom.pose.pose.orientation.x = self.M1_Ori[0]
        odom.pose.pose.orientation.y = self.M1_Ori[2]
        odom.pose.pose.orientation.z = -self.M1_Ori[1]
        odom.pose.pose.orientation.w = self.M1_Ori[3]
        
        #Publish de Odometry in the topic
        self.publisher_.publish(odom)

 
def main(args=None):
    rclpy.init(args=args)
    node = OptiTrackNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
