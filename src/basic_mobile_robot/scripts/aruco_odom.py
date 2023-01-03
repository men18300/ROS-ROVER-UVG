import math
from math import sin, cos, pi
import serial
import struct
import time
import sys
import subprocess
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

serdev = '/dev/ttyACM0' # serial device of JeVois
ser= serial.Serial(
	port='/dev/ttyACM0', 
	baudrate=115200, 
	parity= serial.PARITY_NONE,
	stopbits= serial.STOPBITS_ONE,
	bytesize= serial.EIGHTBITS,
	timeout=1.5,
	inter_byte_timeout=0.1)
ser.close()
ser.open()
e=subprocess.run(["bash ~/JeVoisArucoStream.sh"], shell=True)

class DWM1001Node(Node):
    def __init__(self):
    
        super().__init__('Nodo_aruco')
        self.publisher_ = self.create_publisher(Odometry, '/odom_aruco', 50) 
        timer_period = 0.5  # seconds
        self.i = 0.0
        self.timer_ = self.create_timer(timer_period, self.publish_message)
        
    def publish_message(self): 
        line = ser.readline().rstrip()
        tok = line.split()
        if len(tok) < 1: print("ok")
        
        if len(tok) >= 8:
            # Asignamos variables a los valores:
            key, id, x, y, z, w, h, d, q1, q2, q3, q4= tok
            odom=Odometry()
            odom.header.frame_id = "odom_aruco"
            # set the position
            odom.pose.pose.position.x = float(x)
            odom.pose.pose.position.y = float(y)
            odom.pose.pose.position.z = float(z)
            odom.pose.pose.orientation.x = float(q1)
            odom.pose.pose.orientation.y = float(q2)
            odom.pose.pose.orientation.z = float(q3)
            odom.pose.pose.orientation.w = float(q4)
            # set the velocity
            odom.child_frame_id = str(id)
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = 0.0
            self.publisher_.publish(odom)
    
# ----------- CODIGO ----------- #
def main(args=None):
    rclpy.init(args=args)
    node = DWM1001Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
