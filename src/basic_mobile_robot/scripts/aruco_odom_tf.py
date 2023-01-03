#!/usr/bin/env python3
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

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

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
e=subprocess.run(["bash /home/diego/Documents/JevoisArucoStream.sh"], shell=False)


class ARUCO_TF(Node):
    def __init__(self):
        super().__init__('Nodo_aruco_tf')
        
        #Publisher
        self.tf_broadcaster = TransformBroadcaster(self)
        
        #Subscriber
        self.subscriber_ = self.create_subscription(Odometry, '/odom_aruco', self.subscribe_message, 1)
        self.subscriber_  # prevent unused variable warning

    def publish_message(self): 
        line = ser.readline().rstrip()
        tok = line.split()
        if len(tok) < 1: print("ok")
        if len(tok) >= 8:
	# Asignamos variables a los valores:
	key, id, x, y, z, w, h, d, q1, q2, q3, q4= tok
	#LO NUEVO
	t = TransformStamped()
	t.header.stamp = self.get_clock().now().to_msg()
	t.header.frame_id = 'base_footprint'
	t.child_frame_id = str(id)
	t.transform.translation.x = float(x)
	t.transform.translation.y = float(y)
	t.transform.translation.z = float(z)
	t.transform.rotation.x = float(q1)
	t.transform.rotation.y = float(q2)
	t.transform.rotation.z = float(q3)
	t.transform.rotation.w = float(q4)
	self.tf_broadcaster.sendTransform(t)
		    
    
def main(args=None):
    rclpy.init(args=args)
    node = ARUCO_TF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
