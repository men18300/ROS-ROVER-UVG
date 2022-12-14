#!/usr/bin/env python3
import math
from math import sin, cos, pi


import serial
import struct
import time
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


dwm_ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

dwm_ser.close()  
dwm_ser.open()   

if dwm_ser.isOpen() == True:
	print("Serial communication is open...")
else:
	print("Error opening serial communication")
	#print(dwm_ser.isOpen())

# ----------- FUNCIONES ----------- #

### POS GET ###
def dwm_pos_get(): #(pag.41)
	d_send= [0x02, 0x00] #dwm_pos_get 
	dwm_ser.write(serial.to_bytes(d_send))
	s = dwm_ser.read(18)
	return s

def get_coord(s):
	n1 = 5 # En esta posicion comienza los valores de X
	n2 = 9 # En esta posicion comienza los valores de Y
	n3 = 13 # En esta posicion comienza los valores de Z
	Xa = [0,0,0,0]
	Ya = [0,0,0,0]
	Za = [0,0,0,0]
	for i in range(4):
		Xa[i] = s[n1]
		Ya[i] = s[n2]
		Za[i] = s[n3]
		n1 = n1 + 1
		n2 = n2 + 1
		n3 = n3 + 1
		i = i + 1
	X = ((Xa[3]*(2**24))+(Xa[2]*(2**16))+(Xa[1]*(2**8))+(Xa[0]))/1000 # Se pasa a decimal
	Y = ((Ya[3]*(2**24))+(Ya[2]*(2**16))+(Ya[1]*(2**8))+(Ya[0]))/1000 # Se pasa a decimal
	Z = ((Za[3]*(2**24))+(Za[2]*(2**16))+(Za[1]*(2**8))+(Za[0]))/1000 # Se pasa a decimal
	return X,Y,Z
	
def get_X(s):
	n1 = 5 # En esta posicion comienza los valores de X
	Xa = [0,0,0,0]
	for i in range(4): #Solo 4 valores son de X
		Xa[i] = s[n1]
		n1 = n1 + 1
		i = i + 1
	X_mm = ((Xa[3]*(2**24))+(Xa[2]*(2**16))+(Xa[1]*(2**8))+(Xa[0]))/1000 # Se pasa a decimal
	return X_mm#, X_cm, X_m

def get_Y(s):
	n2 = 9 # En esta posicion comienza los valores de Y
	Ya = [0,0,0,0]
	for i in range(4):
		Ya[i] = s[n2]
		n2 = n2 + 1
		i = i + 1
	Y_mm = ((Ya[3]*(2**24))+(Ya[2]*(2**16))+(Ya[1]*(2**8))+(Ya[0]))/1000 # Se pasa a decimal
	return Y_mm#, Y_cm, Y_m

def get_Z(s): 
	n3 = 13 # En esta posicion comienza los valores de Z
	Za = [0,0,0,0]
	for i in range(4):
		Za[i] = s[n3]
		n3 = n3 + 1
		i = i + 1
	Z_mm = ((Za[3]*(2**24))+(Za[2]*(2**16))+(Za[1]*(2**8))+(Za[0]))/1000 # Se pasa a decimal
	return Z_mm#, Z_cm, Z_m


## PARTE DE ROS %%
class DWM1001Node(Node):

    def __init__(self):
    
        super().__init__('Nodo_dwm1001')
        self.publisher_ = self.create_publisher(Odometry, "/wheel/odometry", 50) 
        timer_period = 0.5  # seconds
        self.i = 0.0
        self.timer_ = self.create_timer(timer_period, self.publish_message)

    def publish_message(self):  
    	s = dwm_pos_get()
    	C = get_coord(s) 
    	print("X = "+str(C[0]))
    	print("Y = "+str(C[1]))
    	odom=Odometry()
    	odom.header.stamp = self.get_clock().now().to_msg()
    	odom.header.frame_id = "odom"
    	odom.child_frame_id= "base_footprint"
    	# set the position
    	odom.pose.pose.position.x = C[0]
    	odom.pose.pose.position.y = C[1]
    	odom.pose.pose.position.z = C[2]
    	odom.pose.pose.orientation.x = 0.0
    	odom.pose.pose.orientation.y = 0.0
    	odom.pose.pose.orientation.z = 0.0
    	odom.pose.pose.orientation.w = 0.0
    	# set the velocity
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
    

