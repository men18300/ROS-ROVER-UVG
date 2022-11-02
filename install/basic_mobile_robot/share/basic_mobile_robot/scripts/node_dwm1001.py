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
	#X_cm = X_mm/10 # se convierte a cm
	#X_m = X_mm/1000 #se convierte a metro
	#print(X)
	return X_mm#, X_cm, X_m

def get_Y(s):
	n2 = 9 # En esta posicion comienza los valores de Y
	Ya = [0,0,0,0]
	for i in range(4):
		Ya[i] = s[n2]
		n2 = n2 + 1
		i = i + 1
	Y_mm = ((Ya[3]*(2**24))+(Ya[2]*(2**16))+(Ya[1]*(2**8))+(Ya[0]))/1000 # Se pasa a decimal
	#Y_cm = Y_mm/10 # se convierte a cm
	#Y_m = Y_mm/1000 #se convierte a metro
	#print(Y)
	return Y_mm#, Y_cm, Y_m

def get_Z(s): 
	n3 = 13 # En esta posicion comienza los valores de Z
	Za = [0,0,0,0]
	for i in range(4):
		Za[i] = s[n3]
		n3 = n3 + 1
		i = i + 1
	Z_mm = ((Za[3]*(2**24))+(Za[2]*(2**16))+(Za[1]*(2**8))+(Za[0]))/1000 # Se pasa a decimal
	#Z_cm = Z_mm/10 # se convierte a cm
	#Z_m = Z_mm/1000 #se convierte a metro
	#print(Z)
	return Z_mm#, Z_cm, Z_m



### MODULO CONFIG SET ###
def dwm_cfg_set(): #(pag.48)
	d_send= [0x07, 0x02, 0x9e, 0x00] #dwm_cfg_set 
	dwm_ser.write(serial.to_bytes(d_send))
	s = dwm_ser.read(3)
	return s
		
### MODULO CONFIG GET ###
def dwm_cfg_get(): #(pag.48)
	d_send= [0x08, 0x00] #dwm_cfg_get 
	dwm_ser.write(serial.to_bytes(d_send))
	s = dwm_ser.read(7)
	return s
	
### MODULO BAUDRATE GET ###r
def dwm_baddr_get(): #(pag.56)
	d_send= [0x10, 0x00] #dwm_baddr_get 
	dwm_ser.write(serial.to_bytes(d_send))
	s = dwm_ser.read(11)
	return s
	
### MODULO STATUS GET ###
def dwm_status_get(): #(pag.76)
	d_send= [0x32, 0x00] #dwm_status_get 
	dwm_ser.write(serial.to_bytes(d_send))
	s = dwm_ser.read(7)
	return s


### SET GPIO 13 HIGH ### (EJEMPLO PARA COMPROBARR)
def set_13_high(): #(pag.25)
	d_send= [0x28, 0x02,0x0D,0x01]  #se espera 40 01 00 
	dwm_ser.write(serial.to_bytes(d_send))
	s = dwm_ser.read(3)
	return s

### MODULO POSITION UPDATE RATE ### (que tan rapido actualiza la posicion, que tan rapido vuelve a leer datos)
def dwm_upd_rate_get(): #(pag.43)
	d_send= [0x32, 0x00] #dwm_upd_rate_get 
	dwm_ser.write(serial.to_bytes(d_send))
	s = dwm_ser.read(9)
	return s

### MODULO PANID GET ###
def dwm_panid_get(): #(pag.74)
	d_send= [0x2F, 0x00] #dwm_panid_get 
	dwm_ser.write(serial.to_bytes(d_send))
	s = dwm_ser.read(7)
	return s

## PARTE DE ROS %%
class DWM1001Node(Node):

    def __init__(self):
    
        super().__init__('Nodo_dwm1001')
        self.publisher_ = self.create_publisher(Odometry, '/odom_nati', 50) 
        timer_period = 0.5  # seconds
        self.i = 0.0
        self.timer_ = self.create_timer(timer_period, self.publish_message)

    def publish_message(self):  
    	s = dwm_pos_get()
    	C = get_coord(s) 
    	print("X = "+str(C[0]))
    	print("Y = "+str(C[1]))
    	odom=Odometry()
    	odom.header.frame_id = "odom_nati"
    	# set the position
    	odom.pose.pose.position.x = C[0]
    	odom.pose.pose.position.y = C[1]
    	odom.pose.pose.position.z = C[2]
    	odom.pose.pose.orientation.x = 0.0
    	odom.pose.pose.orientation.y = 0.0
    	odom.pose.pose.orientation.z = 0.0
    	odom.pose.pose.orientation.w = 0.0
    	# set the velocity
    	odom.child_frame_id = "base_link"
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
    


#~/ws_dwm1001/src/pack_dwm1001/pack_dwm1001
