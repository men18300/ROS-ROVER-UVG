#!/usr/bin/env python3

import time
import sys
import rclpy

import math
from math import sin, cos, pi

from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from math import atan2
l=0.67
r=0.32

x=0.0
y=0.0
theta=0.0

#Para los calculos
#PID posición
kpO = 1.2
kiO = 0.01 
kdO = 0
EO = 0
eO_1 = 0

#En simulacion
v0 = 0.3
alpha = 0.4

real=0

if real==1:
    kpO = 25.0
    kiO = 0.01 
    kdO = 0

    v0 = 2.40
    alpha = 0.9


class ControladorVelocidad(Node):

    def __init__(self):
        global speed
        global goal
    
        super().__init__('controller_vel')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel',1) 
        timer_period = 0.5  # seconds
        self.i = 0.0
        self.timer_ = self.create_timer(timer_period, self.publish_message)
        
        #Agregado para probar suscripcion
        self.subscriber_ = self.create_subscription(Odometry,'/wheel/odometry', self.subscribe_message,1)
        self.subscriber_  # prevent unused variable warning
        
        speed=Twist()
        goal=Point()
        goal.x=8.0
        goal.y=-7.0
        
        #Coordenada deseada
        #goal.x=-0.001
        #goal.y=-0.001

    def publish_message(self):   
        global goal
        global x
        global y
        global theta
        global eP
        global eP_1
        global eO_1
        global eO
        global eO_D
        global EP
        global EO

        inc_x = goal.x-x
        inc_y = goal.y-y
        thetag=atan2(inc_y,inc_x)
        
        eP = math.sqrt(inc_x*inc_x+inc_y*inc_y)
        eO = thetag - theta
        eO = atan2(sin(eO), cos(eO))
        
        # Control de velocidad lineal
        kP = v0 * (1-math.exp(-alpha*eP*eP)) / eP;
        v = kP*eP;
        
        #Pruebas con real
        
        #v=0.0;
        
        # Control de velocidad angular
        eO_D = eO - eO_1;
        EO = EO + eO;
        w = kpO*eO + kiO*EO + kdO*eO_D;
        
        #Para pruebas con el ROver UVG real
        if real == 1:
            w=-w
        
    
        #w=0.0;
        eO_1 = eO;

        ####################################
        ##SOLO PARA VER QUE ESTA ENVIANDO
        left_wheel_vel=(v+(w*l))/r
        #LLanta derecha
        right_wheel_vel=(v-(w*l))/r
        
        print("w:", w)
        #print("LLANTA IZQUIERDA:",left_wheel_vel)
        #print("LLANTA DERECHA:", right_wheel_vel)
        ####################################    
        
        speed.linear.x=v
        speed.angular.z=w
        #print(v)
        #print(w)
        self.publisher_.publish(speed)
        
        
        
    def subscribe_message(self,msg):
        global x
        global y
        global theta
        
        x=msg.pose.pose.position.x
        y=msg.pose.pose.position.y	
        
        rot_q=msg.pose.pose.orientation
        (roll,pitch,theta)=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        
        #Pruebas con ODOMETRIA DE JORGE
        #theta=msg.pose.pose.position.z
        #print("x:" , x)
        #print("y:" , y)
    

def main(args=None):
    rclpy.init(args=args)
    nodo = ControladorVelocidad()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

