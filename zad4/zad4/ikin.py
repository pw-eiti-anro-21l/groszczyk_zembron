import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from rclpy.clock import ROSClock
from rclpy.qos import QoSProfile
import time

from scipy.optimize import fsolve
from math import  pi, sin, cos
import numpy as np
from sympy import Symbol, solve, sin, cos, Eq, Matrix, nsolve


base_y=0.2
element1_param=[0.2, 0.2, 1.6]
element2_param=[0.2, 0.2, 0.2]
element3_param=[0.2, 0.2, 0.6]
element4_param=[]
element_param=[element1_param, element2_param, element3_param]



class ikin(Node):
	
	def __init__(self):
		super().__init__('ikin')
		self.limit=(pi/2)*0.5
		self.sphere_center=[0, 0, element1_param[2]+base_y+0.1]
		self.sphere_radius=4
		self.subscription = self.create_subscription(PoseStamped, '/oint_interpolate', self.listener_callback, 10)


		#self.subscription #prevent unused variable waring
		qos_profile = QoSProfile(depth=10)
		self.publisher = self.create_publisher(JointState, 'joint_interpolate', qos_profile)
		self.previous_joint_values=[0, 0, 0]

		


	def listener_callback(self, msg):
		
		self.position_x= msg.pose.position.x
		self.position_y= msg.pose.position.y
		self.position_z= msg.pose.position.z
		self.position= [self.position_x, self.position_y, self.position_z]


		bool=self.point_in_sphere(self.position_x, self.position_y, self.position_z)
		#self.get_logger().info(str(self.dataDH))


		
		i=0;
		v1=self.previous_joint_values[0]
		v2=self.previous_joint_values[1]
		v3=self.previous_joint_values[2]

		theta= fsolve(self.f, [v1, v2, v3])
		theta2= fsolve(self.f2, [v1, v2, v3])

		theta[0]=float(theta[0])%(2*pi)
		theta[0]=self.from_minus_pi_to_plus_pi(theta[0])

		theta[1]=float(theta[1])%(2*pi)
		theta[1]=self.from_minus_pi_to_plus_pi(theta[1])

		theta[2]=float(theta[2])%(2*pi)
		theta[2]=self.from_minus_pi_to_plus_pi(theta[2])


		theta2[0]=float(theta[0])%(2*pi)
		theta2[0]=self.from_minus_pi_to_plus_pi(theta2[0])

		theta2[1]=float(theta2[1])%(2*pi)
		theta2[1]=self.from_minus_pi_to_plus_pi(theta2[1])

		theta2[2]=float(theta[2])%(2*pi)*-1
		theta2[2]=self.from_minus_pi_to_plus_pi(theta2[2])

		if self.position_x==0 and self.position_y==0:
			theta[0]=self.previous_joint_values[0]
			theta2[0]= self.previous_joint_values[1]

		
		if abs(theta[1]-self.previous_joint_values[1])<=abs(theta2[1]-self.previous_joint_values[1]):
			solution=theta
		else:
			solution=theta2

		
			
		
		


		
		

		joint_states = JointState()
		joint_states.name = ['joint_base_element1', 'joint_element1_element2', 'joint_element2_element3']
		
		
		joint1_value=solution[0]		
		joint2_value=solution[1]
		joint3_value=solution[2]
		
		joint_states.position=[joint1_value, joint2_value, joint3_value]


		self.previous_joint_values[0]=joint1_value
		self.previous_joint_values[1]=joint2_value
		self.previous_joint_values[2]=joint3_value
		self.publisher.publish(joint_states)

		self.get_logger().info(str(theta)+" "+str(theta2))
		
		


	def point_in_sphere(self, x, y, z):
		if (x-self.sphere_center[0])**2+(y-self.sphere_center[1])**2+(z-self.sphere_center[2])**2>self.sphere_radius**2:
			return False
		else:
			return True

	def from_minus_pi_to_plus_pi(self, value):
		if value>pi:
			value= value- 2*pi
		if value<-pi:
			value= value+ 2*pi
		return value




	def f(self, x):
		theta1= x[0]
		theta2= x[1]
		theta3= x[2]
		l2=2
		l3=2
		T1 = np.array([
			[cos(theta1), -sin(theta1), 0, 0],
			[sin(theta1), cos(theta1), 0, 0],
			[0, 0, 1, 0],
			[0, 0, 0, 1]
			])
		T2 = np.array([
			[cos(theta2), -sin(theta2), 0, 0],
			[0, 0, 1, 0],
			[-sin(theta2), -cos(theta2), 0, 0],
			[0, 0, 0, 1]
			])
		T3 = np.array([
			[cos(theta3), -sin(theta3), 0, l2],
			[sin(theta3), cos(theta3), 0, 0],
			[0, 0, 1, 0],
			[0, 0, 0, 1]
			])
		T4 = np.array([
			[1, 0, 0, l3],
			[0, 1, 0, 0],
			[0, 0, 1, 0],
			[0, 0, 0, 1]
			])
		T= T1 @ T2 @ T3@ T4

		f=np.zeros(3)
		f[0]= T[0][3] -self.position_x
		f[1]= T[1][3] -self.position_y
		f[2]= T[2][3] -self.position_z+ self.sphere_center[2]

		return f

	def f2(self, x):
		theta1= self.previous_joint_values[0]
		p0=x[0]
		theta2= x[1]
		p=x[2]
		theta3= -self.previous_joint_values[2]
		l2=2
		l3=2
		T1 = np.array([
			[cos(theta1), -sin(theta1), 0, 0],
			[sin(theta1), cos(theta1), 0, 0],
			[0*p, 0*p0, 1, 0],
			[0, 0, 0, 1]
			])
		T2 = np.array([
			[cos(theta2), -sin(theta2), 0, 0],
			[0, 0, 1, 0],
			[-sin(theta2), -cos(theta2), 0, 0],
			[0, 0, 0, 1]
			])
		T3 = np.array([
			[cos(theta3), -sin(theta3), 0, l2],
			[sin(theta3), cos(theta3), 0, 0],
			[0, 0, 1, 0],
			[0, 0, 0, 1]
			])
		T4 = np.array([
			[1, 0, 0, l3],
			[0, 1, 0, 0],
			[0, 0, 1, 0],
			[0, 0, 0, 1]
			])
		T= T1 @ T2 @ T3@ T4

		f=np.zeros(3)
		f[0]= T[0][3] -self.position_x
		f[1]= T[1][3] -self.position_y
		f[2]= T[2][3] -self.position_z+ self.sphere_center[2]

		return f



def main(args=None):
	rclpy.init(args=args)

	ikin_node = ikin()
	rclpy.spin(ikin_node)

	ikin.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()