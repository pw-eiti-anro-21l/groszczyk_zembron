from rclpy.node import Node
import rclpy
import math
from rclpy.clock import ROSClock
import time
from rclpy.qos import QoSProfile
from zad4_srv.srv import RobotInterpolationZad5
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Quaternion
import transforms3d
import matplotlib.pyplot as plt
from math import sin, cos, pi

base_y=0.2
element1_param=[0.2, 0.2, 1.6]
element2_param=[0.2, 0.2, 0.2]
element3_param=[0.2, 0.2, 0.6]
element4_param=[]
element_param=[element1_param, element2_param, element3_param]

class oint_srv(Node):

	def __init__(self):
		super().__init__('oint_srv')
		self.sphere_center=[0, 0, element1_param[2]+base_y+0.1]
		self.sphere_radius=4

		self.srv = self.create_service(RobotInterpolationZad5,'oint_control_srv', self.oint_control_callback)
		qos_profile = QoSProfile(depth=10)
		self.publisher = self.create_publisher(PoseStamped, '/oint_interpolate', qos_profile)
		self.marker_publisher = self.create_publisher(MarkerArray, '/marker', qos_profile)
		self.start_position = [4, 0, 1.9]
		

	def oint_control_callback(self, request, response):
		if request.move_time > 0:
			if request.interpolation_method=='lin':
				self.linear_interpolation(request)
			elif request.interpolation_method=='pol':
				self.polynomial_interpolation(request)
			elif request.interpolation_method=="rectangle":
				self.rectangle_interpolation(request)
			elif request.interpolation_method=="ellipse":
				self.ellipse_interpolation(request)
			else:
				self.linear_interpolation(request)
				response.output= "[!!!!!] Invalid interpolation method input. Methon changed to linear interpolation [!!!!!]"
			response.output = "Interpolation completed"
		else:
			self.get_logger().info('Time must be positive!!')

		return response


	def linear_interpolation(self, request):
		time_interval = 0.1
		number_of_steps = math.floor(request.move_time/time_interval)
		pose = PoseStamped()
		marker = Marker()
		marker_array = MarkerArray()
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.color.a = 1.0
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.type = 1
		marker.action = 0
		marker.header.frame_id = "/base_link"



		for i in range(1, number_of_steps+1):
			x_current = self.start_position[0]+((request.x_pose- self.start_position[0])/request.move_time)*time_interval*i
			y_current = self.start_position[1]+((request.y_pose - self.start_position[1])/request.move_time)*time_interval*i
			z_current = self.start_position[2]+((request.z_pose - self.start_position[2])/request.move_time)*time_interval*i



			pose.header.frame_id = "base_link"
			pose.pose.position.x = x_current
			pose.pose.position.y = y_current
			pose.pose.position.z = z_current

			if self.point_in_sphere(x_current, y_current, z_current):

				self.publisher.publish(pose)

				marker.pose.position.x = x_current
				marker.pose.position.y = y_current
				marker.pose.position.z = z_current
			
				marker_array.markers.append(marker)
				id=0
				for marker in marker_array.markers:
				    marker.id = id
				    id += 1
				self.marker_publisher.publish(marker_array)

				time.sleep(time_interval)
				self.start_position=[x_current, y_current, z_current]
			else:
				self.get_logger().info("Dany punkt jest nieosiągalny")

		
		


	def polynomial_interpolation(self, request):
		time_interval = 0.1
		number_of_steps = math.floor(request.move_time/time_interval)
		pose = PoseStamped()
		marker = Marker()
		marker_array = MarkerArray()
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 1.0
		marker.type = 1
		marker.action = 0
		marker.header.frame_id = "/base_link"

		a0= [self.start_position[0], self.start_position[1], self.start_position[2]]
		a1= [0,0,0]
		a2= [3*(request.x_pose - self.start_position[0])/(request.move_time**2),
		3*(request.y_pose - self.start_position[1])/(request.move_time**2),
		3*(request.z_pose - self.start_position[2])/(request.move_time**2)
		]
		a3= [-2*(request.x_pose - self.start_position[0])/(request.move_time**3),
		-2*(request.y_pose - self.start_position[1])/(request.move_time**3),
		-2*(request.z_pose - self.start_position[2])/(request.move_time**3),
		]



		for i in range(1, number_of_steps+1):
			x_current = a0[0]+ a1[0]*(time_interval*i) + a2[0]*((time_interval*i)**2)+ a3[0]*((time_interval*i)**3)
			y_current = a0[1]+ a1[1]*(time_interval*i) + a2[1]*((time_interval*i)**2)+ a3[1]*((time_interval*i)**3)
			z_current = a0[2]+ a1[2]*(time_interval*i) + a2[2]*((time_interval*i)**2)+ a3[2]*((time_interval*i)**3)


			if self.point_in_sphere(x_current, y_current, z_current):
				pose.header.frame_id = "base_link"
				pose.pose.position.x = x_current
				pose.pose.position.y = y_current
				pose.pose.position.z = z_current

				self.publisher.publish(pose)


				marker.pose.position.x = x_current
				marker.pose.position.y = y_current
				marker.pose.position.z = z_current

				marker_array.markers.append(marker)
				id=0
				for marker in marker_array.markers:
				    marker.id = id
				    id += 1
				self.marker_publisher.publish(marker_array)

				time.sleep(time_interval)
				self.start_position=[x_current, y_current, z_current]
			else:
				self.get_logger().info("Dany punkt jest nieosiągalny")
		

	def interpolate_ellipse(self, request):
		time_interval = 0.1
		number_of_steps = math.floor(request.move_time/time_interval)
		pose = PoseStamped()
		marker = Marker()
		marker_array = MarkerArray()
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.color.a = 1.0
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.type = 1
		marker.action = 0
		marker.header.frame_id = "/base_link"

		x_current=self.start_position[0]
		start_position=self.start_position

		a= request.a
		b= request.b

		request.y_pose=start_position[1]+ a*cos(2*pi/number_of_steps)
		request.z_pose=start_position[2]+ b*sin(2*pi/number_of_steps)
		self.linear_interpolation(request)
		
		for i in range(1, number_of_steps+1):
			
			y_current = start_position[1]+ a*cos(2*pi*i/number_of_steps)
			z_current = start_position[2]+ b*sin(2*pi*i/number_of_steps)



			pose.header.frame_id = "base_link"
			pose.pose.position.x = x_current
			pose.pose.position.y = y_current
			pose.pose.position.z = z_current

			if self.point_in_sphere(x_current, y_current, z_current):

				self.publisher.publish(pose)

				marker.pose.position.x = x_current
				marker.pose.position.y = y_current
				marker.pose.position.z = z_current
			
				marker_array.markers.append(marker)
				id=0
				for marker in marker_array.markers:
				    marker.id = id
				    id += 1
				self.marker_publisher.publish(marker_array)

				time.sleep(time_interval)
			else:
				self.get_logger().info("Zadana trajektoria jest nieosiągalna.")
				break;
			self.start_position=[x_current, y_current, z_current]
      

	def point_in_sphere(self, x, y, z):
		if (x-self.sphere_center[0])**2+(y-self.sphere_center[1])**2+(z-self.sphere_center[2])**2>self.sphere_radius**2:
			return False
		else:
			return True



	def rectangle_interpolation(self, request):
		self.linear_interpolation(request)

		request.z_pose+=request.a
		self.linear_interpolation(request)
		request.x_pose-=request.b
		self.linear_interpolation(request)
		request.z_pose-=request.a
		self.linear_interpolation(request)
		request.x_pose+=request.b
		self.linear_interpolation(request)

	def ellipse_interpolation(self, request):
		self.linear_interpolation(request)
		self.interpolate_ellipse(request)





def main(args=None):
    rclpy.init(args=args)
    oint_node = oint_srv()

    rclpy.spin(oint_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
