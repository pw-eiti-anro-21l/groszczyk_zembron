from rclpy.node import Node
import rclpy
import math
from sensor_msgs.msg import JointState
from rclpy.clock import ROSClock
import time
from rclpy.qos import QoSProfile
from zad4_srv.srv import RobotInterpolation

class jint_srv(Node):

    def __init__(self):
        super().__init__('jint_srv')
        self.srv = self.create_service(RobotInterpolation,'jint_control_srv', self.jint_control_callback)
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(JointState, 'joint_interpolate', qos_profile)
        self.start_position = [0,0,0]
        self.subscriber = self.create_subscription(JointState, 'joint_states',self.listener_callback, 10)
        self.in_action = False

    def listener_callback(self, msg):
        #pobieranie aktualnych położeń stawów
        if not self.in_action:
            self.start_position[0] = msg.position[0]
            self.start_position[1] = msg.position[1]
            self.start_position[2] = msg.position[2]

    def jint_control_callback(self, request, response ):
        self.in_action = True
        if self.req.interpolation_method==lin:
        	self.linear_interpolation(request)
        if self.req.interpolation_method==pol:
        	self.polynomial_interpolation(request)

        response.output = "Interpolation completed"
        self.in_action = False
        return response

    def linear_interpolation(self, request):
        time_interval = 0.1
        number_of_steps = math.floor(request.move_time/time_interval)
        joint_states = JointState()
        joint_states.name = ['joint_base_element1', 'joint_element1_element2', 'joint_element2_element3']
        start_positon = self.start_position

        for i in range(1, number_of_steps+1):
            joint1_current = self.start_position[0]+((request.joint1_pose - self.start_position[0])/request.move_time)*time_interval*i
            joint2_current = self.start_position[1]+((request.joint2_pose - self.start_position[1])/request.move_time)*time_interval*i
            joint3_current = self.start_position[2]+((request.joint3_pose - self.start_position[2])/request.move_time)*time_interval*i

            joint_states.position = [float(joint1_current), float(joint2_current), float(joint3_current)]
            print(joint_states)
            self.publisher.publish(joint_states)
            time.sleep(time_interval)

        self.start_positon = [joint1_current, joint2_current, joint3_current]

    def polynomial_interpolation(self,request):
        time_interval = 0.1
        number_of_steps = math.floor(request.move_time/time_interval)
        joint_states = JointState()
        joint_states.name = ['joint_base_element1', 'joint_element1_element2', 'joint_element2_element3']
        start_positon = self.start_positionn

        for i in range(1, number_of_steps+1):
        	if i==1:
		        a0= [start_position[0], start_position[1], start_position[2]]
		        a1= 0
		        a2= [3*(request.joint1_pose - self.start_position[0])/(request.move_time**2),
		        3*(request.joint2_pose - self.start_position[1])/(request.move_time**2),
		        3*(request.joint3_pose - self.start_position[2])/(request.move_time**2)
		        ]
		        a3= [-2*(request.joint1_pose - self.start_position[0])/(request.move_time**3),
		        -2*(request.joint2_pose - self.start_position[1])/(request.move_time**3),
		        -2*(request.joint2_pose - self.start_position[2])/(request.move_time**3),
		        ]
	        joint1_current = a0[0]+ a1[0]*(time_interval*i) + a2[0]*((time_interval*i)**2)+ a3[0]*((time_interval*i)**3)
	        joint2_current = a0[1]+ a1[1]*(time_interval*i) + a2[1]*((time_interval*i)**2)+ a3[1]*((time_interval*i)**3)
	        joint3_current = a0[2]+ a1[2]*(time_interval*i) + a2[2]*((time_interval*i)**2)+ a3[2]*((time_interval*i)**3)

	        joint_states.position = [float(joint1_current), float(joint2_current), float(joint3_current)]
	        print(joint_states)
	        self.publisher.publish(joint_states)
	        time.sleep(time_interval)
        self.start_positon = [joint1_current, joint2_current, joint3_current]

def main(args=None):
    rclpy.init(args=args)
    jint_node = jint_srv()

    rclpy.spin(jint_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
