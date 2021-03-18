
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from curtsies import Input

class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds

        self.timer = self.create_timer(timer_period, self.turtle_control)

        self.declare_parameter('forward', 'w')
        self.declare_parameter('back', 'x')
        self.declare_parameter('left', 'a')
        self.declare_parameter('right', 'd')

        self.linear= 0
        self.angular= 0

    def set_velocity(self, lin, ang):
    	self.linear= float(lin)
    	self.angular= float(ang)


    def turtle_control(self):
    	forward = self.get_parameter('forward').get_parameter_value().string_value
    	back= self.get_parameter('back').get_parameter_value().string_value
    	left= self.get_parameter('left').get_parameter_value().string_value
    	right= self.get_parameter('right').get_parameter_value().string_value


    	with Input(keynames='curtsies') as input_generator:
    		#e=input_generator.send(0.1)
    		for e in Input():
    			e=str(e)
	    		if(e==forward):
	    			print('do przodu')
	    			self.set_velocity(1, 0)

	    		elif(e==back):
	    			print('do tylu')
	    			self.set_velocity(-1, 0)

	    		elif(e==left):
	    			print('w lewo')
	    			self.set_velocity(0, 1)

	    		elif(e==right):
	    			print('w prawo')
	    			self.set_velocity(0, -1)

	    		msg= Twist()
	    		msg.linear.x= self.linear
	    		msg.angular.z= self.angular
	    		self.publisher_.publish(msg)

        # self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # timer_period = 1.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)


    # def timer_callback(self):
    #     msg = Twist()
    #     msg.linear.x = 1.0
    #     msg.angular.z = 3.14
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.linear.x)

def main(args=None):
    rclpy.init(args=args)

    velocity_publisher = VelocityPublisher()

    rclpy.spin(velocity_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    velocity_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
