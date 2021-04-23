import rclpy
from rclpy.node import Node
import numpy
from sensor_msgs.msg import JointState

from rclpy.clock import ROSClock

import os
import mathutils

from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster, TransformStamped

from rclpy.clock import ROSClock
import time

base_y=0.2
element1_param=[0.2, 0.2, 1.6]
element2_param=[0.2, 0.2, 0.2]
element3_param=[0.2, 0.2, 0.6]
element4_param=[]
element_param=[element1_param, element2_param, element3_param]

data=[['r1'], ['r2'], ['r3']]

class No_KDL(Node):
    def __init__(self):
        super().__init__('no_kdl')
        # self.DH = readParameters('parametryDH.txt'),
        self.subscription = self.create_subscription(
        JointState,
        'joint_states',
        self.listener_callback, 1)
        self.publisher = self.create_publisher(PoseStamped, 'no_kdl', 1)


    def listener_callback(self, msg):
        readParameters('parametryDH.txt')

        M_M = calculate(msg)

        rpy = M_M.to_euler()
        xyz = M_M.to_translation()
        qua = rpy.to_quaternion()
        qos_profile = QoSProfile(depth=10)
        publisher = self.create_publisher(PoseStamped, '/pose_stamped_nonkdl', qos_profile)

        pose = PoseStamped()
        now = self.get_clock().now()
        pose.header.stamp = ROSClock().now().to_msg()
        pose.header.frame_id = "base_link"


        pose.pose.position.x = xyz[0]
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]
        pose.pose.orientation = Quaternion(w=qua[0], x=qua[1], y=qua[2], z=qua[3])

        publisher.publish(pose)
        #self.get_logger().info('Publishing: "%s"' % pose)



def readParameters(plik):
    i=0
    if os.path.isfile(plik):
        with open(plik, "r") as f:
            x_ssafnlskjdf =1

            for line in f:
                line = line.strip('\n')
                line=line.split(" ")
                data[i].append(line)
                i+=1

        f.close()
        return data

    else:
    	print("Blad odczytu danych!")

def calculate(msg):
    i=0
    M = []

    for dat in data:
        name=dat[0]

        box_x= float(element_param[i][0])
        box_y= float(element_param[i][1])
        box_z= float(element_param[i][2])
    
        a=float(dat[1][0])
        d=float(dat[1][1])
        if i==0:
        	d=float(dat[1][1]) + box_z+ base_y+float(element_param[i+1][2])/2
        	#sztuczne przesuniecie o dlugosc elementow tak zeby joint 1 i join2 
        	#nie byly przesuniete wzgledem siebie

        alpha=float(dat[1][2])
        #wykomentowalem poniewaz strasznie szalalo przy sczytywaniu poprawki z msg
        theta=float(dat[1][3])+float(msg.position[i])
        trans_z= mathutils.Matrix.Translation((0, 0, d))
        rot_z = mathutils.Matrix.Rotation(theta, 4, 'Z')
        trans_x = mathutils.Matrix.Translation((a, 0, 0))
        rot_x =mathutils.Matrix.Rotation(alpha,4,  'X')
        m =trans_x @ rot_x @ rot_z@trans_z


        M.append(m)
        i+=1

    trans_z= mathutils.Matrix.Translation((0, 0, 0))
    rot_z = mathutils.Matrix.Rotation(0, 4, 'Z')
    trans_x = mathutils.Matrix.Translation((a/2, 0, 0))
    rot_x =mathutils.Matrix.Rotation(0,4,  'X')
    m =trans_x @ rot_x @ rot_z@trans_z


    M_M = M[0] @M[1]@ M[2]@m
    return M_M





def main(args=None):
    rclpy.init(args=args)

    no_kdl = No_KDL()

    rclpy.spin(no_kdl)

    no_kdl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
