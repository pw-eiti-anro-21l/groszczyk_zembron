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
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster, TransformStamped
from PyKDL import *
import time
import yaml

base_y=0.2
element1_param=[0.2, 0.2, 1.6]
element2_param=[0.2, 0.2, 0.2]
element3_param=[0.2, 0.2, 0.6]
element4_param=[]
element_param=[element1_param, element2_param, element3_param]
global RPY
global XYZ


data=[['r1'], ['r2'], ['r3']]

class KDL_dkin(Node):
    def __init__(self):
        super().__init__('kdl_dkin')
        self.subscription = self.create_subscription(
        JointState,
        'joint_states',
        self.listener_callback, 1)
        self.publisher = self.create_publisher(PoseStamped, 'kdl_dkin', 1)


    def listener_callback(self, msg):
        readParameters('parametryDH.txt')
        calculate(msg)

        joint = JntArray(3)
        joint[0]= msg.position[0]
        joint[1] = msg.position[1]
        joint[2] = msg.position[2]
        print(RPY[0][0],RPY[0][1],RPY[0][2])
        print("######")



        chain = Chain()

        base_link__link_1 = Joint(Joint.RotZ)
        frame1 = Frame(Rotation.RPY(RPY[0][0],RPY[0][1],RPY[0][2]),Vector(XYZ[0][0],XYZ[0][1],XYZ[0][2]))
        segment1 = Segment(base_link__link_1,frame1)
        chain.addSegment(segment1)


        link_1__link_2 = Joint(Joint.RotZ)
        frame2 = Frame(Rotation.RPY(RPY[1][0],RPY[1][1],RPY[1][2]),Vector(XYZ[1][0],XYZ[1][1],XYZ[1][2]))
        segment2=Segment(link_1__link_2,frame2)
        chain.addSegment(segment2)


        link_2__link_3 = Joint(Joint.RotZ)
        frame3 = Frame(Rotation.RPY(RPY[2][0],RPY[2][1],RPY[2][2]),Vector(XYZ[2][0],XYZ[2][1],XYZ[2][2]))
        segment3=Segment(link_2__link_3,frame3)
        chain.addSegment(segment3)

        fk=ChainFkSolverPos_recursive(chain)
        finalFrame=Frame()
        fk.JntToCart(joint,finalFrame)
        # print(finalFrame)
        # print("######")

        qua = finalFrame.M.GetQuaternion()

        xyz = finalFrame.p



        qos_profile = QoSProfile(depth=10)
        publisher = self.create_publisher(PoseStamped, '/pose_stamped_kdl', qos_profile)

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

def calculate(msg):
    global RPY, XYZ
    i=0
    M = []
    rpy_list = []
    xyz_list =[]



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
        rpy = m.to_euler()
        xyz = m.to_translation()
        rpy_list.append(rpy)
        xyz_list.append(xyz)


        M.append(m)
        i+=1

    trans_z= mathutils.Matrix.Translation((0, 0, 0))
    rot_z = mathutils.Matrix.Rotation(0, 4, 'Z')
    trans_x = mathutils.Matrix.Translation((a/2, 0, 0))
    rot_x =mathutils.Matrix.Rotation(0,4,  'X')
    m =trans_x @ rot_x @ rot_z@trans_z
    rpy = m.to_euler()
    xyz = m.to_translation()
    rpy_list.append(rpy)
    xyz_list.append(xyz)
    RPY = rpy_list
    XYZ = xyz_list



def readParameters(plik):
    i=0
    # if os.path.isfile(plik):
    with open(os.path.join(get_package_share_directory('zad2'),plik), 'r') as f:
        for line in f:
            line = line.strip('\n')
            line=line.split(" ")
            data[i].append(line)
            i+=1

    f.close()

    # else:
    # 	print("Blad odczytu danych!")

def readYAMLfile():


	with open(os.path.join(
		get_package_share_directory('zad2'),'urdf_parameters.yaml'), 'r') as file:

		data = yaml.load(file, Loader=yaml.FullLoader)

	my_data=[]

	joint1_RPY = data['row1']['j_rpy']
	joint1_Vector = data['row1']['j_xyz']
	joint2_RPY = data['row2']['j_rpy']
	joint2_Vector = data['row2']['j_xyz']
	joint3_RPY = data['row3']['j_rpy']
	joint3_Vector = data['row3']['j_xyz']

	my_data.extend((joint1_RPY,joint1_Vector,joint2_RPY,joint2_Vector,joint3_RPY,joint3_Vector))

	values = []
	for element in my_data:
		new_element = element.split()
		list_of_floats = [float(item) for item in new_element]
		values.append(list_of_floats)


	return values





def main(args=None):
    rclpy.init(args=args)

    kdl_dkin = KDL_dkin()

    rclpy.spin(kdl_dkin)

    kdl_dkin.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
