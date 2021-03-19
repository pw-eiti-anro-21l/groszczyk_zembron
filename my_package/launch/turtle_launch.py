from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            # namespace='lab1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='my_package',
            # namespace='lab1',
            executable='my_node',
            name='velocity_publisher',
            prefix=['gnome-terminal --disable-factory -- '],
            # parameters = [
            #     {'forward' : 'w'},
            #     {'back' : 'x'},
            #     {'left' : 'a'},
            #     {'right' : 'd'}
            # ]
        ),

    ])
