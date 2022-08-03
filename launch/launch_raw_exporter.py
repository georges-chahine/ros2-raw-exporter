from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()
    

    node = Node(package = "raw_exporter", executable = "raw_exporter", output='screen',
    emulate_tty=True,
    arguments=[('__log_level:=debug')])
    #remappings=[('image_raw', '/zedm/zed_node/left_raw/image_raw_color')]
    
    ld.add_action(node)

    return ld
