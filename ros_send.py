import rclpy
from rclpy.node import Node
from std_msgs.msg import String # type: ignore

rclpy.init()
node=Node("groundstation")
motor_control_topic=node.create_publisher(String,"motorcontrol",10)


def send_motor_positions(motor_poses):
    message_text=""
    for i,t in enumerate(motor_poses):
        if i:
            message_text+=","
        message_text+=str(round(t,4))
    
    message=String()
    message.data=message_text
    motor_control_topic.publish(message)
