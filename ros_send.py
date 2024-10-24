import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # type: ignore

node=Node("groundstation")
motor_control_topic=node.create_publisher(String,"pico/command",10)


def send_motor_positions(motor_poses):

    pin_vals=motor_poses

    message_text=""
    for i,t in enumerate(pin_vals):
        if i:
            message_text+=","
        if i==0:
            pass
        if i==1:
            t*=180/math.pi
        if i==2:
            t*=180/math.pi
            t*=180/70
        if i==3:
            t*=180/math.pi
        if i==4:
            pass
        message_text+=str(round(t,4))
    
    message=String()
    message.data=message_text
    motor_control_topic.publish(message)

