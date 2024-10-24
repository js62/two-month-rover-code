import math
import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from std_msgs.msg import String # type: ignore

node=Node("groundstation")
motor_control_topic=node.create_publisher(String,"pico/command",10)


def send_motor_positions(motor_poses):
    old=motor_poses[:]
    motor_poses=old[1:4]
    motor_poses.append(old[0])
    motor_poses.append(old[4])
    

    pin_vals=motor_poses

    message_text="motor:"
    for i,t in enumerate(pin_vals):
        if i:
            message_text+=","
        

        if i==0: #joint 1
            t*=180/math.pi
        if i==1: #joint 2
            t*=180/math.pi
            t*=180/70
        if i==2: #joint 3
            t*=180/math.pi
        if i==3:
            t*=255
        if i==4:
            t*=255/2
        message_text+=str(round(t,4))
    
    message=String()
    message.data=message_text
    motor_control_topic.publish(message)

