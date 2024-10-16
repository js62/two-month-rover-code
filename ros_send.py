import rclpy
from rclpy.node import Node
from std_msgs.msg import String # type: ignore

node=Node("groundstation")
motor_control_topic=node.create_publisher(String,"motorcontrol",10)

def set_send_positions_callback(f):
    global send_positions_callback
    send_positions_callback=f

def send_motor_positions(motor_poses):
    message_text=""
    for i,t in enumerate(motor_poses):
        if i:
            message_text+=","
        message_text+=str(round(t,4))
    
    message=String()
    message.data=message_text
    motor_control_topic.publish(message)
    send_positions_callback("Sent to libre: " + message.data)
    
    