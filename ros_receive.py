import threading
import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from std_msgs.msg import String # type: ignore


def sensor_data_callback(m):
    print("sensor_data_callback not set!")
def servo_data_callback(m):
    print("servo_data_callback not set!")
def log_data_callback(m):
    print("log_data_callback not set!")

def set_sensor_data_callback(f):
    global sensor_data_callback
    sensor_data_callback=f
def set_servo_data_callback(f):
    global servo_data_callback
    servo_data_callback=f
def set_log_data_callback(f):
    global log_data_callback
    log_data_callback=f


def process_data(m):
    components=m.data.split(":")
    if components[0]=="log" or len(components)==2:
        data_type=components[0]
        data=components[1]
        if len(components)>2:
            data+=":" + components[2]
        if data_type=="sensor_data":
            sensor_data_callback(data)
        elif data_type=="servo":
            servo_data_callback(data)
        elif data_type=="log":
            log_data_callback(data)
        else:
            print(f"Error: Unknown data type: '{data_type}' received in 'pico/output'")
    else:
        print(f"Error: Can't parse pico/output data: '{m.data}'. Wrong number of colons.")



def listener():
    node=Node("groundstationlistener")
    node.create_subscription(String,"pico/output",process_data,10)
    rclpy.spin(node)


listener_thread=threading.Thread(target=listener)
listener_thread.start()
