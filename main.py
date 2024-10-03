from time import sleep #tmp

import rclpy
rclpy.init()

import ros_send
import ros_receive

import pygame


#define callback functions for ros_receive
# def handle_log_data(data):
#     print("Log data: ",data)
# ros_receive.set_log_data_callback(handle_log_data)


#define function for calcutating IK that gets called in the main loop

motor_angles=[0,0,0,0,0,0]

IK_target_pos=[0,0,0]

# def calculate IK
# def calculate direct motor control

running=True
pygame.init()
screen = pygame.display.set_mode((1280, 720))

while running:
    sleep(0.1)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    #draw GUI
        #graph data
        #displaying logs
        #displaying motor angles and stuff
    
    # if IK:
    #     calculate_IK()
    # else:
    #     direct_motor_control()
    # clamp_andgles()

    screen.fill("blue")
    pygame.display.flip()


rclpy.shutdown()
