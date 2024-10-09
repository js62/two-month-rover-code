from time import sleep #tmp
import math

import rclpy
rclpy.init()

import ros_send
import ros_receive

import pygame
import pygame_gui

pygame.display.set_caption("Bob, Did He? Rover Base Station")

#define callback functions for ros_receive
# def handle_log_data(data):
#     print("Log data: ",data)
# ros_receive.set_log_data_callback(handle_log_data)

#--GUI Variables--#
window_size = (1280, 720)
banner_size = 75 # This is in pixels
graph_border = 10


#define function for calcutating IK that gets called in the main loop

motor_angles=[0,0,0,0,0,0]
IK_target_pos=[0,0,0]

# def calculate IK
# def calculate direct motor control

running=True
pygame.init()
screen = pygame.display.set_mode(window_size)
gui_manager = pygame_gui.UIManager(window_size, "theme.json")
clock = pygame.time.Clock()


## THIS IS THE SECTION WITH ALL THE GUI ELEMENTS ##

# TODO: find a way to make this section collapsible >.>

## CONTAINERS ##
graphs_panel = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((window_size[0]/2+graph_border, banner_size+graph_border), (window_size[0]/2-(graph_border*2), (window_size[1]-banner_size)-(graph_border*2))), 
                                                                     manager=gui_manager)

## GUI ELEMENTS ##
graph_1_background = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((graph_border/2,graph_border/2),(window_size[0]/4-(graph_border*2), (window_size[1]-banner_size)/2-(graph_border*2))),
                                                  container=graphs_panel,
                                                  visible=True,
                                                  manager=gui_manager)

graph_2_background = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((window_size[0]/4-graph_border/2,graph_border/2),(window_size[0]/4-(graph_border*2), (window_size[1]-banner_size)/2-(graph_border*2))),
                                                  container=graphs_panel,
                                                  visible=True,
                                                  manager=gui_manager)

graph_3_background = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((graph_border/2,(window_size[1]-banner_size)/2-graph_border/2),(window_size[0]/4-(graph_border*2), (window_size[1]-banner_size)/2-(graph_border*2))),
                                                  container=graphs_panel,
                                                  visible=True,
                                                  manager=gui_manager)

graph_4_background = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((window_size[0]/4-graph_border/2,(window_size[1]-banner_size)/2-graph_border/2),(window_size[0]/4-(graph_border*2), (window_size[1]-banner_size)/2-(graph_border*2))),
                                                  container=graphs_panel,
                                                  visible=True,
                                                  manager=gui_manager)
while running:
    sleep(0.1)
    time_delta = clock.tick(60)/1000.0
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
        gui_manager.process_events(event)
        
    gui_manager.update(time_delta)
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
    gui_manager.draw_ui(screen)
    pygame.display.update()


rclpy.shutdown()
