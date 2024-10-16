from time import sleep #tmp
import math

import pygame_gui.elements.ui_label

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
log_size = 250
graph_border = 10

graphs_panel_width = window_size[0]/2-(graph_border*2)
graphs_panel_height = (window_size[1]-banner_size)-(graph_border*2)
graph_size = (graphs_panel_height/2 - graph_border, graphs_panel_width/2 - graph_border)

#--Debug Variables--#
input_1 = [0,0] #This is just for the left stick


#define function for calcutating IK that gets called in the main loop

motor_angles=[0,0,0,0,0,0]
IK_target_pos=[0,0,0]

# def calculate IK
# def calculate direct motor control

running=True
pygame.init()
screen = pygame.display.set_mode(window_size,flags=pygame.RESIZABLE)
gui_manager = pygame_gui.UIManager(window_size, "theme.json")
clock = pygame.time.Clock()


## THIS IS THE SECTION WITH ALL THE GUI ELEMENTS ##

# TODO: find a way to make this section collapsible >.>

## CONTAINERS ##

def setup_gui():
    global graphs_panel, claw_panel, graph_1_background, graph_2_background, graph_3_background, graph_4_background, console_log

    graphs_panel = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((window_size[0]/2+graph_border, banner_size+graph_border), (graphs_panel_width, graphs_panel_height)), 
                                                                        manager=gui_manager)
    claw_panel = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((graph_border, banner_size+graph_border), (graphs_panel_width, graphs_panel_height -log_size)), 
                                                                        manager=gui_manager)

    ## GUI ELEMENTS ##
    graph_1_background = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((graph_border/2,graph_border/2), graph_size),
                                                    container=graphs_panel,
                                                    visible=True,
                                                    manager=gui_manager)

    graph_2_background = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((window_size[0]/4-graph_border/2,graph_border/2), graph_size),
                                                    container=graphs_panel,
                                                    visible=True,
                                                    manager=gui_manager)

    graph_3_background = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((graph_border/2,(window_size[1]-banner_size)/2-graph_border/2),graph_size),
                                                    container=graphs_panel,
                                                    visible=True,
                                                    manager=gui_manager)

    graph_4_background = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((window_size[0]/4-graph_border/2,(window_size[1]-banner_size)/2-graph_border/2),graph_size),
                                                    container=graphs_panel,
                                                    visible=True,
                                                    manager=gui_manager)
    console_log = pygame_gui.elements.UITextBox(relative_rect=pygame.Rect((graph_border, graphs_panel_height + graph_border + banner_size - log_size ), (graphs_panel_width, log_size)),
                                                html_text="null",
                                                manager=gui_manager)
    # input_display = pygame_gui.elements.UILabel(relative_rect=pygame.Rect((0, 0), (100, 100)),
    #                                            manager=gui_manager)

setup_gui()


while running:
    sleep(0.1)
    time_delta = clock.tick(60)/1000.0
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN and pygame.key.get_pressed()[27]:
            running = False
        elif event.type == pygame.JOYAXISMOTION:
            # apply input to variables TODO
            running = True
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
    
    # check that joystick input is detected
    

    screen.fill("blue")
    gui_manager.draw_ui(screen)
    pygame.display.update()


rclpy.shutdown()
