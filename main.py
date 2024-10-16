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
console_size = 250
console_input_size = 25
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
screen = pygame.display.set_mode(window_size)
gui_manager = pygame_gui.UIManager(window_size, "theme.json")
clock = pygame.time.Clock()
#controller = None

#region GUI ELEMENTS
## CONTAINERS ##
graphs_panel = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((window_size[0]/2+graph_border, banner_size+graph_border), (graphs_panel_width, graphs_panel_height)), 
                                                                    manager=gui_manager)
claw_panel = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((graph_border, banner_size+graph_border), (graphs_panel_width, graphs_panel_height -console_size)), 
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
console_log = pygame_gui.elements.UITextBox(relative_rect=pygame.Rect((graph_border, graphs_panel_height + graph_border + banner_size - console_size ), (graphs_panel_width, console_size-console_input_size)),
                                            html_text="",
                                            manager=gui_manager)
console_input = pygame_gui.elements.UITextEntryLine(relative_rect=pygame.Rect((graph_border, graphs_panel_height + graph_border + banner_size-console_input_size), (graphs_panel_width, console_input_size)),
                                                    manager=gui_manager)
#endregion

def console_send():
    input = console_input.get_text()
    if input == "":
        return
    console_input.clear()
    console_input.focus()
    # Display the input
    console_print("> " + input)
    # Parse the input to find if there are any commands
    
    

def console_print(text):
    console_log.append_html_text(text + "<br>")

while running:
    sleep(0.1)
    time_delta = clock.tick(60)/1000.0
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        gui_manager.process_events(event)
        if event.type == pygame.JOYDEVICEADDED:
            controller = pygame.joystick.Joystick(event.device_index)
            console_print("Joystick connected")
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RETURN:
                console_send()
        
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
    #input_1[0] = controller.get_axis(0)
    #input_1[1] = controller.get_axis(1)

    screen.fill("blue")
    gui_manager.draw_ui(screen)
    pygame.display.update()


rclpy.shutdown()
