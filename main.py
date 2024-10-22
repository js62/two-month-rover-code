from time import sleep #tmp
import math
import random

import pygame_gui.elements.ui_label
import pygame_gui.elements.ui_text_box
from pygame_gui.core import ObjectID

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

#--GUI Variables--#
window_size = (1280, 720)
banner_size = 75 # This is in pixels
console_size = 250
console_input_size = 25
graph_y_border = 25
graph_range = 25 # This is sample size for the graphs
graph_data = [[0],[0],[0],[0]]
battery = 100

graphs_panel_width = window_size[0]/2
graphs_panel_height = window_size[1]-banner_size
graph_size = (graphs_panel_width/2, graphs_panel_height/2)
graph_origin = [(graphs_panel_width +graph_y_border, banner_size+(graph_size[1]/2)),
                (graphs_panel_width + graph_size[0]+graph_y_border, banner_size + graph_size[1]/2),
                (graphs_panel_width+graph_y_border, banner_size+(graph_size[1]*2) - graph_size[1]/2),
                (graphs_panel_width + graph_size[0]+graph_y_border, banner_size+(graph_size[1]*2))]

#--Debug Variables--#
input_1 = [0,0] #This is just for the left stick

control_method="" # either "ik" or "direct" or "" for none

#region IK CALCULATIONS
motor_angles=[0,0,0,0,0] #the first and the last aren't actually angles because they're controlling cd motors
IK_target_pos=[0,0,0]


#lengths of arm segments for IK math
length1=6.173
length2=10

def calculate_IK_angles(x,y,z):
    angles=[0,0,0]

    dist_squared=(x**2 + y**2 + z**2)

    if dist_squared**0.5>=length1+length2-0.1:
        d=dist_squared**0.5/(length1+length2-0.1)
        x/=d
        y/=d
        z/=d
        dist_squared=(x**2 + y**2 + z**2)
    if dist_squared**0.5<=abs(length1-length2)+0.1:
        d=dist_squared**0.5/(abs(length1-length2)+0.1)
        x/=d
        y/=d
        z/=d
        dist_squared=(x**2 + y**2 + z**2)

    angles[0]=math.pi/2+math.atan2(y,-x)
    y=(x**2+y**2)**0.5

    angles[2]=math.pi-math.acos((length1**2+length2**2-dist_squared)/(2*length1*length2)) #this is just the law of cosins salved for the angle
    
    a=math.atan2(z,y)
    b=math.asin(math.sin(angles[2])/dist_squared**0.5*length2) #this is the law of sines salved for the angle
    
    angles[1]=a-b+math.pi/2
    
    return angles
#endregion

# def calculate direct motor control

running=True
pygame.init()
screen = pygame.display.set_mode(window_size)
gui_manager = pygame_gui.UIManager(window_size, "theme.json")
clock = pygame.time.Clock()

pygame.joystick.init()
# pygame.joystick.Joystick(0)

#region GUI ELEMENTS
## CONTAINERS ##
graphs_panel = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((graphs_panel_width, banner_size), (graphs_panel_width, graphs_panel_height)), 
                                                                    manager=gui_manager)
claw_panel = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((0, banner_size), (graphs_panel_width, graphs_panel_height -console_size)), 
                                                                    manager=gui_manager)

## GUI ELEMENTS ##
imu_graph_background = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((0,0), graph_size),
                                                    container=graphs_panel,
                                                    visible=True,
                                                    manager=gui_manager)

imu_graph_label = pygame_gui.elements.UILabel(relative_rect=pygame.Rect(((graph_size[1]/2) - 125, 5), (250,25)),
                                              text="IMU Sensor",
                                              container=imu_graph_background,
                                              manager=gui_manager)

temp_graph_background = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((graphs_panel_width/2,0), graph_size),
                                                    container=graphs_panel,
                                                    visible=True,
                                                    manager=gui_manager)

temp_graph_label = pygame_gui.elements.UILabel(relative_rect=pygame.Rect(((graph_size[1]/2) - 125, 5), (250,25)),
                                              text="Temp Sensor",
                                              container=temp_graph_background,
                                              manager=gui_manager)

pressure_graph_background = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((0,graphs_panel_height/2),graph_size),
                                                    container=graphs_panel,
                                                    visible=True,
                                                    manager=gui_manager)
pressure_graph_label = pygame_gui.elements.UILabel(relative_rect=pygame.Rect(((graph_size[1]/2) - 125, 5), (250,25)),
                                              text="Pressure Sensor",
                                              container=pressure_graph_background,
                                              manager=gui_manager)

battery_graph_background = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((graphs_panel_width/2,graphs_panel_height/2),graph_size),
                                                container=graphs_panel,
                                                visible=True,
                                                manager=gui_manager)
battery_graph_label = pygame_gui.elements.UILabel(relative_rect=pygame.Rect(((graph_size[1]/2) - 125, 5), (250,25)),
                                              text="Battery",
                                              container=battery_graph_background,
                                              manager=gui_manager)

console_log = pygame_gui.elements.UITextBox(relative_rect=pygame.Rect((0, graphs_panel_height + banner_size - console_size ), (graphs_panel_width, console_size-console_input_size)),
                                            html_text="",
                                            manager=gui_manager)
console_input = pygame_gui.elements.UITextEntryLine(relative_rect=pygame.Rect((0, graphs_panel_height + banner_size-console_input_size), (graphs_panel_width, console_input_size)),
                                                    manager=gui_manager)

banner_title = pygame_gui.elements.UILabel(relative_rect=pygame.Rect(((0, 0), (window_size[0],banner_size))),
                                           text="Bob, Did He? Base Station :D",
                                           object_id=ObjectID(class_id="@title_label"),
                                           manager=gui_manager)
    
#endregion

def console_send():
    global running
    input = console_input.get_text()
    if input == "":
        return
    console_input.clear()
    console_input.focus()
    # Display the input
    console_print("> " + input)
    # Parse the input to find if there are any commands
    # separate string into array of arguments
    args = parse_input(input)
    if args[0] == "send":
        if len(args) != 6:
            console_print("Error: must send exactly 5 motor positions, separated by spaces.")
        else:
            positions = [float(args[1]),float(args[2]),float(args[3]),float(args[4]),float(args[5])]
            ros_send.send_motor_positions(positions)
    elif args[0]=="exit":
        running=False



def parse_input(input):
    args = []
    arg = ""
    for c in input:
        if c == ' ':
            args.append(arg)
            arg = ""
        else:
            arg += c
    if (arg != "") & (arg != " "):
        args.append(arg)
    return args



def console_print(text):
    console_log.append_html_text(text + "<br>")

def update_graph(graph_values, new_data):
    new_values = []
    i = 0
    if (len(graph_values) == graph_range): i = 1
    
    while i < graph_range:
        if i > len(graph_values)-1:
            break
        new_values.append(graph_values[i])
        i += 1
    new_values.append(new_data)
    graph_values[:] = new_values
    
def graph_offset(graph, origin):
    output = []
    for i in range(len(graph)):
        output.append(((i*((graph_size[1]-graph_y_border-2)/(graph_range-1))+origin[0]), origin[1] - graph[i]*2))
    return output
        
    
def draw_graphs():
    
    # Draw three graphs for the imu sensor
    pygame.draw.lines(screen, "green", False, graph_offset(graph_data[0], graph_origin[0]), 1)
    pygame.draw.lines(screen, "green", False, graph_offset(graph_data[0], graph_origin[0]), 1)
    pygame.draw.lines(screen, "green", False, graph_offset(graph_data[0], graph_origin[0]), 1)
    
    pygame.draw.lines(screen, "red", False, graph_offset(graph_data[1], graph_origin[1]), 1)
    pygame.draw.lines(screen, "cyan", False, graph_offset(graph_data[2], graph_origin[2]), 1)
    pygame.draw.lines(screen, "magenta", False, graph_offset(graph_data[3], graph_origin[3]), 1)
    
    
ros_receive.set_log_data_callback(console_print)
ros_send.set_send_positions_callback(console_print)


while running:
    sleep(0.1)
    time_delta = clock.tick(60)/1000.0
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN and pygame.key.get_pressed()[27]:
            running = False
        gui_manager.process_events(event)
        if event.type == pygame.JOYDEVICEADDED:
            controller = pygame.joystick.Joystick(event.device_index)
            console_print("Joystick connected")
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RETURN:
                console_send()
        
    gui_manager.update(time_delta)

    screen.fill("navy blue")
    gui_manager.draw_ui(screen)
    ## JUST FOR TESTING
    battery -= time_delta
    update_graph(graph_data[0], random.randint(-60, 40))
    update_graph(graph_data[1], random.randint(-40, 40))
    update_graph(graph_data[2], random.randrange(-40, 40))
    update_graph(graph_data[3], battery)
    draw_graphs()
    #displaying logs
    #displaying motor angles and stuff
    
    if control_method=="ik":
        # index 0 and index 4 are not set here (because they're for the cd motors)
        motor_angles[1],motor_angles[2],motor_angles[3]=calculate_IK_angles(IK_target_pos[0],IK_target_pos[1],IK_target_pos[2])
    
    # if control_method=="direct":
    #     direct_motor_control()
    # clamp_andgles() #May not be necessary since this is done on the pico
    
    # check that joystick input is detected
    #input_1[0] = controller.get_axis(0)
    #input_1[1] = controller.get_axis(1)
    
    pygame.display.update()


rclpy.shutdown()
