import time
import math
import random

import pygame_gui.elements.ui_label
import pygame_gui.elements.ui_text_box
from pygame_gui.core import ObjectID

import rclpy # type: ignore
rclpy.init()

import ros_send
import ros_receive

import ssh_in_libre

import pygame
import pygame_gui

pygame.display.set_caption("Bob, Did He? Rover Base Station")

#--GUI Variables--#
window_size = (1280, 720)
banner_size = 75 # This is in pixels
console_size = 250
console_input_size = 25
graph_y_border = 25
graph_range = 25 # This is sample size for the graphs
graph_data = [[0],[0],[0],[0],[0],[0]]
battery = 100

graphs_panel_width = window_size[0]/2
graphs_panel_height = window_size[1]-banner_size
graph_size = (graphs_panel_width/2, graphs_panel_height/2)
graph_origin = [(graphs_panel_width +graph_y_border, banner_size+(graph_size[1]/2)),
                (graphs_panel_width + graph_size[0]+graph_y_border, banner_size + graph_size[1]/2),
                (graphs_panel_width+graph_y_border, banner_size+(graph_size[1]*2) - graph_size[1]/2),
                (graphs_panel_width + graph_size[0]+graph_y_border, banner_size+(graph_size[1]*2))]

#--Debug Variables--#




def write_line_to_log_file(s):
    path="log_data.csv"
    file=open(path,"a")
    file.writelines(["\n",str(s)])
    file.close()

write_line_to_log_file("\nStarting recording on: "+time.ctime())
write_line_to_log_file("ACCELERATION_X,ACCELERATION_Y,ACCELERATION_Z,ALTITUDE,PRESSURE,TEMPERATURE")

#region IK CALCULATIONS
control_method="ik" # either "ik" or "direct" or "" for none

motor_angles=[0,0,0,0,0] #the first and the last aren't actually angles because they're controlling cd motors
IK_target_pos=[15,-10,0] # x, y, angle of claw

def clamp(num,min,max):
    if num<min:
        num=min
    if num>max:
        num=max
    return num

#lengths of arm segments for IK math
length1=6.173
length2=10
length3=4

def calculate_IK_angles():
    global IK_target_pos, length1, length2, length3

    tx=IK_target_pos[0]-math.cos(IK_target_pos[2])*length3
    ty=IK_target_pos[1]-math.sin(IK_target_pos[2])*length3
    angles=[0,0,0]

    if tx<0.1:
        IK_target_pos[0]+=1
        tx+=1

    dist_squared=(tx**2 + ty**2)


    angles[1]=math.pi-math.acos(clamp((length1**2+length2**2-dist_squared)/(2*length1*length2),-1,1)) #this is just the law of cosins salved for the angle
    angles[1]=clamp(angles[1],0,math.pi/180*70)
    
    a=math.atan2(tx,-ty)
    b=math.asin(clamp(math.sin(angles[1])/dist_squared**0.5*length2,-1,1)) #this is the law of sines salved for the angle
    
    angles[0]=a-math.pi/2-b
    angles[0]=clamp(angles[0],-math.pi/2,0)

    angles[2]=IK_target_pos[2]-angles[1]-angles[0]
    angles[2]=clamp(angles[2],-math.pi/2,math.pi/2)


    IK_target_pos[2]=clamp(IK_target_pos[2],angles[0]+angles[1]+angles[2]-0.1,angles[0]+angles[1]+angles[2]+0.1)




    #recalculate target position based on angles
    p=[0,0]
    p[0]=math.cos(angles[0])*length1+math.cos(angles[0]+angles[1])*length2+math.cos(angles[0]+angles[1]+angles[2])*length3
    p[1]=math.sin(angles[0])*length1+math.sin(angles[0]+angles[1])*length2+math.sin(angles[0]+angles[1]+angles[2])*length3



    min_dist=(p[0]**2+p[1]**2)**0.5*0.95

    max_dist=(p[0]**2+p[1]**2)**0.5*1.05

    if ((IK_target_pos[0]**2+IK_target_pos[1]**2)**0.5)>=max_dist:
        d=((IK_target_pos[0]**2+IK_target_pos[1]**2)**0.5)/max_dist
        IK_target_pos[0]/=d
        IK_target_pos[1]/=d
    if ((IK_target_pos[0]**2+IK_target_pos[1]**2)**0.5)<min_dist:
        d=min_dist/((IK_target_pos[0]**2+IK_target_pos[1]**2)**0.5)
        IK_target_pos[0]*=d
        IK_target_pos[1]*=d



    return angles

def set_target_pos():
    IK_target_pos[0]=math.cos(motor_angles[1])*length1+math.cos(motor_angles[1]+motor_angles[2])*length2+math.cos(motor_angles[1]+motor_angles[2]+motor_angles[3])*length3
    IK_target_pos[1]=math.sin(motor_angles[1])*length1+math.sin(motor_angles[1]+motor_angles[2])*length2+math.sin(motor_angles[1]+motor_angles[2]+motor_angles[3])*length3
    IK_target_pos[2]=motor_angles[1]+motor_angles[2]+motor_angles[3]


#endregion

def direct_motor_control():
    if controller:
        if abs(controller.get_axis(1))>0.07:
            motor_angles[1]+=controller.get_axis(1)/6
        if abs(controller.get_axis(4))>0.07:
            motor_angles[2]+=controller.get_axis(4)/6
        
        if controller.get_axis(2)>-0.95 or controller.get_axis(5)>-0.95:
            d=controller.get_axis(5)-controller.get_axis(2)
            motor_angles[3]+=d/6
    motor_angles[1]=clamp(motor_angles[1],-math.pi/2,0)
    motor_angles[2]=clamp(motor_angles[2],0,math.pi/180*70)
    motor_angles[3]=clamp(motor_angles[3],-math.pi/2,math.pi/2)
    

running=True
pygame.init()
screen = pygame.display.set_mode(window_size)
gui_manager = pygame_gui.UIManager(window_size, "theme.json")
clock = pygame.time.Clock()

pygame.joystick.init()
controller=False

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
                                              text="Acceleration",
                                              container=imu_graph_background,
                                              manager=gui_manager)

temp_graph_background = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((graphs_panel_width/2,0), graph_size),
                                                    container=graphs_panel,
                                                    visible=True,
                                                    manager=gui_manager)

temp_graph_label = pygame_gui.elements.UILabel(relative_rect=pygame.Rect(((graph_size[1]/2) - 125, 5), (250,25)),
                                              text="Temperature",
                                              container=temp_graph_background,
                                              manager=gui_manager)

pressure_graph_background = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((0,graphs_panel_height/2),graph_size),
                                                    container=graphs_panel,
                                                    visible=True,
                                                    manager=gui_manager)
pressure_graph_label = pygame_gui.elements.UILabel(relative_rect=pygame.Rect(((graph_size[1]/2) - 125, 5), (250,25)),
                                              text="Pressure",
                                              container=pressure_graph_background,
                                              manager=gui_manager)

battery_graph_background = pygame_gui.elements.UIPanel(relative_rect=pygame.Rect((graphs_panel_width/2,graphs_panel_height/2),graph_size),
                                                container=graphs_panel,
                                                visible=True,
                                                manager=gui_manager)
battery_graph_label = pygame_gui.elements.UILabel(relative_rect=pygame.Rect(((graph_size[1]/2) - 125, 5), (250,25)),
                                              text="Altitude",
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
    if args[0] == "clear":
        console_log.clear()
    elif args[0] == "send":
        m=input[input.find(" "):]
        ros_send.send_command(m)
        console_print("sent: "+m)
        # if len(args) != 6:
        #     console_print("Error: must send exactly 5 motor positions, separated by spaces.")
        # else:
        #     positions = [float(args[1]),float(args[2]),float(args[3]),float(args[4]),float(args[5])]
        #     ros_send.send_motor_positions(positions)
    elif args[0]=="exit":
        global running
        running=False
    elif args[0]=="control":
        global control_method
        control_method=args[1]
        if control_method=="ik":
            set_target_pos()
    elif args[0]=="reconnect":
        ssh_in_libre.restart()
    # command list for reference:
    #reconnect
    #control
    #send
    #send reboot
    #clear



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

ssh_in_libre.console_send=console_print

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

def graph_new_data(data):
    write_line_to_log_file(data)
    l=[]
    for i in data.split(","):
        l.append(float(i))
    x,y,z,altitude,pressure,temp=l

    update_graph(graph_data[0], x*3)
    update_graph(graph_data[1], y*3)
    update_graph(graph_data[2], z*3)
    update_graph(graph_data[3], temp)
    update_graph(graph_data[4], ((pressure/101300)-0.98437)*80000)
    update_graph(graph_data[5], (altitude-134)*30+75)

ros_receive.set_sensor_data_callback(graph_new_data)

update_graph(graph_data[0], 0)
update_graph(graph_data[1], 0)
update_graph(graph_data[2], 0)
update_graph(graph_data[3], 0)
update_graph(graph_data[4], 0)
update_graph(graph_data[0], 0)
update_graph(graph_data[1], 0)
update_graph(graph_data[2], 0)
update_graph(graph_data[3], 0)
update_graph(graph_data[4], 0)
update_graph(graph_data[5], 0)
update_graph(graph_data[5], 0)


def graph_offset(graph, origin):
    output = []
    for i in range(len(graph)):
        output.append(((i*((graph_size[1]-graph_y_border-2)/(graph_range-1))+origin[0]), origin[1] - graph[i]*2))
    return output
        
    
def draw_graphs():
    
    # Draw three graphs for the imu sensor
    pygame.draw.lines(screen, "red", False, graph_offset(graph_data[0], graph_origin[0]), 1)
    pygame.draw.lines(screen, "green", False, graph_offset(graph_data[1], graph_origin[0]), 1)
    pygame.draw.lines(screen, "blue", False, graph_offset(graph_data[2], graph_origin[0]), 1)
    
    pygame.draw.lines(screen, "red", False, graph_offset(graph_data[3], graph_origin[1]), 1)
    pygame.draw.lines(screen, "cyan", False, graph_offset(graph_data[4], graph_origin[2]), 1)
    pygame.draw.lines(screen, "magenta", False, graph_offset(graph_data[5], graph_origin[3]), 1)
    
    #draw arm position
    scale=10
    pos0=[40, (graphs_panel_height/2)-banner_size]
    pos1=pos0[:]
    pos1[0]+=length1*math.cos(motor_angles[1])*scale
    pos1[1]+=length1*math.sin(motor_angles[1])*scale
    pygame.draw.line(screen, "black", (pos0[0],pos0[1]), (pos1[0],pos1[1]), 5)

    pos2=pos1[:]
    pos2[0]+=length2*math.cos(motor_angles[1]+motor_angles[2])*scale
    pos2[1]+=length2*math.sin(motor_angles[1]+motor_angles[2])*scale
    pygame.draw.line(screen, "black", (pos1[0],pos1[1]), (pos2[0],pos2[1]), 5)
    
    pos3=pos2[:]
    pos3[0]+=length3*math.cos(motor_angles[1]+motor_angles[2]+motor_angles[3])*scale
    pos3[1]+=length3*math.sin(motor_angles[1]+motor_angles[2]+motor_angles[3])*scale
    pygame.draw.line(screen, "black", (pos2[0],pos2[1]), (pos3[0],pos3[1]), 5)

    pygame.draw.circle(screen, "black", (IK_target_pos[0]*scale+pos0[0],IK_target_pos[1]*scale+pos0[1]),4)

    
ros_receive.set_log_data_callback(console_print)


while running:
    time.sleep(0.1)
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
        if event.type == pygame.JOYDEVICEREMOVED:
            console_print("Joystick disconnected")
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RETURN:
                console_send()
        
    gui_manager.update(time_delta)

    screen.fill("navy blue")
    gui_manager.draw_ui(screen)

    draw_graphs()
    #displaying logs
    #displaying motor angles and stuff

    if controller:
        motor_angles[4]=controller.get_button(4)-controller.get_button(5)
        if controller.get_axis(0)**2>0.02:
            motor_angles[0]=controller.get_axis(0)
        else:
            motor_angles[0]=0
        
        if controller.get_button(0):
            control_method="ik"
            set_target_pos()
            console_print("ik")
        if controller.get_button(2):
            control_method="direct"
            console_print("direct")

    if control_method=="ik":
        # index 0 and index 4 are not set here (because they're for the cd motors)
        motor_angles[1],motor_angles[2],motor_angles[3]=calculate_IK_angles()
        if controller:
            
            if controller.get_axis(4)**2+controller.get_axis(1)**2>0.02:
                IK_target_pos[0]-=controller.get_axis(1)
                IK_target_pos[1]+=controller.get_axis(4)/1.5
            
            if controller.get_axis(2)>-0.98 or controller.get_axis(5)>-0.98:
                d=controller.get_axis(2)-controller.get_axis(5)
                IK_target_pos[2]-=d/8
    
    if control_method=="direct":
        direct_motor_control()

    ros_send.send_motor_positions(motor_angles)
    
    pygame.display.update()


rclpy.shutdown()
time.sleep(0.1)
ssh_in_libre.kill() #this is only necessary to kill the ssh process, but it will kill everything