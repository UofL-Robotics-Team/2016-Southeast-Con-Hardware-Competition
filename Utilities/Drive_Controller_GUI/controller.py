# -*- coding: utf-8 -*-
from Tkinter import *
import serial

# Define root window
root = Tk()
root.title("Drivetrain Controller")
root.resizable(width=FALSE, height=FALSE)

# Define movement settings
forward_velocity = 0
right_velocity = 0
clockwise_velocity = 0
microstep_setting = IntVar()
speed_increment = IntVar()


# Add direction commands
def stop_movement():
    global forward_velocity, right_velocity, clockwise_velocity
    forward_velocity = 0
    right_velocity = 0
    clockwise_velocity = 0
    send_movement(forward_velocity, right_velocity, clockwise_velocity)


def move_northwest():
    global forward_velocity, right_velocity, clockwise_velocity
    forward_velocity += speed_increment.get()
    right_velocity -= speed_increment.get()
    send_movement(forward_velocity, right_velocity, clockwise_velocity)


def move_north():
    global forward_velocity, right_velocity, clockwise_velocity
    forward_velocity += speed_increment.get()
    send_movement(forward_velocity, right_velocity, clockwise_velocity)


def move_northeast():
    global forward_velocity, right_velocity, clockwise_velocity
    forward_velocity += speed_increment.get()
    right_velocity += speed_increment.get()
    send_movement(forward_velocity, right_velocity, clockwise_velocity)


def move_west():
    global forward_velocity, right_velocity, clockwise_velocity
    stop_movement()
    right_velocity -= speed_increment.get()
    send_movement(forward_velocity, right_velocity, clockwise_velocity)


def move_east():
    global forward_velocity, right_velocity, clockwise_velocity
    right_velocity += speed_increment.get()
    send_movement(forward_velocity, right_velocity, clockwise_velocity)


def move_southwest():
    global forward_velocity, right_velocity, clockwise_velocity
    forward_velocity -= speed_increment.get()
    right_velocity -= speed_increment.get()
    send_movement(forward_velocity, right_velocity, clockwise_velocity)


def move_south():
    global forward_velocity, right_velocity, clockwise_velocity
    forward_velocity -= speed_increment.get()
    send_movement(forward_velocity, right_velocity, clockwise_velocity)


def move_southeast():
    global forward_velocity, right_velocity, clockwise_velocity
    forward_velocity -= speed_increment.get()
    right_velocity += speed_increment.get()
    send_movement(forward_velocity, right_velocity, clockwise_velocity)


def move_ccw():
    global forward_velocity, right_velocity, clockwise_velocity
    clockwise_velocity -= speed_increment.get()
    send_movement(forward_velocity, right_velocity, clockwise_velocity)


def move_cw():
    global forward_velocity, right_velocity, clockwise_velocity
    clockwise_velocity += speed_increment.get()
    send_movement(forward_velocity, right_velocity, clockwise_velocity)


# Adds direction buttons
def add_direction_controls():
    controller_frame = Frame(root)
    controller_frame.pack(side=LEFT)

    controller_top_frame = Frame(controller_frame)
    controller_top_frame.pack()

    nw_btn = Button(controller_top_frame, command=move_northwest, text="↖", font=("Arial, 24"), height=1, width=4,
                    padx=10, pady=10)
    nw_btn.pack(side=LEFT)
    n_btn = Button(controller_top_frame, command=move_north, text="↑", font=("Arial, 24"), height=1, width=4, padx=10,
                   pady=10)
    n_btn.pack(side=LEFT)
    ne_btn = Button(controller_top_frame, command=move_northeast, text="↗", font=("Arial, 24"), height=1, width=4,
                    padx=10, pady=10)
    ne_btn.pack(side=LEFT)

    controller_middle_frame = Frame(controller_frame)
    controller_middle_frame.pack()

    w_btn = Button(controller_middle_frame, command=move_west, text="←", font=("Arial, 24"), height=1, width=4, padx=10,
                   pady=10)
    w_btn.pack(side=LEFT)
    stop_btn = Button(controller_middle_frame, command=stop_movement, text="STOP", font=("Arial, 24"), height=1,
                      width=4, padx=10, pady=10)
    stop_btn.pack(side=LEFT)
    e_btn = Button(controller_middle_frame, command=move_east, text="→", font=("Arial, 24"), height=1, width=4, padx=10,
                   pady=10)
    e_btn.pack(side=LEFT)

    controller_bottom_frame = Frame(controller_frame)
    controller_bottom_frame.pack()

    sw_btn = Button(controller_bottom_frame, command=move_southwest, text="↙", font=("Arial, 24"), height=1, width=4,
                    padx=10, pady=10)
    sw_btn.pack(side=LEFT)
    s_btn = Button(controller_bottom_frame, command=move_south, text="↓", font=("Arial, 24"), height=1, width=4,
                   padx=10, pady=10)
    s_btn.pack(side=LEFT)
    se_btn = Button(controller_bottom_frame, command=move_southeast, text="↘", font=("Arial, 24"), height=1, width=4,
                    padx=10, pady=10)
    se_btn.pack(side=LEFT)

    controller_rot_frame = Frame(controller_frame)
    controller_rot_frame.pack()

    r_ccw_btn = Button(controller_rot_frame, command=move_ccw, text="↺", font=("Arial, 24"), height=1, width=4, padx=36,
                       pady=5)
    r_ccw_btn.pack(side=LEFT)
    r_cw_btn = Button(controller_rot_frame, command=move_cw, text="↻", font=("Arial, 24"), height=1, width=4, padx=36,
                      pady=5)
    r_cw_btn.pack(side=LEFT)

    # Adds direction buttons


# Adds speed increment slider
def add_speed_control():
    speed_control_frame = Frame(root)
    speed_control_frame.pack(side=LEFT)
    speed_increment_scale = Scale(speed_control_frame, variable=speed_increment, from_=0, to=1000, resolution=10,
                                  length=300, showvalue=True)
    speed_increment_scale.pack(anchor=CENTER)


# Adds microstepping selector
def add_microstep_selection():
    microstep_selection_frame = Frame(root)
    microstep_selection_frame.pack(side=LEFT)
    one = Radiobutton(microstep_selection_frame, text="1x", var=microstep_setting, value=1)
    one.pack()
    two = Radiobutton(microstep_selection_frame, text="2x", var=microstep_setting, value=2)
    two.pack()
    four = Radiobutton(microstep_selection_frame, text="4x", var=microstep_setting, value=4)
    four.pack()
    eight = Radiobutton(microstep_selection_frame, text="8x", var=microstep_setting, value=8)
    eight.pack()
    sixteen = Radiobutton(microstep_selection_frame, text="16x", var=microstep_setting, value=16)
    sixteen.pack()


# Connect to serial port
try:
    ser = serial.Serial("COM6", baudrate=9600, timeout=5)
except serial.SerialException:
    print "Could not connect to serial port."


# Sends the movement command
def send_movement(forward_velocity, right_velocity, clockwise_velocity):
    try:
        data = "f" + str(forward_velocity) + " r" + str(right_velocity) + " c" + str(clockwise_velocity) + " m" + str(
            microstep_setting.get()) + " "
        print data
        ser.write(data + '\n')
    except serial.SerialException:
        print "Couldn't send command."


# Render direction controls
add_direction_controls()
add_speed_control()
add_microstep_selection()

# Render window
root.mainloop()
