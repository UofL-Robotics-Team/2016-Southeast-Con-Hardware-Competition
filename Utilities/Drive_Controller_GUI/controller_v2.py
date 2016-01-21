# -*- coding: utf-8 -*-
from Tkinter import *
import sys
import serial

# Define global reference variables
connection_established = False

# Lists all available COM ports
def list_ports():
    if sys.platform.startswith('win'):
        ports = ['COM' + str(i + 1) for i in range(256)]

    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this is to exclude your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')

    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')

    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

# Does nothing
def donothing():
    return

# Adds direction buttons
def add_controls():
    controller_frame = Frame(root)
    controller_frame.pack(side=LEFT)

    controller_top_frame = Frame(controller_frame)
    controller_top_frame.pack()

    nw_btn = Button(controller_top_frame, text="↖", font=("Arial, 24"), height=1, width=4, padx=10, pady=10)
    nw_btn.pack(side=LEFT)
    n_btn = Button(controller_top_frame, text="↑", font=("Arial, 24"), height=1, width=4, padx=10, pady=10)
    n_btn.pack(side=LEFT)
    ne_btn = Button(controller_top_frame, text="↗", font=("Arial, 24"), height=1, width=4, padx=10, pady=10)
    ne_btn.pack(side=LEFT)

    controller_middle_frame = Frame(controller_frame)
    controller_middle_frame.pack()

    w_btn = Button(controller_middle_frame, text="←", font=("Arial, 24"), height=1, width=4, padx=10, pady=10)
    w_btn.pack(side=LEFT)
    stop_btn = Button(controller_middle_frame, text="STOP", font=("Arial, 24"), height=1, width=4, padx=10, pady=10)
    stop_btn.pack(side=LEFT)
    e_btn = Button(controller_middle_frame, text="→", font=("Arial, 24"), height=1, width=4, padx=10, pady=10)
    e_btn.pack(side=LEFT)

    controller_bottom_frame = Frame(controller_frame)
    controller_bottom_frame.pack()

    sw_btn = Button(controller_bottom_frame, text="↙", font=("Arial, 24"), height=1, width=4, padx=10, pady=10)
    sw_btn.pack(side=LEFT)
    s_btn = Button(controller_bottom_frame, text="↓", font=("Arial, 24"), height=1, width=4, padx=10, pady=10)
    s_btn.pack(side=LEFT)
    se_btn = Button(controller_bottom_frame, text="↘", font=("Arial, 24"), height=1, width=4, padx=10, pady=10)
    se_btn.pack(side=LEFT)

    controller_rot_frame = Frame(controller_frame)
    controller_rot_frame.pack()

    r_ccw_btn = Button(controller_rot_frame, text="↺", font=("Arial, 24"), height=1, width=4, padx=36, pady=5)
    r_ccw_btn.pack(side=LEFT)
    r_cw_btn = Button(controller_rot_frame, text="↻", font=("Arial, 24"), height=1, width=4, padx=36, pady=5)
    r_cw_btn.pack(side=LEFT)

# Connects to the passed in serial port
def connect_serial(port):
    try:
        port.set()
        ser = serial.Serial(port, baudrate=9600, timeout=5)
    except serial.serialutil.SerialException:
        print "Could not connect to serial port."
    connection_established = True
    return

# Define root window
root = Tk()
root.title("Drivetrain Controller")
root.resizable(width=FALSE, height=FALSE)

# Setup menu
menu_bar = Menu(root)

# Create file menu
file_menu = Menu(menu_bar, tearoff=0)
file_menu.add_command(label="Exit", command=root.quit())

# Create serial port menu
port_selected = BooleanVar()
port_selected.set(0)

port_menu = Menu(menu_bar, tearoff=0)
for port in list_ports():
    port_menu.add_checkbutton(label=port, variable=port_selected, onvalue=1, offvalue=0)

# Add cascades
menu_bar.add_cascade(label="File", menu=file_menu)
menu_bar.add_cascade(label="Port", menu=port_menu)

port = StringVar()
port.set("Disconnected")

connection_status = Label(root, textvar=port, bd=0, fg="RED", padx=40, pady=20)
connection_status.pack()

def update_window():
    if port_selected.get():
        connection_status.config(fg="GREEN")
        port.set("Connected to")
        add_controls()
        port_selected.set(0)
    root.after(100, update_window)

# Render window
root.config(menu=menu_bar)
root.after(0, update_window)
root.mainloop()
