import sys, time, serial, matplotlib, random, select
# sudo apt-get install python3-matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque
import tkinter.ttk
from PIL import Image, ImageTk
import os
import numpy as np

import Tkinter as tk

matplotlib.style.use("seaborn")

states = { 0:"IDLE",
           1:"Prop loading",
           2:"Pressurized",
           3:"ignite",
           4:"Boost",
           5:"Coast",
           6:"Manual Abort",
           7:"Auto Abort",
           8:"Error",
           9:"count" }

curr_State = 0
#HAVE TO CHANGE THIS if you want to do the google earth thing still
# plan was to constantly update a KML file and use the google earth saved file set to reopen every second in order to create a live trajectory
google_earth_location = r"start C:\Users\croix\googleearth.lnk"

def init_gui():
    '''
    This function will set up the gui
    '''
    global root

    #pressure values
    global p1_box
    global p2_box
    global p3_box
    global hybrid_box

    #graph variables
    global plot_size
    global x_values
    global p1_values
    global p2_values
    global p3_values
    global fig
    global ax1
    global line1
    global line2
    global line3

    #hybrid state
    global curr_State
    global IDLE_label
    global PROP_label
    global PRESS_label
    global IGNITE_label
    global BOOST_label
    global COAST_label
    global hybrid_state_button
    global abort_button
    
    #global performance
    global vel_box
    global lat_box
    global lon_box
    global roll_box
    global accel_box
    global alt_box
    global google_earth_button

    #serial
    global serial_port
    global serial_status
    global serial_connection
    global sock

    serial_connection = False

    curr_State = 0
    print(curr_State)

    root = tk.Tk()
    root.geometry('+0+0')
    root.protocol("WM_DELETE_WINDOW", _quit)
    root.wm_title("IREC MISSION CONTROL")

    #--------------------------------------------------------------
    #pressure plot
    plot_size = 1000

    x_values = [i for i in range(plot_size)]
    p1_values = deque(np.zeros(plot_size))
    p2_values = deque(np.zeros(plot_size))
    p3_values = deque(np.zeros(plot_size))

    # plt.ion()
    fig = plt.figure(figsize=(4,4))
    ax1 = fig.add_subplot(111)
    ax1.set_title("Pressure Readings")
    ax1.set_xlim(0, plot_size)
    ax1.set_ylim(0, 8300)
    ax1.set_yticks([0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000])
    ax1.axhline(8192, color="r", linestyle="--", linewidth=0.7)
    line1, = ax1.plot(x_values, p1_values, 'r-', linewidth=0.6)
    line2, = ax1.plot(x_values, p2_values, 'g-', linewidth=0.6)
    line3, = ax1.plot(x_values, p3_values, 'b-', linewidth=0.6)
    #----------------------------------------------------------------

    primary_frame = tk.Frame(root)

    hybrid_frame = tk.Frame(primary_frame)
    
    #----------------------------------------------------------------
    #Hybrid checklist stuff
    hybrid_checklist = tk.Frame(hybrid_frame)


    IDLE_label = soc_label = tk.Label(hybrid_checklist, text="IDLE", font=("Comic Sans MS", 12, ), height=3, width=20, borderwidth=2)
    IDLE_label.grid(row = 1, column=1, columnspan = 1, pady=20)
    IDLE_label.config({"background" : "yellow"})

    PROP_label = soc_label = tk.Label(hybrid_checklist, text="Propellant loading", font=("Comic Sans MS", 12, ), height=3, width=20, borderwidth=2)
    PROP_label.grid(row = 2, column=1, columnspan = 1,pady=20)
    PROP_label.config({"background" : "red"})

    PRESS_label = soc_label = tk.Label(hybrid_checklist, text="Presurized", font=("Comic Sans MS", 12, ), height=3, width=20, borderwidth=2)
    PRESS_label.grid(row = 3, column=1, columnspan = 1,pady=20)
    PRESS_label.config({"background" : "red"})

    IGNITE_label = soc_label = tk.Label(hybrid_checklist, text="Ignite", font=("Comic Sans MS", 12, ), height=3, width=20, borderwidth=2)
    IGNITE_label.grid(row = 4, column=1, columnspan = 1,pady=20)
    IGNITE_label.config({"background" : "red"})

    BOOST_label = soc_label = tk.Label(hybrid_checklist, text="Boost", font=("Comic Sans MS", 12, ), height=3, width=20, borderwidth=2)
    BOOST_label.grid(row = 5, column=1, columnspan = 1,pady=20)
    BOOST_label.config({"background" : "red"})

    COAST_label = soc_label = tk.Label(hybrid_checklist, text="Coast", font=("Comic Sans MS", 12, ), height=3, width=20, borderwidth=2)
    COAST_label.grid(row = 6, column=1, columnspan = 1,pady=20)
    COAST_label.config({"background" : "red"})

    hybrid_state_button = tk.Button(hybrid_checklist, text="load prop", height=3, width = 20, command=lambda: state_toggle())
    hybrid_state_button.grid(row=7, column=1,pady=20)
    #-----------------------------------------------------------------------
    # Abort button
    abort_button = tk.Button(hybrid_frame, text="ABORT", height=3, width = 60, command=lambda: abort())
    abort_button.grid(row=7, column=1,columnspan=3,pady=20)
    abort_button.config({"background" : "dark red"})
    #-----------------------------------------------------------------------
    canvas = FigureCanvasTkAgg(fig, master=hybrid_frame)  # A tk.DrawingArea.
    canvas.draw()
    canvas.get_tk_widget().grid(row=5, column = 1, columnspan = 2)

    hybrid_label = soc_label = tk.Label(hybrid_frame, text="Hybrid", font=("Comic Sans MS", 20, "bold"))
    hybrid_label.grid(row=1, column=1, columnspan=1)

    hybrid_box = tk.Text(hybrid_frame, height = 1, width = 20)
    hybrid_box.insert(tk.END, (states[0]))
    hybrid_box.tag_add("center", 1.0, "end")
    hybrid_box.tag_configure("center", justify="center")
    hybrid_box.configure(font=("Comic Sans MS", 12))
    hybrid_box.grid(row = 1, column = 2, columnspan=1, padx = 10, pady = 2, sticky="w")

    p1_label = soc_label = tk.Label(hybrid_frame, text="Pressure 1", font=("Comic Sans MS", 12, ))
    p1_label.grid(row = 2, column=1, columnspan = 1, pady=2)

    p1_box = tk.Text(hybrid_frame, height = 1, width = 20)
    p1_box.insert(tk.END, ("0"))
    p1_box.tag_add("center", 1.0, "end")
    p1_box.tag_configure("center", justify="center")
    p1_box.configure(font=("Comic Sans MS", 12))
    p1_box.grid(row = 2, column = 2, columnspan=1, padx = 10, pady = 2, sticky="w")
    
    p2_label = soc_label = tk.Label(hybrid_frame, text="Pressure 2", font=("Comic Sans MS", 12, ))
    p2_label.grid(row = 3, column=1, columnspan = 1, pady=2)

    p2_box = tk.Text(hybrid_frame, height = 1, width = 20)
    p2_box.insert(tk.END, ("0"))
    p2_box.tag_add("center", 1.0, "end")
    p2_box.tag_configure("center", justify="center")
    p2_box.configure(font=("Comic Sans MS", 12))
    p2_box.grid(row = 3, column = 2, columnspan=1, padx = 10, pady = 2, sticky="w")

    p3_label = soc_label = tk.Label(hybrid_frame, text="Pressure 3", font=("Comic Sans MS", 12, ))
    p3_label.grid(row = 4, column=1, columnspan = 1,pady=2,)

    p3_box = tk.Text(hybrid_frame, height = 1, width = 20)
    p3_box.insert(tk.END, ("0"))
    p3_box.tag_add("center", 1.0, "end")
    p3_box.tag_configure("center", justify="center")
    p3_box.configure(font=("Comic Sans MS", 12))
    p3_box.grid(row = 4, column = 2, columnspan=1, padx = 10, pady = 2, sticky="w")

    hybrid_checklist.grid(row=1, column=3, rowspan=5, padx=30, sticky="e")
    hybrid_frame.grid(row=1, column=1, columnspan = 1,)

    #tkinter.ttk.Separator(primary_frame, orient=tk.VERTICAL).grid(column=1, row=0, rowspan=2, sticky='ns')
    #--------------------------------------------------------------------------
    '''
    performance stuff
    '''
    right_panel=tk.Frame(primary_frame)
    perf_frame = tk.Frame(right_panel)

    perf_label = soc_label = tk.Label(perf_frame, text="Performance", font=("Comic Sans MS", 20, "bold"))
    perf_label.grid(row=1, column=1, columnspan=1, sticky="n")

    #----------------------------------------------------------------------------------
    #GPS peformance
    coord_frame = tk.Frame(perf_frame)

    lat_label = soc_label = tk.Label(coord_frame, text="Lat:", font=("Comic Sans MS", 12, ), width=4, height=1)
    lat_label.grid(row = 1, column = 1, columnspan = 1,sticky="e")

    lat_box = tk.Text(coord_frame, height = 1, width = 10)
    lat_box.insert(tk.END, ("00.000N"))
    lat_box.tag_add("center", 1.0, "end")
    lat_box.tag_configure("center", justify="center")
    lat_box.configure(font=("Comic Sans MS", 12))
    lat_box.grid(row = 1, column = 2, columnspan=1, padx = 10, pady = 2, sticky="w")

    lon_label = soc_label = tk.Label(coord_frame, text="Lat:", font=("Comic Sans MS", 12, ), width=4, height=1)
    lon_label.grid(row = 1, column = 3, columnspan = 1, sticky="e")

    lon_box = tk.Text(coord_frame, height = 1, width = 10)
    lon_box.insert(tk.END, ("000.000E"))
    lon_box.tag_add("center", 1.0, "end")
    lon_box.tag_configure("center", justify="center")
    lon_box.configure(font=("Comic Sans MS", 12))
    lon_box.grid(row = 1, column = 4, columnspan=1, padx = 10, pady = 2, sticky="w")

    coord_frame.grid(row=2,column=1,pady=5)
    #-----------------------------------------------------------------------------------
    # altitude
    altitude_frame = tk.Frame(perf_frame)

    altitude_label = soc_label = tk.Label(altitude_frame, text="Altitude:", font=("Comic Sans MS", 12, ), width=9, height=1)
    altitude_label.grid(row = 1, column = 1, columnspan = 1,sticky="e")

    alt_box = tk.Text(altitude_frame, height = 1, width = 24)
    alt_box.insert(tk.END, ("0.0"))
    alt_box.tag_add("center", 1.0, "end")
    alt_box.tag_configure("center", justify="center")
    alt_box.configure(font=("Comic Sans MS", 12))
    alt_box.grid(row = 1, column = 2, columnspan=1, padx = 10, pady = 2, sticky="w")

    alt_unit_label = soc_label = tk.Label(altitude_frame, text="ft", font=("Comic Sans MS", 12, ), width=2, height=1)
    alt_unit_label.grid(row = 1, column = 3, columnspan = 1,sticky="w")

    altitude_frame.grid(row=3,column=1,pady=5)
    #--------------------------------------------------------------------------------------
    # velocity
    velocity_frame = tk.Frame(perf_frame)

    velocity_label = soc_label = tk.Label(velocity_frame, text="Velocity:", font=("Comic Sans MS", 12, ), width=9, height=1)
    velocity_label.grid(row = 1, column = 1, columnspan = 1,sticky="e")

    vel_box = tk.Text(velocity_frame, height = 1, width = 22)
    vel_box.insert(tk.END, ("0.0"))
    vel_box.tag_add("center", 1.0, "end")
    vel_box.tag_configure("center", justify="center")
    vel_box.configure(font=("Comic Sans MS", 12))
    vel_box.grid(row = 1, column = 2, columnspan=1, padx = 10, pady = 2, sticky="w")

    vel_unit_label = soc_label = tk.Label(velocity_frame, text="ft/s", font=("Comic Sans MS", 12, ), width=4, height=1)
    vel_unit_label.grid(row = 1, column = 3, columnspan = 1,sticky="w")

    velocity_frame.grid(row=4,column=1,pady=5)
    #--------------------------------------------------------------------------------------
    # acceleration
    accel_frame = tk.Frame(perf_frame)

    accel_label = soc_label = tk.Label(accel_frame, text="Acceleration:", font=("Comic Sans MS", 12, ), width=13, height=1)
    accel_label.grid(row = 1, column = 1, columnspan = 1,sticky="e")

    accel_box = tk.Text(accel_frame, height = 1, width = 16)
    accel_box.insert(tk.END, ("0.0"))
    accel_box.tag_add("center", 1.0, "end")
    accel_box.tag_configure("center", justify="center")
    accel_box.configure(font=("Comic Sans MS", 12))
    accel_box.grid(row = 1, column = 2, columnspan=1, padx = 10, pady = 2, sticky="w")

    accel_unit_label = soc_label = tk.Label(accel_frame, text="ft/s/s", font=("Comic Sans MS", 12, ), width=6, height=1)
    accel_unit_label.grid(row = 1, column = 3, columnspan = 1,sticky="w")

    accel_frame.grid(row=5,column=1,pady=5)
    #---------------------------------------------------------------------------------------
    # roll
    roll_frame = tk.Frame(perf_frame)

    roll_label = soc_label = tk.Label(roll_frame, text="Roll:", font=("Comic Sans MS", 12, ), width=9, height=1)
    roll_label.grid(row = 1, column = 1, columnspan = 1,sticky="e")

    roll_box = tk.Text(roll_frame, height = 1, width = 22)
    roll_box.insert(tk.END, ("0.0"))
    roll_box.tag_add("center", 1.0, "end")
    roll_box.tag_configure("center", justify="center")
    roll_box.configure(font=("Comic Sans MS", 12))
    roll_box.grid(row = 1, column = 2, columnspan=1, padx = 10, pady = 2, sticky="w")

    roll_unit_label = soc_label = tk.Label(roll_frame, text="deg/s", font=("Comic Sans MS", 12, ), width=6, height=1)
    roll_unit_label.grid(row = 1, column = 3, columnspan = 1,sticky="w")

    roll_frame.grid(row=6,column=1,pady=5)
    #----------------------------------------------------------------------------------------
    # ISS logo

    load = Image.open("groundPi/iss.png")
    load = load.resize((144,128))
    render = ImageTk.PhotoImage(load)

    root.iconphoto(False, render)

    img = tk.Label(primary_frame, image=render)
    img.image = render
    img.grid(row=1,column=2,sticky="se")
    #----------------------------------------------------------------------------------------
    # Google Earth button

    google_earth_button = tk.Button(perf_frame, text="Google Earth", height=2, width = 20, command=lambda: google_earth())
    google_earth_button.grid(row=7, column=1,pady=5)

    #-----------------------------------------------------------------------------------------
    # communication

    comm_frame = tk.Frame(right_panel)

    comm_label = soc_label = tk.Label(comm_frame, text="Communication Status", font=("Comic Sans MS", 20, ), width=20, height=1)
    comm_label.grid(row = 1, column = 1, columnspan = 2)

    #------------------------------------------------------------------------------------------
    # serial connection
    
    serial_frame = tk.Frame(comm_frame)

    serial_label = soc_label = tk.Label(comm_frame, text="Serial:", font=("Comic Sans MS", 12, ), width=7, height=1)
    serial_label.grid(row = 2, column = 1, columnspan = 1, stick='e')

    serial_port = tk.Entry(serial_frame, text="COM?", justify = 'center')
    #serial_port.insert(tk.END, ("COM?"))
    serial_port.configure(font=("Comic Sans MS", 12))
    serial_port.grid(row = 1, column = 1, columnspan=1, padx = 10, pady = 2, sticky="w")

    serial_status = soc_label = tk.Label(serial_frame, text="Not Connected", font=("Comic Sans MS", 12, ), width=15, height=1)
    serial_status.grid(row = 1, column = 2, columnspan = 1, stick='w', padx=5)
    serial_status.config({"background":"red", "foreground":"white"})

    serial_button = tk.Button(serial_frame, text="Connect", height=1, width = 25, command=lambda: send_serial_port())
    serial_button.grid(row=2, column=2,pady=5)

    tkinter.ttk.Separator(hybrid_frame, orient=tk.VERTICAL).grid(column=3, row=1, rowspan=7,sticky='nse')
    tkinter.ttk.Separator(perf_frame, orient=tk.HORIZONTAL).grid(column=1, row=8, columnspan=7,sticky='sew')
    right_panel.grid(row=1, column=2,sticky="n")
    perf_frame.grid(row=1,column=1, padx=5,sticky="n")
    comm_frame.grid(row=2, column=1, sticky="n")
    serial_frame.grid(row=2, column=2, sticky="w")
    primary_frame.pack()

def state_toggle():
    
    #these are states that are toggled using our button, unfinished

    global curr_State
    if curr_State == 0:
        curr_State = 1
        IDLE_label.config({"background" : "green"})
        PROP_label.config({"background" : "yellow"})
        hybrid_state_button.config(text="Do nothing")
    elif curr_State == 2:
        curr_State = 2
        PRESS_label.config({"background" : "green"})
        IGNITE_label.config({"background" : "yellow"})
        hybrid_state_button.grid_forget()
    else:
        pass

# end state_toggle

def abort():
    print("ABORT")

def google_earth():
    os.system(google_earth_location)

def send_serial_port():
    print("sending over the serial port")

# end def
def serial_connected( is_connected ):

    if not serial_connection:
        if is_connected:
            serial_status.config({"background" : "green"})
            serial_status.config(text="Connected")
        else:
            serial_status.config({"background" : "green"})
            serial_status.config(text="Not Connected")
        # end if
    # end if
# end def

def state_check(current_state):
    global curr_State

    if current_state == 0:
        curr_State = 1
        IDLE_label.config({"background" : "green"})
        PROP_label.config({"background" : "yellow"})
        hybrid_state_button.config(visible=False)
        hybrid_state_button.config(text="IGNITE!")

    elif current_state == 1:
        curr_State = 2
        PROP_label.config({"background" : "green"})
        PRESS_label.config({"background" : "yellow"})
        hybrid_state_button.config(visible=True)
        hybrid_state_button.config(text="IGNITE!")
    elif current_state == 2:
        curr_State = 2
        PROP_label.config({"background" : "green"})
        PRESS_label.config({"background" : "yellow"})
        hybrid_state_button.config(visible=True)
        hybrid_state_button.config(text="IGNITE!")
    elif current_state == 3:
        curr_State = 2
        PROP_label.config({"background" : "green"})
        PRESS_label.config({"background" : "yellow"})
        hybrid_state_button.config(visible=True)
        hybrid_state_button.config(text="IGNITE!")
    elif current_state == 4:
        curr_State = 2
        PROP_label.config({"background" : "green"})
        PRESS_label.config({"background" : "yellow"})
        hybrid_state_button.config(visible=True)
        hybrid_state_button.config(text="IGNITE!")
    elif current_state == 5:
        curr_State = 2
        PROP_label.config({"background" : "green"})
        PRESS_label.config({"background" : "yellow"})
        hybrid_state_button.config(visible=True)
        hybrid_state_button.config(text="IGNITE!")
    elif current_state == 6:
        curr_State = 2
        PROP_label.config({"background" : "green"})
        PRESS_label.config({"background" : "yellow"})
        hybrid_state_button.config(visible=True)
        hybrid_state_button.config(text="IGNITE!")
    elif current_state == 7:
        curr_State = 2
        PROP_label.config({"background" : "green"})
        PRESS_label.config({"background" : "yellow"})
        hybrid_state_button.config(visible=True)
        hybrid_state_button.config(text="IGNITE!")
    elif current_state == 8:
        curr_State = 2
        PROP_label.config({"background" : "green"})
        PRESS_label.config({"background" : "yellow"})
        hybrid_state_button.config(visible=True)
        hybrid_state_button.config(text="IGNITE!")
# end state_check

def update_plot(new_p1, new_p2, new_p3):
    '''
    This function will take in arguments and fill in the graph with those arguemtns
    '''
    p1_values.popleft()
    p2_values.popleft()
    p3_values.popleft()

    if new_p1 == None:
        p1_values.append(p1_values[len(p1_values)-1])
    else:
        p1_values.append(new_p1)

    if new_p2 == None:
        p2_values.append(p2_values[len(p2_values)-1])
    else:
        p2_values.append(new_p2)

    if new_p3 == None:
        p3_values.append(p3_values[len(p3_values)-1])
    else:
        p3_values.append(new_p3)

    line1.set_ydata(p1_values)
    line2.set_ydata(p2_values)
    line3.set_ydata(p3_values)

    fig.canvas.draw()

    fig.canvas.flush_events()

    p1_box.delete("1.0", tk.END)
    p1_box.insert(tk.END, str(new_p1))
    p1_box.tag_add("center", 1.0, "end")
    p2_box.delete("1.0", tk.END)
    p2_box.insert(tk.END, str(new_p2))
    p2_box.tag_add("center", 1.0, "end")
    p3_box.delete("1.0", tk.END)
    p3_box.insert("1.0", str(new_p3))
    p3_box.tag_add("center", 1.0, "end")

def update_values(lat, lon, alt, vel, accel, roll, state, serial):
    #global performance
    global vel_box
    global lat_box
    global lon_box
    global roll_box
    global accel_box
    global alt_box
    global google_earth_button

    #serial
    global serial_port
    global serial_status
    global serial_connection
    global sock

    vel_box.delete("1.0", tk.END)
    lat_box.delete("1.0", tk.END)
    lon_box.delete("1.0", tk.END)
    alt_box.delete("1.0", tk.END)
    accel_box.delete("1.0", tk.END)
    roll_box.delete("1.0", tk.END)
    vel_box.insert( tk.END, (str(vel)) )
    lat_box.insert( tk.END, (str(lat)) )
    lon_box.insert( tk.END, (str(lon)) )
    alt_box.insert( tk.END, (str(alt)) )
    accel_box.insert( tk.END, (str(accel)) )
    roll_box.insert( tk.END, (str(roll)) )

    if serial:
        serial_status.config({"background" : "green"})
        serial_status.config(text="Connected")
    else:
        serial_status.config({"background" : "green"})
        serial_status.config(text="Not Connected")

    #NOTHING IS IMPLEMENTED WITH THE STATE CURRENTLY
    state = None
# end def

def _quit():
    '''
    This function will exit the gui
    '''
    root.quit()
    root.destroy()
    sys.exit()

def gui_loop( ):
    '''
    This function will contain the loop that makes the gui work
    '''
    while True:
        try:
            '''
            data = str(ser.readline()[:-2].decode("utf-8"))
            ser.flush()
            '''
            # if data:
            #     tStamp, val1, val2, val3 = data.split("\t")

            #     update_plot(int(val1), int(val2), int(val3))
                
            '''
            ser.flush()
            '''

            time.sleep(.01)
            #there is a change root_update is slow, there are ways to not use it, but it is here.
            root.update()
        except KeyboardInterrupt:
            print("A")
            break
            '''
            ser.flush()
            ser.write("stahp".encode("utf-8"))
            '''
        except ValueError:
            #ser.flush()
            print("B")

if __name__ == "__main__":
    init_gui()

    try:
        gui_loop()
    except KeyboardInterrupt:
        sys.exit()
