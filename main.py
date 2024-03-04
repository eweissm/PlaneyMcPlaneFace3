import serial
import tkinter as tk
import time
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg)
import numpy as np
import re

FileName = r"C:\Users\Ericw\Desktop\topSecretedata.csv"

# calculate the angles using inverse kinematics
theta0 = [0, 0]
prevTheta1 = 0

# set up serial comms---------------------------------------------------------------------------------------------------
ser = serial.Serial('com3', 9600, timeout=10) # create Serial Object, baud = 9600, read times out after 10s
time.sleep(3)  # delay 3 seconds to allow serial com to get established


# generate and plot the graph
def plot(x_coord, y_coord):
    global pathX
    global pathY

    # the figure that will contain the plot
    fig = Figure(figsize=(8, 8), dpi=100)

    # adding the subplot
    plot1 = fig.add_subplot(111)

    # set limits for graphs
    plot1.set_xlim([0, 300])
    plot1.set_ylim([0, 300])
    plot1.grid()

    # plotting path
    plot1.plot(pathX, pathY, color='blue', linestyle='dashed')

    # plotting the arm
    plot1.plot(0, 0, marker="o", markersize=20)
    plot1.plot(x_coord, y_coord, marker="o", markersize=10)

    # creating the Tkinter canvas
    # containing the Matplotlib figure
    canvas = FigureCanvasTkAgg(fig, master=RightFrame)
    canvas.draw()

    # placing the canvas on the Tkinter window
    canvas.get_tk_widget().place(relx=0, rely=0)


def GcodeConverter(fileName):
    x = []
    y = []

    with open(fileName, "r", encoding='utf-8-sig') as f:
        lines = f.read()
        lines2 = lines.splitlines()

        for line in lines2:
            match = re.search(r"X(\S*) Y(\S*)|X(\S*)|  Y(\S*)", line)

            if match != None:  # if we find a match
                result = match.groups()

                if result[0] != None:
                    x_temp = result[0]
                    y_temp = (result[1])

                    x.append(float(x_temp))
                    y.append(float(y_temp))
                elif result[2] != None:
                    x_temp = result[2]
                    y_temp = y[len(y) - 1]  # get last input

                    x.append(float(x_temp))
                    y.append(float(y_temp))
                elif result[3] != None:
                    y_temp = result[3]
                    x_temp = x[len(x) - 1]  # get last input

                    x.append(float(x_temp))
                    y.append(float(y_temp))

    return x, y


def startupPlot():
    global pathX
    global pathY

    # the figure that will contain the plot
    fig = Figure(figsize=(8, 8), dpi=100)

    # adding the subplot
    plot1 = fig.add_subplot(111)

    # set limits for graphs
    plot1.set_xlim([0, 300])
    plot1.set_ylim([0, 300])
    plot1.grid()

    # plotting path
    plot1.plot(pathX, pathY, color='blue', linestyle='dashed')

    # creating the Tkinter canvas
    # containing the Matplotlib figure
    canvas = FigureCanvasTkAgg(fig, master=RightFrame)
    canvas.draw()

    # placing the canvas on the Tkinter window
    canvas.get_tk_widget().place(relx=0, rely=0)

def generate_semicircle(center_x, center_y, radius, stepsize=0.1):
    """
    generates coordinates for a semicircle, centered at center_x, center_y
    """
    x = np.arange(center_x, center_x+radius+stepsize, stepsize)

    y = np.sqrt(abs(radius**2 - x**2))

    # since each x value has two corresponding y-values, duplicate x-axis.
    # [::-1] is required to have the correct order of elements for plt.plot.
    x = np.concatenate([x,x[::-1]])

    # concatenate y and flipped y.
    y = np.concatenate([y,-y[::-1]])

    return x, y + center_y


def StartPathFollow():
    global pathX
    global pathY

    set_coordinates_state(pathX, pathY)

prev_X_and_Y = [0,0]
# when update button is pressed--> take entered coordinates and calculate new coordinates, then update graph, then send
# to serial
def set_coordinates_state(x_coord, y_coord):
    global prev_X_and_Y  # Previous X and Y coordinates

    XCoords = []
    YCoords = []

    try:
        NumEntries = len(x_coord)
    # handle list / array case
    except TypeError:  # oops, was a float
        NumEntries = 1

    ser.reset_input_buffer()  # clear input buffer

    for i in range(NumEntries):
        if NumEntries > 1:
            XCoords.append(x_coord[i])
            YCoords.append(y_coord[i])

        else:
            XCoords.append(x_coord)
            YCoords.append(y_coord)


    plot(XCoords[-1], YCoords[-1])

    # Run through the angles

    for i in range(NumEntries):
        start = time.time()

        # send serial data to arduino
        ser.write(bytes(str(XCoords[i]), 'UTF-8'))
        ser.write(bytes('A', 'UTF-8'))
        ser.write(bytes(str(YCoords[i]), 'UTF-8'))
        ser.write(bytes('B', 'UTF-8'))

        # get expected move time from arduino
        ExpectedTime_bytes = ser.readline()
        ExpectedTime_string = ExpectedTime_bytes.decode("utf-8")
        print("ExpectedTime: " + ExpectedTime_string)

        try:
            # convert expected time to float (minimum time is 0.005s)
            ExpectedTime = max(float(ExpectedTime_string), 0.005)
        except ValueError:
            ExpectedTime = 0.1

        ser.reset_input_buffer()  # clear input buffer

        # if we get a 'y' from arduino, we move on, otherwise we will wait 0.5 sec. We will repeat this 5 times.
        # After which, if we still do not have confirmation, we will print to the monitor that there was a problem
        # and move on

        DidMoveWork = False

        ArduinoMessage = ''

        MoveStartTime = time.time()

        while time.time()-MoveStartTime < ExpectedTime*4 and not DidMoveWork:
            if ser.inWaiting():
                ArduinoMessage = ser.read(1)  # read one bit from buffer
                #print(ArduinoMessage)

            if ArduinoMessage == b'y':
                DidMoveWork = True
                print("Move was successful")

        if not DidMoveWork:
            print("Move was not successful")

        ser.reset_input_buffer()  # clear input buffer
        end = time.time()
        print("Difference between expected time and actual time: " + str(end - start - ExpectedTime))

# set path defaults
ActivePath = 0
pathX = [100, 200, 200, 100, 100]
pathY = [100, 100, 200, 200, 100]

def ChangeSelectPathButton():
    global ActivePath
    global pathX
    global pathY

    numCases = 8

    if ActivePath >= numCases-1:
        ActivePath=0
    else:
        ActivePath=ActivePath+1

    if ActivePath==0:
         # rectangle
         pathX = [100, 200, 200, 100, 100]
         pathY = [100, 100, 200, 200, 100]
    elif ActivePath==1:
        u = np.linspace(0, 6.5 * np.pi, 150)
        c = 4
        pathX = (c * (np.cos(u) + u * np.sin(u)))+150
        pathY = (c * (np.sin(u) - u * np.cos(u)))+150
    elif ActivePath == 2:

        u = np.linspace(0,  2 * np.pi, 100)
        c = 2
        pathX = (2*c*np.sin(u))**3 + 150
        pathY = (15*c*np.cos(u)-5*c*np.cos(2*u)-2*c*np.cos(3*u)-c*np.cos(4*u))*2 + 150

    elif ActivePath == 3:  # lemniscate
        u = np.linspace(0, 2 * np.pi, 50)
        c = 100
        pathX = (c * np.cos(u)) +150
        pathY = c * np.sin(2 * u) +150
    elif ActivePath ==4:
     # Lisajous curves
        u = np.linspace(0, 15 * np.pi, 400)
        pathX = 8*(8 * np.sin(u*.9)) +150
        pathY = 8*8 * np.sin(u) +150
    elif ActivePath == 5:

        u = np.linspace(0, 2 * np.pi, 400)
        pathX = 80 * np.sin(7*u)+150
        pathY = 80 * np.cos(5*u) +150

    elif ActivePath == 6: #circle
        numLoops = 1
        diameter = 75
        u = np.linspace(0, numLoops * 2 * np.pi, 400)
        pathX = diameter * np.sin(u) + 150
        pathY = diameter * np.cos(u) + 150

    elif ActivePath == 7: #Gcode input
        tempX, tempY = GcodeConverter(r"C:\Users\Ericw\Desktop\squareSpiral.gc")
        tempX = np.array(tempX)/15- 7.5
        tempY = np.array(tempY)/15 - 7.5

        pathX = tempX
        pathY = tempY

    else: #rectangle
        pathX = [100, 200, 200, 100, 100]
        pathY = [100, 100, 200, 200, 100]

    startupPlot()



# Build GUI------------------------------------------------------------------------------------------------------------
tkTop = tk.Tk()  # Create GUI Box
tkTop.geometry('1200x800')  # size of GUI
tkTop.title("2 DOF GUI")  # title in top left of window

# Title on top middle of screen
Title = tk.Label(text='Enter the desired coordinates of the 2 DOF arm', font=("Courier", 14, 'bold')).pack()

# Fill in the left Side------------------------------------------------------------------------------------------------
leftFrame = tk.Frame(master=tkTop, width=600) # create frame for the entry controls

leftFrame.pack(fill=tk.BOTH, side=tk.LEFT, expand=True)

TextFrame = tk.Frame(master=leftFrame, width=100)
x_input_Lable = tk.Label(master=TextFrame, text='X Coordinate:',
                                 font=("Courier", 12, 'bold')).pack(side='top', ipadx=0, padx=0, pady=0)
y_input_Lable = tk.Label(master=TextFrame, text='Y Coordinate:',
                                 font=("Courier", 12, 'bold')).pack(side='top', ipadx=0, padx=0, pady=0)

EntryFrame = tk.Frame(master=leftFrame, width=100)

x_coord_entry = tk.Entry(EntryFrame)
x_coord_entry.pack(side='top', ipadx=0, padx=0, pady=0)

y_coord_entry = tk.Entry(EntryFrame)
y_coord_entry.pack(side='top', ipadx=0, padx=0, pady=0)

# set initial coords to zero
x_coord_entry.insert(0,10)
y_coord_entry.insert(0,10)

UpdateCoordsButton = tk.Button(EntryFrame,
                                   text="Update Coordinates",
                                   command=lambda: set_coordinates_state(float(x_coord_entry.get()),float(y_coord_entry.get())),
                                   height=4,
                                   fg="black",
                                   width=20,
                                   bd=5,
                                   activebackground='green'
                                   )
UpdateCoordsButton.pack(side='top', ipadx=10, padx=10, pady=40)

StartPathButton = tk.Button(EntryFrame,
                                   text="Follow Path",
                                   command=StartPathFollow,
                                   height=4,
                                   fg="black",
                                   width=20,
                                   bd=5,
                                   activebackground='green'
                                   )
StartPathButton.pack(side='top', ipadx=10, padx=10, pady=40)

PathSelectorButton = tk.Button(EntryFrame,
                                   text="Change Path",
                                   command=ChangeSelectPathButton,
                                   height=4,
                                   fg="black",
                                   width=20,
                                   bd=5,
                                   activebackground='green'
                                   )
PathSelectorButton.pack(side='top', ipadx=10, padx=10, pady=40)

TextFrame.pack(fill=tk.BOTH, side=tk.LEFT, expand=True)
EntryFrame.pack(fill=tk.BOTH, side=tk.LEFT, expand=True)

# Fill in the right Side of GUI----------------------------------------------------------------------------------------
RightFrame = tk.Frame(master=tkTop, width=600, bg="gray")


RightFrame.pack(fill=tk.BOTH, side=tk.LEFT, expand=True)
startupPlot()

# run loop watching for gui interactions
tk.mainloop()