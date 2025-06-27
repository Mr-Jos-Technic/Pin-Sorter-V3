#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, Image, ImageFile, Font
from pybricks.messaging import BluetoothMailboxServer, BluetoothMailboxClient, LogicMailbox, NumericMailbox, TextMailbox
from threading import Thread
from random import choice
from math import fmod
import sys
import os
import math
import struct

from pybricks.iodevices import UARTDevice
from utime import ticks_ms
from uartremote import *

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
# Any (parts) of program taken from Anton's Mindstorms Hacks are used under the;
# MIT License: Copyright (c) 2021 Anton's Mindstorms
# MIT License: Copyright (c) 2022 Mr Jos for the rest of the code

#####################################################################
#####################################################################
##########~~~~~PROGRAM WRITTEN BY JOZUA VAN RAVENHORST~~~~~##########
##########~~~~~~~~~~~~~TECHNIC PIN SORTER V4.0~~~~~~~~~~~~~##########
##########~~~~~~~~~~~~~YOUTUBE CHANNEL: MR JOS~~~~~~~~~~~~~##########
#####################################################################
##########~~~~~~~~~~~~~EV3 ADVANCED MACHINERY~~~~~~~~~~~~~~##########
#####################################################################
#####################################################################


##########~~~~~~~~~~HARDWARE CONFIGURATION~~~~~~~~~~##########
ev3 = EV3Brick()                                                                    #Name we will be using to make the brick do tasks
#   UART setup 
port        = Port.S1
baudrate    = 115200
ur          = UartRemote(Port.S1)
ur.uart     = UARTDevice(port, baudrate=baudrate, timeout=100)
#   Motors definition
turning_arm   = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)        #Name for the motor that rotates the swingarm
storage_belt  = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)        #Name for the motor that powers the conveyor belt on the swingarm
scanning_belt = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE)        #Name for the motor that powers the conveyor belt with the color scanning sensors
feeder_belt   = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)        #Name for the motor that powers the 3 conveyor belts supplying the scanning belt
#   Sensor definition
touch_arm     = TouchSensor(Port.S2)                                                #Name for the touch sensor that is used to home the swingarm position
color_black   = ColorSensor(Port.S3)                                                #Name for the color sensor that uses a default black background
color_white   = ColorSensor(Port.S4)                                                #Name for the color sensor that uses a default white background


##########~~~~~~~~~~HOMING POSITION ANGLES WHEN SENSOR ACTIVATED~~~~~~~~~~##########
homing_pos_turning_arm = 400                                                          #The encoder position of the turning arm motor when being homed   #0 standard                          #Default 0


##########~~~~~~~~~~GEARING~~~~~~~~~~##########


##########~~~~~~~~~~MAXIMUM SPEED, MAXIMUM ACCELERATION, MAXIMUM POWER~~~~~~~~~~##########
feeder_belt.control.limits(   900,   100, 100)                                      #Default     900,   100, 100
scanning_belt.control.limits( 900, 36000, 100)                                      #Default     900, 36000, 100
storage_belt.control.limits( 1200,  3600, 100)                                      #Default    1200,  3600, 100
turning_arm.control.limits(  1200,  3600, 100)                                      #Default    1200,  3600, 100
nominal_speed_feeder =  250                                                         #Normal speed for the 3 supply conveyors                                                    #Default 250
maximum_speed_feeder =  900                                                         #Maximal speed for the 3 supply conveyors if there is no pin detected for a long time       #Default 900
current_speed_feeder = nominal_speed_feeder                                         #Global variable for the currently used speed for the supplying belts
speed_scanner        =  600                                                         #Real motor speed (deg/s) used for forward (scanning)                                       #Default 600
speed_turning_arm    = 1200                                                         #Real motor speed (deg/s) used for the swingarm rotation                                    #Default 1200
speed_dropoff_belt   = 1200                                                         #Real motor speed (deg/s) used for the swingarm conveyor belt                               #Default 1200


##########~~~~~~~~~~MAXIMUM ACCELERATION AND MAXIMUM ANGLE TO SAY A MOVEMENT IS FINISHED~~~~~~~~~~##########
feeder_belt.control.target_tolerances(  1000, 10)                                   #Allowed deviation from the target before motion is considered complete. (deg/s, deg)       (1000, 10)
scanning_belt.control.target_tolerances(1000, 10)                                   #Allowed deviation from the target before motion is considered complete. (deg/s, deg)       (1000, 10)
storage_belt.control.target_tolerances( 1200, 10)                                   #Allowed deviation from the target before motion is considered complete. (deg/s, deg)       (1200, 10)
turning_arm.control.target_tolerances(  1200, 10)                                   #Allowed deviation from the target before motion is considered complete. (deg/s, deg)       (1200, 10)


##########~~~~~~~~~~BLUETOOTH SETUP, SERVER SIDE~~~~~~~~~~##########                #This is not used in this project I use my standard template to program all my projects
#server = BluetoothMailboxServer()
#commands_bt_text = TextMailbox('commands text', server)                            #Main mailbox for sending commands and receiving feedback to/from other brick
#yaw_base_bt_zeroing = NumericMailbox('zero position yaw', server)                  #Mailbox for sending theta1 homing position


##########~~~~~~~~~~CREATING AND STARTING A TIMER, FOR INVERSE KINEMATIC SMOOTH CONTROL~~~~~~~~~~##########
timer_calibrate  = StopWatch()                                                      #Creating the timer that will be used for calibration
timer_pin_accept = StopWatch()                                                      #Creating the timer that will save the time a pin gets dropped on the swingarm conveyor
timer_feed_speed = StopWatch()                                                      #Creating the timer that will accelerate feeding conveyors if no pin is detected for some time
timer_pause      = StopWatch()


##########~~~~~~~~~~BUILDING GLOBAL VARIABLES~~~~~~~~~~##########
white_measuring_distance_start =    30                                              #30° motor rotation to start checking the pin color on the white RGB sensor, the start of a pin is inaccurate
white_samples                  =     5                                              #Take 5x RGB samples with the white background sensor to determine the pin color.
reject_to_sensor               =  -450                                              #Distance to return the undetermined pin, back to the start of the white sensor
reject_to_bin                  =  -800                                              #Distance to return the undetermined pin to the hopper
max_length_allowed             =   160                                              #Length for a pin to reject it automatically (mostly for 2pins touching each other)
minimal_distance               =  1500                                              #ms between 2 pins that are not equal. Needed for dropoff before turning the arm away
calibration_time               = 10000                                              #ms that the calibration function will be running if requested          #Default 60000 (60seconds)

reversing        = False                                                            #Global variable to know if the scanning belt is turning backwards
black_controlled = 0                                                                #Global counter to see what pin has its data completed by the last sensor (black background color sensor)
reject_in_row    = 0                                                                #Global counter to see howmany times in row a pin has been undetermined
arm_turned       = 0                                                                #Global counter to see what pin has been completely put in the storage bins
pin_list         = []                                                               #Global list with all the scanned pins data
cursor_pos       = 0                                                                #Global position counter to know what line in the menu is selected
pause_request    = False


#THE NEXT 2 VARIABLES MIGHT NEED SOME ADJUSTING, DEPENING ON YOUR COLOR SENSORS VALUES, BUT NORMALLY THE CALIBRATION FUNCTION WILL DO THIS AUTOMATICALLY
#In the next lines values R G & B will be used for RED GREEN & BLUE.
#limits_scanned is a variable defining the lower and upper RGB limit the sensor will detect when running stationary without seeing any pin.
#Whenever the value runs out of this range the program will think a pin just entered in front of the sensor
#If the scanning belt turns backwards without pins on the belt, one or more values will be incorrect. Use the calibration function to check the values and it will change these values itselve
#                   lower limit  /upper limit / lower limit   /upper limit
#RGB White empty:   R    G    B   R   G   B   /  R    G    B   R   G   B : RGB Black empty belt scan
limits_scanned = [ 16,  26,  28, 19, 31, 35,     0,   2,   8,  3,  5, 14]           #Operation settings to start without calibration [ 16,  26,  28, 19, 31, 35,   0,   2,   8,  3,  5, 14]
limits_factory = [ 20,  27,  19, 23, 31, 23,     0,   0,   0,  1,  2,  1]           #Factory settings to reset without calibration [ 16,  26,  28, 19, 31, 35,   0,   2,   8,  3,  5, 14]
#limits_factory = [ 16,  26,  28, 19, 31, 35,     0,   2,   8,  3,  5, 14]           #Factory settings to reset without calibration [ 16,  26,  28, 19, 31, 35,   0,   2,   8,  3,  5, 14]

#The next variable is a dictionary, it holds all data to determine what color and length any kind of pin is. It also holds the angle where the swingarm should dropoff this pin, and counts them
#For each pin the limit values are noted, lower limit and upper limit for R G B & length, some pins have small overlapps so they are left out. 
#That's why rescans are done to try again and see if it now falls completely in range of 100% certain, watch out if you change these values to much that you don't take in other pins in range
#1700 = front view, 150 interval = little space, try 120
#                                                                             White      Black          White                         Black
#                Name           Scancounter     Swingarm pos     limits       <len >len <len >len       <=R >=R <=G >=G <=B >=B       <=R >=R <=G >=G <=B >=B
pins_scanned = {"ReScan"     : {"counter" : 0 , "angle" :  200 , "dataset" : [   0,   0,   0,   0,       0,  0,  0,  0,  0,  0,       0,  0,  0,  0,  0,  0]} , \
                "Reject"     : {"counter" : 0 , "angle" :  200 , "dataset" : [   0,   0,   0,   0,       0,  0,  0,  0,  0,  0,       0,  0,  0,  0,  0,  0]} , \
                "Black 2L"   : {"counter" : 0 , "angle" : 2140 , "dataset" : [  91, 120,  50,  98,       1,  4,  1,  4,  0,  1,       0,  1,  0,  1,  0,  0]} , \
                "Black 3L"   : {"counter" : 0 , "angle" : 2260 , "dataset" : [ 130, 157,  50, 143,       1,  4,  1,  4,  0,  1,       0,  1,  0,  1,  0,  0]} , \
                "DBG 3L"     : {"counter" : 0 , "angle" :  980 , "dataset" : [ 112, 157,  30, 151,       5,  7,  5,  8,  0,  4,       1,  4,  2,  4,  1,  3]} , \
                "DBG 1.5L"   : {"counter" : 0 , "angle" :  880 , "dataset" : [  73, 100,  50,  89,       3,  8,  4,  7,  0,  4,       1,  4,  2,  4,  0,  3]} , \
                "Blue 3L"    : {"counter" : 0 , "angle" : 1920 , "dataset" : [ 130, 159, 107, 143,       1,  6,  4, 11,  5, 16,       0,  2,  3,  5,  8, 18]} , \
                "Blue 2L"    : {"counter" : 0 , "angle" : 1820 , "dataset" : [  95, 116,  70, 100,       1,  6,  4, 11,  5, 16,       0,  2,  3,  5,  8, 24]} , \
                "Blue 1.25L" : {"counter" : 0 , "angle" : 1700 , "dataset" : [  66,  90,  40,  76,       1,  6,  4, 11,  5, 16,       0,  2,  1,  5,  5, 24]} , \
                "Tan 3L"     : {"counter" : 0 , "angle" : 1600 , "dataset" : [ 128, 171, 114, 159,      17, 24, 13, 20,  5, 10,       8, 16,  9, 14,  3, 11]} , \
                "Tan 2L"     : {"counter" : 0 , "angle" : 1500 , "dataset" : [  95, 115,  79, 106,      15, 28, 13, 25,  4, 10,       8, 16,  8, 15,  3, 12]} , \
                "Tan 1.5L"   : {"counter" : 0 , "angle" : 1400 , "dataset" : [  77,  89,  58,  79,      15, 28, 13, 25,  3, 10,       8, 16,  6, 15,  5, 12]} , \
                "Red 3L"     : {"counter" : 0 , "angle" : 2020 , "dataset" : [ 116, 157, 110, 149,      11, 21,  3,  8,  0,  2,       6, 15,  1,  3,  0,  2]} , \
                "LBG 1.25L"  : {"counter" : 0 , "angle" : 1100 , "dataset" : [  66,  90,  45,  74,       9, 14,  9, 16,  4, 11,       2,  8,  1,  9,  4, 11]} , \
                "LBG 2L"     : {"counter" : 0 , "angle" : 1200 , "dataset" : [  95, 120,  80, 105,       9, 14,  9, 16,  4, 11,       5,  9,  5, 10,  3, 12]} , \
                "LBG 3L"     : {"counter" : 0 , "angle" : 1300 , "dataset" : [ 130, 159, 112, 152,       9, 14,  9, 16,  4, 11,       3, 10,  3, 11,  2, 17]} }
#[25, 33, 25, 27, 36, 28, 0, 2, 2, 2, 3, 5] as basic background colors for this table

#pins_scanned = {"ReScan"     : {"counter" : 0 , "angle" :  200 , "dataset" : [   0,   0,   0,   0,       0,  0,  0,  0,  0,  0,       0,  0,  0,  0,  0,  0]} , \
#                "Reject"     : {"counter" : 0 , "angle" :  200 , "dataset" : [   0,   0,   0,   0,       0,  0,  0,  0,  0,  0,       0,  0,  0,  0,  0,  0]} , \
#                "Black 2L"   : {"counter" : 0 , "angle" : 1700 , "dataset" : [  91, 116,  75, 104,       0,  2,  0,  2,  0,  1,       0,  2,  0,  2,  0,  1]} , \
#                "Black 3L"   : {"counter" : 0 , "angle" : 2000 , "dataset" : [ 119, 157, 110, 143,       0,  3,  0,  2,  0,  1,       0,  2,  0,  2,  0,  2]} , \
#                "DBG 3L"     : {"counter" : 0 , "angle" : 2450 , "dataset" : [ 130, 150, 100, 142,       0,  5,  2,  5,  0,  2,       1,  4,  2,  5,  0,  7]} , \
#                "DBG 1.5L"   : {"counter" : 0 , "angle" :  650 , "dataset" : [  73,  97,  52,  83,       0,  5,  0,  5,  0,  2,       1,  5,  2,  5,  0,  7]} , \
#                "Blue 3L"    : {"counter" : 0 , "angle" : 1850 , "dataset" : [ 115, 150, 110, 137,       0,  3,  2,  7,  5, 17,       0,  4,  0,  7,  6, 50]} , \
#                "Blue 2L"    : {"counter" : 0 , "angle" : 1550 , "dataset" : [  85, 111,  76,  95,       0,  4,  1,  7,  6, 17,       0,  4,  0,  8, 13, 50]} , \
#                "Blue 1.25L" : {"counter" : 0 , "angle" :  950 , "dataset" : [  65,  84,  20,  68,       0,  3,  2,  9,  2, 15,       0,  2,  0,  5,  9, 26]} , \
#                "Tan 3L"     : {"counter" : 0 , "angle" : 2300 , "dataset" : [ 120, 153, 107, 149,       9, 15,  7, 13,  2, 11,      12, 25,  9, 20,  8, 32]} , \
#                "Tan 2L"     : {"counter" : 0 , "angle" : 1400 , "dataset" : [  88, 111,  74,  97,       9, 15,  7, 13,  2, 11,      11, 25,  9, 20,  7, 32]} , \
#                "Tan 1.5L"   : {"counter" : 0 , "angle" :  800 , "dataset" : [  67,  90,  52,  78,     7.2, 14,  5, 11,  1,  8,       9, 19,  5, 16,  7, 20]} , \
#                "Red 3L"     : {"counter" : 0 , "angle" : 2150 , "dataset" : [ 120, 152, 109, 140,       6, 12,  1,  4,  0,  3,      10, 18,  1,  4,  1,  6]} , \
#                "LBG 1.25L"  : {"counter" : 0 , "angle" : 1100 , "dataset" : [  65,  83,  42,  68,       3,  7,  3,  8,  1,  7,       2, 12,  3, 11,  5, 28]} , \
#                "LBG 2L"     : {"counter" : 0 , "angle" : 1250 , "dataset" : [  91, 111,  72, 100,       3,  7,  3,  7,  1,  7,       5, 12,  6, 13,  5, 28]} , \
#                "LBG 3L"     : {"counter" : 0 , "angle" : 2600 , "dataset" : [ 127, 148, 102, 135,       2,  7,  3,  8,  1,  8,       5, 12,  6, 13,  5, 28]} }

##########~~~~~~~~~~BRICK STARTUP SETTINGS~~~~~~~~~~##########
ev3.speaker.set_volume(volume=80, which='_all_')                                    #Set the volume for all sounds (speaking and beeps etc)
ev3.speaker.set_speech_options(language='en', voice='m7', speed=None, pitch=None)   #Select speaking language, and a voice (male/female)
small_font = Font(size=6)                                                           # 6 pixel height for text on screen
normal_font = Font(size=10)                                                         #10 pixel height for text on screen
big_font = Font(size=16)                                                            #16 pixel height for text on screen
ev3.screen.set_font(normal_font)                                                    #Choose a preset font for writing next texts
ev3.screen.clear()                                                                  #Make the screen empty (all pixels white)
#ev3.speaker.beep()                                                                 #Brick will make a beep sound 1 time
ev3.light.off()                                                                     #Turn the lights off on the brick


##########~~~~~~~~~~CREATING A FILE THAT IS SAVED OFFLINE~~~~~~~~~~##########       #This is used to store your own last calibration values offline, so it will remember them next startup
#os.remove("calibrationdata.txt")                                                   #This is for removing the file we will make next, this is for debugging for me, keep the # in front of it
create_file = open("calibrationdata.txt", "a")                                      #Create a file if it does not exist and open it, if it does exist, just open it
create_file.write("")                                                               #Write the default values to the file, for first ever starttup so it holds values
create_file.close()                                                                 #Close the file again, to be able to call it later again

with open("calibrationdata.txt") as retrieve_data:                                  #Open the offline data file
    data_retrieval_string = retrieve_data.read().splitlines()                       #The data is in the Type: String , read the complete file line by line
if len(data_retrieval_string) < 12: data_background_offline = limits_scanned        #Check if there are 12 values in the string list, if not then it is first start of this program ever
else:                                                                               #If there are 12 then it will convert the String to a Integer list.
    data_background_offline = []
    for x in data_retrieval_string:
        data_background_offline.append(int(x))
limits_scanned = data_background_offline                                            #The background color is now defined from the offline file (last calibration done)
print(limits_scanned)
#[20, 27, 18, 22, 30, 22, 0, 1, 0, 2, 3, 3]

##########~~~~~~~~~~CREATING FUNCTIONS THAT CAN BE CALLED TO PERFORM REPETITIVE OR SIMULTANEOUS TASKS~~~~~~~~~~##########
##########~~~~~~~~~~UART RECEIVING COMMUNICATION COMMANDS~~~~~~~~~~##########
def mode_selection(task, state):
    global pause_request
    print("task received with", task, state)
    if task == 0:
        if state == 0:
            timer_pause.reset()
            timer_pause.resume()
            pause_request = True
            feeder_belt.stop()
            while pause_request == True:
                if timer_pause.time() > 5000:
                    scanning_belt.stop()
                    timer_pause.reset()
                    break
            while pause_request == True:
                if timer_pause.time() > 3000:
                    storage_belt.stop()
                    break
            timer_pause.pause()
        elif state == 1:
            timer_pause.pause()
            pause_request = False
            scanning_belt.run(speed_scanner)
            storage_belt.run(speed_dropoff_belt)
            feeder_belt.run(current_speed_feeder)                       #And restart the 3 feeding belts


ur.add_command(mode_selection)

def send_update_scan():
    global black_controlled
    last_msg = 0

    while True:
        wait(100)
        if last_msg < black_controlled:
            while ur.call("update_scan", '%ss'%len(pin_list[last_msg][7]), pin_list[last_msg][7]) == None: continue
            last_msg += 1
        ur.process_uart()


def save_offline_data():                                                            #This definition will save the current background limits to the offline file, if it is called
    with open("calibrationdata.txt", "w") as backup_data:
        for current_data in limits_scanned:
            backup_data.write(str(current_data) + "\n")


def calibration_sensors():                                                          #This definition is only used if a calibration is requested for the background color
    global limits_scanned

    limits_scanned = [100, 100, 100,  0,  0,  0, 100, 100, 100,  0,  0,  0]         #Reset the limits for the default background to their maximum range
    onscreen_timer_line = "Calibration time: {} / {} seconds   "                    #Create a text line with 2 blank spots, to be filled in later
    onscreen_limits     = "{} {} sensor              "
    ev3.screen.draw_text(4, 60, "KEEP THE BELT EMPTY                                                 ", text_color=Color.WHITE, background_color=Color.BLACK) 
    ev3.screen.draw_text(103, 114, "Mr Jos creation", text_color=Color.BLACK, background_color=Color.WHITE)
    scanning_belt.run_angle(900, reject_to_bin)                                     #Reverse the scanning belt at full speed to throw all pins back in the bulk hopper
    scanning_belt.run(speed_scanner)                                                #Start the scanning conveyor to have a real simulation what the background will look like
    timer_calibrate.reset()                                                         #Reset the timer to 0ms

    while timer_calibrate.time() < calibration_time:
        ev3.screen.draw_text(4, 71, onscreen_timer_line.format(math.floor(timer_calibrate.time() / 1000), calibration_time / 1000), text_color=Color.BLACK, background_color=Color.WHITE)
        ev3.screen.draw_text(4, 82, onscreen_limits.format(limits_scanned[:6], "White"), text_color=Color.BLACK, background_color=Color.WHITE)  #Write text on the EV3 by filling in blank spots
        ev3.screen.draw_text(4, 93, onscreen_limits.format(limits_scanned[6:], "Black"), text_color=Color.BLACK, background_color=Color.WHITE)
        wait(10)
        white_scan = color_white.rgb()                                              #Take a sample from the color sensor with a white background
        black_scan = color_black.rgb()                                              #Take a sample from the color sensor with a black background
        for x in range(3):
            if white_scan[x] < limits_scanned[x]:     limits_scanned[x]     = white_scan[x]     #Check for every measurement if it exceeds the current limit value, and override if greater
            if white_scan[x] > limits_scanned[x + 3]: limits_scanned[x + 3] = white_scan[x]
            if black_scan[x] < limits_scanned[x + 6]: limits_scanned[x + 6] = black_scan[x]
            if black_scan[x] > limits_scanned[x + 9]: limits_scanned[x + 9] = black_scan[x]
        print(limits_scanned)
    ev3.screen.draw_text(4, 60, "Calibration is finished                                             ", text_color=Color.WHITE, background_color=Color.BLACK)
    scanning_belt.brake()                                                           #Stopping the scanning belt conveyor
    save_offline_data()                                                             #Saving the new background color limits to the offline data file
    wait(1000)
    ev3.screen.draw_text(4, 60, "Calibration is finished                                             ", text_color=Color.BLACK, background_color=Color.WHITE)


def check_color_white():                                                            #This definition will handle the color sensor viewing the white background
    global pin_list
    global reversing
    global current_speed_feeder
    global nominal_speed_feeder

    while True:
        counter   = 0                                                               #Local variable to count the amount of samples in row, that are out of range
        pin_start = 0                                                               #Local variable to save the motor angle at which the start of a new pin was first detected
        pin_color = [0, 0, 0]                                                       #Local variable list to save the RGB colors of the current pin measured
        timer_feed_speed.reset()                                                    #Reset the timer to 0 everytime a new pin started detection, or a pin is succesfully measured
        
        ########## Waiting for the start of a new pin ##########
        while True:
            if counter == 0 and timer_feed_speed.time() > 1000 and current_speed_feeder < maximum_speed_feeder and pause_request == False: #Every second no pin is detected and feeding not maxed out yet, do this
                timer_feed_speed.reset()                                            #Reset the timer back to 0 (to start counting back to 1000ms)
                current_speed_feeder += 20                                          #Set the desired speed for the feeding belt 20°/s higher then before
                feeder_belt.run(current_speed_feeder)                               #Send the new speed to the motor controlling the 3 feeding conveyor belts
            white_scan = color_white.rgb()                                          #Take a sample from the color sensor with a white background
            trigger = False                                                         #Local variable used to see if any value will be out of range
            for x in range(3):
                if white_scan[x] < limits_scanned[x] or white_scan[x] > limits_scanned[x + 3]:  #Check if any of the 3 color values is out of background color range
                    if trigger == False and counter == 0: pin_start = scanning_belt.angle()     #Save the motor angle at which the first RGB value out of nominal range is detected
                    if trigger == False: trigger = True
            if trigger == True: counter += 1                                        #If any of the 3 colors was out of range count up by 1
            elif counter > 0: counter = 0                                           #If none of them was out of range reset the in row detected back to 0
            if counter == 3: break                                                  #If 3 times in row a value was out of range a pin is detected (To make sure it's not a single faulty value)
        current_speed_feeder = nominal_speed_feeder                                 #Set the desired speed for the feeding belt back to nominal
        if pause_request == False:                                                  #TODO check if this pause updat works fine
            feeder_belt.run(current_speed_feeder)                                       #Send the new speed to the motor controlling the 3 feeding conveyor belts
        ########## Waiting for the correct position to take a few color samples ##########
        while scanning_belt.angle() < pin_start + white_measuring_distance_start and reversing == False: continue   #Start at a given distance after the pin started for better accuracy
        if reversing == True:                                                       #If the global variable tells that the scanning belt reverses, the new pin detected was incorrect
            while reversing == True: continue                                       #During reversing stay in this loop waiting for the reversing to be finished
            counter = 0
            continue                                                                #This resets back to the top of the "While True: loop", to start scanning for the start of the pin again
        pin_samples = []                                                            #Local variable to save all measurements from each sample
        for x in range(white_samples):                                              #Take the amount of samples required
            white_scan = color_white.rgb()
            pin_samples.extend(white_scan)                                          #Add the RGB result to the end of the list
        for x in range(white_samples):
            for y in range(3):                                                      #Add for each color, the result values together
                pin_color[y] += pin_samples[(3 * x) + y]
        for x in range(3):                                                          #Divide by the amount of sample for 1 single RGB result that is the average from all the samples
            pin_color[x] = round(pin_color[x] / white_samples, 1)                   #Round down to 1 decimal to prevent 8.0000000000001
        ########## Waiting for the end of the new pin ##########
        while True:
            white_scan = color_white.rgb()                                          #Take a sample from the color sensor with a white background
            trigger = 0                                                             #All 3 values have to be in range, so reset every loop the local variable to 0 triggers
            for x in range(3):
                if white_scan[x] < limits_scanned[x] or white_scan[x] > limits_scanned[x + 3]: continue
                else: trigger += 1                                                  #For each R G & B that are back in range trigger is added by 1
            if trigger == 3: break                                                  #If all 3 the RGB values are back in nominal range, end the pin length
        pin_end = scanning_belt.angle()                                             #Save the motor angle at which the first RGB value back in nominal range is detected
        ########## Pin DATA; Startpoint, length and white color values ##########
        pin_list.append([pin_start, pin_end - pin_start, pin_color])                #Store the first pin data; Startpoint , length and white color values


def check_color_black():                                                            #This definition will handle the color sensor viewing the black background
    global black_controlled
    global pin_list
    global reversing
    global reject_in_row
    
    while True:
        counter     = 0                                                             #Local variable to count the amount of samples in row, that are out of range
        pin_start   = 0                                                             #Local variable to save the motor angle at which the start of a new pin was first detected
        pin_color   = [0, 0, 0]                                                     #Local variable list to save the RGB colors of the current pin measured
        pin_to_long = False                                                         #Local variable to know if a pin is to long and might fall off the scanning belt unwanted
        
        if len(pin_list) > black_controlled:                                        #Only start this if data has been added for a pin that has not been scanned yet by black background sensor
            ########## Waiting for the start of a new pin in front of the black sensor ##########
            while True:
                black_scan = color_black.rgb()                                      #Take a sample from the color sensor with a black background
                trigger = False                                                     #Reset the local variable trigger to False
                for x in range(3):
                    if black_scan[x] < limits_scanned[x + 6] or black_scan[x] > limits_scanned[x + 9]:  #Check if any of the 3 color values is out of background color range
                        if trigger == False and counter == 0: pin_start = scanning_belt.angle()         #Save the motor angle at which the first RGB value out of nominal range is detected
                        if trigger == False: trigger = True                         #If one of the values is out of range trigger the whole set as 1 set out of range 
                if trigger == True: counter += 1                                    #Count the sets in row that are out of range
                elif counter > 0: counter = 0                                       #If there was a set out of range, but not now again, it was a false trigger, and resets
                if counter == 3: break                                              #If one/more of the 3 RGB values has been out of nominal range in row, a pin is detected
            ########## Waiting for the correct position to take a color sample ##########
            while scanning_belt.angle() < pin_list[black_controlled][0] + 270: continue                 #Start at a given distance after the pin started at the white sensor, for better accuracy
            pin_color_tuple = color_black.rgb()                                     #Take 1 color sample from the pin
            for x in range(3): pin_color[x] = pin_color_tuple[x]                    #Change from tuple to list, to be able to add the values to the data list later
            ########## Waiting for the end of the new pin ##########
            while True:
                black_scan = color_black.rgb()                                      #Take a sample from the color sensor with a black background
                trigger = 0                                                         #All 3 values have to be in range, so reset every loop the local variable to 0 triggers
                for x in range(3):
                    if black_scan[x] < limits_scanned[x + 6] or black_scan[x] > limits_scanned[x + 9]: continue
                    else: trigger += 1                                              #For each R G & B that are back in range trigger is added by 1
                if trigger == 3: break                                              #If all 3 the RGB values are back in nominal range, end the pin length
                if scanning_belt.angle() > pin_start + max_length_allowed:          #If the end of the pin is not detected for a to long time, stop scanning for the end
                    pin_to_long = True                                              #Save to a local variable that the pin was to long (2touching etc)
                    break                                                           #Stop the scanning
            pin_end = scanning_belt.angle()                                         #Save the motor angle at which the first RGB value back in nominal range is detected
            if pin_to_long == True:                                                 #If the pin was determined to long perform the next task
                pin_end = 999999999                                                 #Save pin end encoder position to a very great value
                pin_to_long = False
            ########## Pin DATA added;        Startpoint, length,              black color values and distanse between the pin starts for both sensors
            pin_list[black_controlled].extend([pin_start, pin_end - pin_start, pin_color, pin_start - pin_list[black_controlled][0]])
            #Example: [3032, 99, [12.0, 9.4, 6.4], 3269, 80, (20, 16, 19), 237]         
            #[3032               , 99          , [12.0, 9.4, 6.4], 3269               , 80          , (20, 16, 19)    , 237]
            #[startpos in ° white, length white, color from white, startpos in ° black, length black, color from black, startpoints distance]

            ########## Read the type of pin being scanned by using all the DATA ##########
            result_pin = check_result_scans(pin_list[black_controlled][1], pin_list[black_controlled][4], pin_list[black_controlled][2] + pin_list[black_controlled][5])
            pin_list[black_controlled].extend([result_pin])                         #Add the name of the determined pin to the data pin list

            ########## If the pin is undetermined, perform a rescan, if 3rd scan still fails, bin it ##########
            if result_pin == "ReScan":                                              #If the name of the pin is Rescan, perform the rescan job and show on laptop the data
                print(pin_list[black_controlled][1], pin_list[black_controlled][2], pin_list[black_controlled][4], pin_list[black_controlled][5], pin_list[black_controlled][7])
                reject_in_row += 1                                                  #Every rescan done in row adds 1 up.
                scanning_belt.brake()                                               #Brake the scanning belt to prevent the undetermined pin to fall off onto the swingarm
                reversing = True                                                    #Change the global variable so all functions know that the scanning belt will reverse
                if reject_in_row < 3:                                               #If less then 3 rescans in row are performed;
                    pins_scanned[result_pin]["counter"] += 1                        #Add 1 pin to the Rescanned counter
                    scanning_belt.run_angle(900, reject_to_sensor)                  #Reverse the scanning belt at full speed so the undetermined pin can be rescanned
                else:                                                               #If it's the 3rd undetermined in row;
                    pins_scanned["Reject"]["counter"] += 1                          #Add 1 pin to the Rejected counter
                    scanning_belt.run_angle(900, reject_to_bin)                     #Reverse the scanning belt at full speed to throw all pins back in the bulk hopper
                    reject_in_row = 0                                               #Reset the variable rescans in row back to 0
                del pin_list[black_controlled:]                                     #Delete all started pin data for the not fully finished pins (including last undetermined)
                scanning_belt.reset_angle(0)                                        #Reset the scanning belt encoder to 0 to prevent bugs with pin length
                reversing = False                                                   #Change the global variable so all functions know that the scanning belt will run again
                scanning_belt.run(speed_scanner)                                    #Start the scanning belt at the scanning speed again
            else: 
                pins_scanned[result_pin]["counter"] += 1                            #Add 1 pin to the determined pin counter
                reject_in_row = 0
                ########## Check if the gap between pins is sufficient for the swingarm to be able to rotate ##########
                if black_controlled != 0:                                           #If it's the first pin after startup, there is no other pin to calculate the difference
                    swing_angle = math.fabs(pins_scanned[pin_list[black_controlled - 1][7]]["angle"] - pins_scanned[pin_list[black_controlled][7]]["angle"]) #Calculate swingarm motion distance
                    time_needed_swing = swing_angle / speed_turning_arm * 1000      #Calculate swingarm motion time needed
                    if swing_angle != 0:                                            #If the determined pin will not be put in the same bin;
                        if time_needed_swing + minimal_distance > timer_pin_accept.time() - pin_list[black_controlled - 1][8]:      #If the pins are scanned to close to eachother, create gap
                            scanning_belt.brake()                                   #Stop the scanning belt
                            feeder_belt.stop()                                      #Stop the feeding belts
                        while time_needed_swing + minimal_distance > timer_pin_accept.time() - pin_list[black_controlled - 1][8]:
                            continue                                                #Wait for the gap to be big enough
                        scanning_belt.run(speed_scanner)                            #If the gap is big enough restart the scanning belt
                        if pause_request == False:                                  #TODO this has been added later, check functionality
                            feeder_belt.run(current_speed_feeder)                       #And restart the 3 feeding belts
                else: time_needed_swing = 2000                                      #Global variable making for the first pin sorted at startup  

                current_time = timer_pin_accept.time()                              #At this time the pin gets dropped off the scanning belt, and the time is saved
                pin_list[black_controlled].extend([current_time, int(current_time + (1500 / speed_dropoff_belt * 1000) - time_needed_swing)])   #Save the data when to start the swingarm motion
                ########## This next line will put all the DATA in 1 line on your laptop screen if you run it in Visual Studio Code, so you can see all values that were needed ##########
                #print(pin_list[black_controlled][1], pin_list[black_controlled][2], pin_list[black_controlled][4], pin_list[black_controlled][5], pin_list[black_controlled][7])
                black_controlled += 1                                               #The global variable is added by 1 now that a pin is completely determined and send to the correct bin
                #send_update_scan(result_pin)


def check_result_scans(length_white, length_black, pin_clr):                        #This definition will check if the scanned data matches with any pin data from the dictionary
    for x in pins_scanned:                                                          #It will automatically loop for every pin defined in the dictionary, so you can add your own pins at line 116
        if  pins_scanned[x]["dataset"][ 0] <  length_white <  pins_scanned[x]["dataset"][ 1] and pins_scanned[x]["dataset"][ 2] <  length_black <  pins_scanned[x]["dataset"][ 3] and \
            pins_scanned[x]["dataset"][ 4] <= pin_clr[0]   <= pins_scanned[x]["dataset"][ 5] and pins_scanned[x]["dataset"][ 6] <= pin_clr[1]   <= pins_scanned[x]["dataset"][ 7] and \
            pins_scanned[x]["dataset"][ 8] <= pin_clr[2]   <= pins_scanned[x]["dataset"][ 9] and pins_scanned[x]["dataset"][10] <= pin_clr[3]   <= pins_scanned[x]["dataset"][11] and \
            pins_scanned[x]["dataset"][12] <= pin_clr[4]   <= pins_scanned[x]["dataset"][13] and pins_scanned[x]["dataset"][14] <= pin_clr[5]   <= pins_scanned[x]["dataset"][15]: return x
    return "ReScan"                                                                 #If it does not match with any pin, it will send back that a rescan is needed


def rotate_turning_arm():                                                           #This definition handles the swingarm motion
    global black_controlled
    global pin_list
    global arm_turned

    while True:
        if arm_turned < black_controlled:                                           #If a new pin is succesfully detected, and has its dropoff time added, start the waiting
            while timer_pin_accept.time() < pin_list[arm_turned][9]: wait(25)       #Wait untill the time is reached that the swinging should start
            turning_arm.run_target(speed_turning_arm, pins_scanned[pin_list[arm_turned][7]]["angle"], then=Stop.HOLD, wait=True)    #Move to the correct position with the swingarm
            arm_turned += 1                                                         #Complete the swingarm motion for this pin
        wait(100)                                                                   #Wait block to make the rest of the program run faster


def homing_swingarm():                                                              #This definiton will find the homing position for the swingarm, so it knows where every bin is
    ev3.screen.draw_text(4,  4, "Homing the swingarm", text_color=Color.BLACK, background_color=Color.WHITE)
    if touch_arm.pressed() == True:                                                 #If at the startup of the program, the senso is already touched;
        while touch_arm.pressed() == True:
            turning_arm.run(200)                                                    #Run the swingarm away from the sensor while it's still touched
        wait(100)                                                                   #Run a little longer to be sure it's free, and will home correctly
        turning_arm.stop()                                                          #Stop the swingarm withotu a hard braking
        wait(250)                                                                   #Let it come to a standstill

    while touch_arm.pressed() == False:      
        turning_arm.run(-600)                                                       #Run it towards the touching sensor untill the touch sensor is pushed in #300before
    turning_arm.hold()                                                              #Lock the motor in position, it will hold the exact encoder position
    ev3.screen.draw_text(4,  4, "Swingarm encoder position is now known", text_color=Color.BLACK, background_color=Color.WHITE)
    turning_arm.reset_angle(homing_pos_turning_arm)                                 #Reset the encoder position for the swingarm
    ev3.screen.draw_text(4, 15, "Emptying the scanning belt", text_color=Color.BLACK, background_color=Color.WHITE)
    scanning_belt.run_angle(900, reject_to_bin, wait=False)                         #Run the scanning belt backwards at full speed, to throw off any pins that are unknown at startup
    ev3.screen.draw_text(4, 26, "Putting swingarm in start position", text_color=Color.BLACK, background_color=Color.WHITE)
    turning_arm.run_target(speed_turning_arm, 1700, then=Stop.HOLD, wait=True)      #Run the swingarm to the position so it is in line with the machine
    scanning_belt.reset_angle(0)                                                    #The scanning belt will have put any excess in the hopper and braked, resetting the encoder to 0 now


def pushingbuttons():                                                               #Function to wait for a button to be pressed on the EV3 brick, and return which one was pressed
    while True:
        if   ev3.buttons.pressed() == [Button.UP]:
            wait_for_release_buttons()
            return "up"
        if ev3.buttons.pressed() == [Button.DOWN]:
            wait_for_release_buttons()
            return "down"
        elif ev3.buttons.pressed() == [Button.LEFT]:
            wait_for_release_buttons()
            return "left"
        elif ev3.buttons.pressed() == [Button.RIGHT]:
            wait_for_release_buttons()
            return "right"
        elif ev3.buttons.pressed() == [Button.CENTER]:
            wait_for_release_buttons()
            return "center"


def wait_for_release_buttons():                                                     #Function to wait for the EV3 buttons to be all released
    while ev3.buttons.pressed() != []: continue


def draw_text_lines_menu(selected):                                                 #Function to color the selected line in the menu
    if   selected == 0:
        ev3.screen.draw_text(4,  4, "Start the sorting installation", text_color=Color.WHITE, background_color=Color.BLACK)            #Cursor pos 0 selected background showing on screen
        ev3.screen.draw_text(4, 15, "Perform a background calibration", text_color=Color.BLACK, background_color=Color.WHITE)          #Cursor pos 1
        ev3.screen.draw_text(4, 26, "Reset to factory background settings", text_color=Color.BLACK, background_color=Color.WHITE)      #Cursor pos 2
    elif selected == 1:
        ev3.screen.draw_text(4,  4, "Start the sorting installation", text_color=Color.BLACK, background_color=Color.WHITE)            #Cursor pos 0
        ev3.screen.draw_text(4, 15, "Perform a background calibration", text_color=Color.WHITE, background_color=Color.BLACK)          #Cursor pos 1
        ev3.screen.draw_text(4, 26, "Reset to factory background settings", text_color=Color.BLACK, background_color=Color.WHITE)      #Cursor pos 2
    elif selected == 2:
        ev3.screen.draw_text(4,  4, "Start the sorting installation", text_color=Color.BLACK, background_color=Color.WHITE)            #Cursor pos 0
        ev3.screen.draw_text(4, 15, "Perform a background calibration", text_color=Color.BLACK, background_color=Color.WHITE)          #Cursor pos 1
        ev3.screen.draw_text(4, 26, "Reset to factory background settings", text_color=Color.WHITE, background_color=Color.BLACK)      #Cursor pos 2
    else:                                                                           #Nothing selected, sub program running
        ev3.screen.draw_text(4,  4, "Start the sorting installation", text_color=Color.BLACK, background_color=Color.WHITE)            #Cursor pos 0
        ev3.screen.draw_text(4, 15, "Perform a background calibration", text_color=Color.BLACK, background_color=Color.WHITE)          #Cursor pos 1
        ev3.screen.draw_text(4, 26, "Reset to factory background settings", text_color=Color.BLACK, background_color=Color.WHITE)      #Cursor pos 2


def clear_screen():
    ev3.screen.clear()                                                                  #Empty the complete screen on the EV3 brick
    ev3.screen.draw_text(103, 114, "Mr Jos creation", text_color=Color.BLACK, background_color=Color.WHITE)     #Write text on the EV3 screen on the XY grid

    
##########~~~~~~~~~~CREATING MULTITHREADS~~~~~~~~~~##########
sub_white_scanner = Thread(target=check_color_white)                                #Creating a multithread so the definition can run at the same time as the main program, if it's called
sub_black_scanner = Thread(target=check_color_black)
sub_turning_arm   = Thread(target=rotate_turning_arm)
sub_send_update   = Thread(target=send_update_scan)

##########~~~~~~~~~~MAIN PROGRAM~~~~~~~~~~##########
clear_screen()
while True:
    draw_text_lines_menu(cursor_pos)                                                #Show on screen the selected mode currently
    lastpress = pushingbuttons()
    if   lastpress == "center":                                                     #If the last button press was the center button;
        if   cursor_pos == 0: break                                                 #Start running program is selected, so start normal operation
        elif cursor_pos == 1:
            draw_text_lines_menu(100)                                               #Remove the black background selector
            calibration_sensors()                                                   #Calibration was selected, so perform the calibration
        elif cursor_pos == 2:
            limits_scanned = limits_factory                                         #Factory reset was selected, so only the background color limit will be reset to factory settings
            save_offline_data()
        cursor_pos = 0                                                              #Then put the selected mode back at start program (but you could choose te recalibrate again)
    elif lastpress == "down" and cursor_pos < 2: cursor_pos += 1                    #Move the cursor position one line down
    elif lastpress == "up"   and cursor_pos > 0: cursor_pos -= 1                    #Move the cursor position one line up


clear_screen()
homing_swingarm()
clear_screen()
sub_white_scanner.start()                                                           #Starting the multithread
sub_black_scanner.start()
sub_turning_arm.start()
sub_send_update.start()
storage_belt.run(speed_dropoff_belt)
feeder_belt.run(current_speed_feeder)
scanning_belt.run(speed_scanner)

onscreen_counter_line = "{}: {} {} "                                                #Create a text line with 3 blank spots, to be filled in later
while True:                                                                         #This loop will write on the EV3 screen all the sorted pin amounts (every 10seconds refreshes)
    pos_screen = 0
    counter_pins = 0
    rescanned_pins = 0
    for x in pins_scanned:                                                          #Check every item in the dictionary
        y = pins_scanned[x]["counter"]
        if x == "ReScan" or x == "Reject": rescanned_pins += y                      
        else: counter_pins += y
        if pos_screen < 8: ev3.screen.draw_text(4, 4 + (11 * pos_screen), onscreen_counter_line.format(x, y, "pins"), text_color=Color.BLACK, background_color=Color.WHITE)
        else:              ev3.screen.draw_text(93, 4 + (11 * (pos_screen - 8)), onscreen_counter_line.format(x, y, "pins"), text_color=Color.BLACK, background_color=Color.WHITE)
        pos_screen += 1
    ev3.screen.draw_text(4, 103, onscreen_counter_line.format("Total pins sorted", counter_pins, "pins"), text_color=Color.BLACK, background_color=Color.WHITE)
    if counter_pins > 0: ev3.screen.draw_text(4, 114, onscreen_counter_line.format("% rescans", int(rescanned_pins / counter_pins * 100), "%  "), text_color=Color.BLACK, background_color=Color.WHITE)
    wait(5000)                                                                     #Every 5 seconds the screen stats are updated, if refreshed to fast it will use to much processing power



##########~~~~~~~~~~BACKUP TO RESTORE THE PIN DATA IF MESSED UP~~~~~~~~~~##########
pins_scanned = {"ReScan"     : {"counter" : 0 , "angle" :  200 , "dataset" : [   0,   0,   0,   0,       0,  0,  0,  0,  0,  0,       0,  0,  0,  0,  0,  0]} , \
                "Reject"     : {"counter" : 0 , "angle" :  200 , "dataset" : [   0,   0,   0,   0,       0,  0,  0,  0,  0,  0,       0,  0,  0,  0,  0,  0]} , \
                "Black 2L"   : {"counter" : 0 , "angle" : 1700 , "dataset" : [  91, 116,  82, 104,       0,  2,  0,  2,  0,  1,       0,  2,  0,  2,  0,  1]} , \
                "Black 3L"   : {"counter" : 0 , "angle" : 2000 , "dataset" : [ 119, 150, 118, 143,       0,  3,  0,  2,  0,  1,       0,  2,  0,  2,  0,  2]} , \
                "DBG 3L"     : {"counter" : 0 , "angle" : 2450 , "dataset" : [ 130, 150, 100, 142,       0,  5,  2,  5,  0,  2,       1,  4,  2,  5,  0,  7]} , \
                "DBG 1.5L"   : {"counter" : 0 , "angle" :  650 , "dataset" : [  73,  97,  52,  83,       0,  5,  0,  5,  0,  2,       1,  5,  2,  5,  0,  7]} , \
                "Blue 3L"    : {"counter" : 0 , "angle" : 1850 , "dataset" : [ 115, 150, 110, 137,       0,  3,  2,  7,  6, 17,       0,  4,  0,  7,  6, 50]} , \
                "Blue 2L"    : {"counter" : 0 , "angle" : 1550 , "dataset" : [  85, 111,  76,  95,       0,  4,  1,  7,  6, 17,       0,  4,  0,  8, 13, 50]} , \
                "Blue 1.25L" : {"counter" : 0 , "angle" :  950 , "dataset" : [  65,  84,  37,  68,       0,  3,  2,  9,  2, 15,       0,  2,  0,  5,  9, 26]} , \
                "Tan 3L"     : {"counter" : 0 , "angle" : 2300 , "dataset" : [ 120, 153, 107, 149,       9, 15,  7, 13,  2, 11,      12, 25,  9, 20,  8, 32]} , \
                "Tan 2L"     : {"counter" : 0 , "angle" : 1400 , "dataset" : [  88, 111,  83,  97,       9, 15,  7, 13,  2, 11,      11, 25,  9, 20,  7, 32]} , \
                "Tan 1.5L"   : {"counter" : 0 , "angle" :  800 , "dataset" : [  67,  90,  52,  78,     7.2, 14,  5, 11,  1,  8,       9, 19,  5, 16,  7, 20]} , #Often rescans, overlaps 2L a lot \
                "Red 3L"     : {"counter" : 0 , "angle" : 2150 , "dataset" : [ 120, 152, 112, 140,       6, 12,  1,  4,  0,  3,      10, 18,  1,  4,  1,  5]} , \
                "LBG 1.25L"  : {"counter" : 0 , "angle" : 1100 , "dataset" : [  65,  83,  42,  68,       3,  7,  3,  8,  1,  7,       2, 12,  3, 11,  5, 28]} , \
                "LBG 2L"     : {"counter" : 0 , "angle" : 1250 , "dataset" : [  91, 111,  72, 100,       3,  7,  3,  7,  1,  7,       5, 12,  6, 13,  5, 28]} , \
                "LBG 3L"     : {"counter" : 0 , "angle" : 2600 , "dataset" : [ 127, 148, 102, 135,       2,  7,  3,  8,  1,  8,       5, 12,  6, 13,  5, 28]} }

#Backup from 21/12/2022
pins_scanned = {"ReScan"     : {"counter" : 0 , "angle" :  200 , "dataset" : [   0,   0,   0,   0,       0,  0,  0,  0,  0,  0,       0,  0,  0,  0,  0,  0]} , \
                "Reject"     : {"counter" : 0 , "angle" :  200 , "dataset" : [   0,   0,   0,   0,       0,  0,  0,  0,  0,  0,       0,  0,  0,  0,  0,  0]} , \
                "Black 2L"   : {"counter" : 0 , "angle" : 1700 , "dataset" : [  91, 116,  75, 104,       0,  2,  0,  2,  0,  1,       0,  2,  0,  2,  0,  1]} , \
                "Black 3L"   : {"counter" : 0 , "angle" : 1700 , "dataset" : [ 119, 157, 107, 143,       0,  3,  0,  3,  0,  1,       0,  2,  0,  2,  0,  2]} , \
                "DBG 3L"     : {"counter" : 0 , "angle" :  980 , "dataset" : [ 130, 150,  20, 142,       0,  5,  2,  5,  0,  1,       1,  4,  2,  5,  1,  3]} , \
                "DBG 1.5L"   : {"counter" : 0 , "angle" :  880 , "dataset" : [  73, 100,  45,  83,       0,  6,  0,  5,  0,  2,       1,  5,  1,  5,  0,  7]} , \
                "Blue 3L"    : {"counter" : 0 , "angle" : 1920 , "dataset" : [ 115, 150, 115, 137,       0,  3,  2,  7,  4, 17,       0,  4,  0,  7,  4, 50]} , \
                "Blue 2L"    : {"counter" : 0 , "angle" : 1820 , "dataset" : [  85, 111,  70, 115,       0,  4,  1,  7,  3, 17,       0,  4,  0,  8,  4, 50]} , \
                "Blue 1.25L" : {"counter" : 0 , "angle" : 1700 , "dataset" : [  46,  84,  20,  70,       0,  3,  2,  9,  2, 15,       0,  2,  0,  5,  4, 26]} , \
                "Tan 3L"     : {"counter" : 0 , "angle" : 1600 , "dataset" : [ 120, 153, 107, 149,      15, 21, 10, 15,  2, 11,       9, 25,  5, 20,  6, 32]} , \
                "Tan 2L"     : {"counter" : 0 , "angle" : 1500 , "dataset" : [  88, 111,  74,  99,      15, 21, 10, 15,  3, 11,       9, 25,  5, 20,  6, 32]} , \
                "Tan 1.5L"   : {"counter" : 0 , "angle" : 1400 , "dataset" : [  67,  90,  52,  78,      13, 21,  8, 11,  2, 11,       4, 25,  2, 20,  4, 25]} , \
                "Red 3L"     : {"counter" : 0 , "angle" : 2020 , "dataset" : [ 120, 152, 100, 140,       6, 18,  1,4.2,  0,  3,       5, 18,  1,  4,  0,  6]} , \
                "LBG 1.25L"  : {"counter" : 0 , "angle" : 1100 , "dataset" : [  50,  83,  42,  78,       3, 12,  3, 12,  1,  8,       2, 12,  3, 11,  2, 28]} , \
                "LBG 2L"     : {"counter" : 0 , "angle" : 1200 , "dataset" : [  91, 111,  80, 105,       3, 11,  3, 11,  1,  8,       3, 12,  4, 13,  3, 28]} , \
                "LBG 3L"     : {"counter" : 0 , "angle" : 1300 , "dataset" : [ 115, 148, 106, 145,       3,  8,  7,  9,  1,  7,       4, 12,  4, 13,  4, 28]} }


#28/12/22
pins_scanned = {"ReScan"     : {"counter" : 0 , "angle" :  200 , "dataset" : [   0,   0,   0,   0,       0,  0,  0,  0,  0,  0,       0,  0,  0,  0,  0,  0]} , \
                "Reject"     : {"counter" : 0 , "angle" :  200 , "dataset" : [   0,   0,   0,   0,       0,  0,  0,  0,  0,  0,       0,  0,  0,  0,  0,  0]} , \
                "Black 2L"   : {"counter" : 0 , "angle" : 2140 , "dataset" : [  91, 120,  50,  98,       1,  3,  2,  4,  0,  1,       0,  1,  0,  1,  0,  0]} , \
                "Black 3L"   : {"counter" : 0 , "angle" : 2260 , "dataset" : [ 130, 157, 110, 143,       1,  3,  2,  4,  0,  1,       0,  1,  0,  1,  0,  0]} , \
                "DBG 3L"     : {"counter" : 0 , "angle" :  980 , "dataset" : [ 112, 157,  30, 151,       5,  7,  6,  8,  2,  4,       1,  4,  2,  4,  1,  3]} , \
                "DBG 1.5L"   : {"counter" : 0 , "angle" :  880 , "dataset" : [  73, 100,  50,  89,       5,  8,  4,  8,  0,  4,       1,  4,  2,  4,  1,  3]} , \
                "Blue 3L"    : {"counter" : 0 , "angle" : 1920 , "dataset" : [ 130, 157, 107, 143,       2,  6,  5, 10,  8, 14,       0,  2,  3,  5,  8, 18]} , \
                "Blue 2L"    : {"counter" : 0 , "angle" : 1820 , "dataset" : [  95, 116,  70, 100,       2,  6,  5, 10,  7, 16,       0,  2,  3,  5,  8, 24]} , \
                "Blue 1.25L" : {"counter" : 0 , "angle" : 1700 , "dataset" : [  66,  90,  40,  76,       2,  6,  5, 10,  7, 16,       0,  2,  1,  5,  5, 24]} , \
                "Tan 3L"     : {"counter" : 0 , "angle" : 1600 , "dataset" : [ 128, 171, 114, 159,      18, 24, 13, 20,  7, 10,      10, 16,  9, 14,  3, 11]} , \
                "Tan 2L"     : {"counter" : 0 , "angle" : 1500 , "dataset" : [  95, 115,  79, 106,      18, 28, 13, 25,  6, 10,      10, 16,  8, 15,  3, 12]} , \
                "Tan 1.5L"   : {"counter" : 0 , "angle" : 1400 , "dataset" : [  77,  89,  63,  79,      18, 28, 13, 25,  4, 10,      10, 16,  6, 15,  5, 12]} , \
                "Red 3L"     : {"counter" : 0 , "angle" : 2020 , "dataset" : [ 116, 157, 110, 149,      14, 21,  4,  8,  0,  2,       9, 15,  1,  3,  0,  2]} , \
                "LBG 1.25L"  : {"counter" : 0 , "angle" : 1100 , "dataset" : [  66,  90,  45,  74,       9, 14,  9, 16,  6, 11,       2,  8,  1,  9,  4, 11]} , \
                "LBG 2L"     : {"counter" : 0 , "angle" : 1200 , "dataset" : [  95, 120,  80, 105,       9, 14,  9, 16,  6, 11,       5,  9,  5, 10,  3, 12]} , \
                "LBG 3L"     : {"counter" : 0 , "angle" : 1300 , "dataset" : [ 133, 159, 117, 145,       9, 14,  9, 16,  6, 11,       3, 10,  3, 11,  2, 17]} }