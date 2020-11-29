# JJROBOTS PYBOT SCARA PYTHON DESKTOP APP
#
# This application allows you to control the pyBOT SCARA ARM robot. It implements trajectory-based control, real-time control and through LEAP Motion.
# An artificial vision module based on colorimetry is also implemented. This module allows the robot to capture objects through a camera.
# 
# author: JJROBOTS 2018-2020
# APP version: 1.32 (12/01/2020)
# Licence: Open Source (GNU LGPLv3)

from Tkinter import *
import Tkinter
import ttk

import tkMessageBox

import tkFileDialog as tkfd


from PIL import Image, ImageTk, ImageChops, ImageOps

import os
import glob
import json
import csv
import sys

import numpy as np



import io
import inspect
import thread

import math
import time
import signal

import platform

import cv2
import colorsys

from random import seed,randint

import serial.tools.list_ports

from PyBotArm import PyBotArm

src_dir = os.path.dirname(inspect.getfile(inspect.currentframe()))

if platform.system() != 'Windows':
    arch_dir = os.path.abspath(os.path.join(src_dir, './LeapSDK/lib'))
else:
    arch_dir = os.path.abspath(os.path.join(src_dir, './LeapSDK_WIN/lib'))
    arch_dir2 = os.path.abspath(os.path.join(src_dir, './LeapSDK_WIN/lib/x64'))
    sys.path.insert(0, os.path.abspath(os.path.join(src_dir, arch_dir2)))
    
sys.path.insert(0, os.path.abspath(os.path.join(src_dir, arch_dir)))

try:  # JJ Check leap libraries
    import Leap
    from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture
    LEAP = True
except:
    print("Error imporing leap library")
    LEAP = False

try:
    import pygame
    from pygame.locals import *
    GAMEPAD = True
except:
    print("Error importing pygame library for GAMEPAD controll")
    GAMEPAD = False
    
import threading
from threading import Timer,Thread

########## BLOCKLY INTERFACE ################
from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
import urlparse

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm


HTTP2UDP_PORT = 8008
PORT_NUMBER = 8080

# Telemetry port
UDP_PORT = 2223

# Blockly Robot interface
b_myRobot = None

# Class to launch servers on a different threads
class ServerThread(threading.Thread):
    def __init__(self,server):
        threading.Thread.__init__(self)
        self.server=server

    def run(self):        
        self.server.serve_forever()

    def close(self):        
        self.server.shutdown()
        

#Browser to handle HTTP to UDP conversion
class HTTP2UDP(BaseHTTPRequestHandler):
    # Override address_string to avoid using name lookup
    NODATA = -20000
    def address_string(self):
        host, port = self.client_address[:2]
        #return socket.getfqdn(host)
        return host
    def do_GET(self):
        #print("GET request")
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin","*")
        self.send_header("Content-type", "text/html")
        self.end_headers()
        params = self.path
        params = params.split('?')
        if (len(params)>1):
            params = urlparse.parse_qs(params[1])
            if 'JJAM' in params.keys():
                print "JJAM:"+params['JJAM'][0]
                mvparams = params['JJAM'][0].split(",")
                ch1 = self.NODATA if mvparams[0]=='' else int(mvparams[0])
                ch2 = self.NODATA if mvparams[1]=='' else int(mvparams[1])
                ch3 = self.NODATA if mvparams[2]=='' else int(mvparams[2])
                ch4 = self.NODATA if mvparams[3]=='' else int(mvparams[3])
                ch5 = self.NODATA if mvparams[4]=='' else int(mvparams[4])
                t = 0 if mvparams[5]=='' else int(mvparams[5])
                s = True if mvparams[5]=='' else bool(mvparams[6])
                b_myRobot.setXYZ45(ch1,ch2,ch3,ch4,ch5)
                
            if 'JJAI' in params.keys():
                print "JJAI:"+params['JJAI'][0]
                mvparams = params['JJAI'][0].split(",")
                ch1 = self.NODATA if mvparams[0]=='' else int(mvparams[0])
                ch2 = self.NODATA if mvparams[1]=='' else int(mvparams[1])
                ch3 = self.NODATA if mvparams[2]=='' else int(mvparams[2])
                ch4 = self.NODATA if mvparams[3]=='' else int(mvparams[3])
                ch5 = self.NODATA if mvparams[4]=='' else int(mvparams[4])
                t = 0 if mvparams[5]=='' else int(mvparams[5])
                s = True if mvparams[5]=='' else bool(mvparams[6])
                b_myRobot.setXYZ45(ch1,ch2,ch3,ch4,ch5)
            
        self.wfile.write('OK')
        return

    def log_request(self, code=None, size=None):
        pass   # Do nothing...
        #print('Request')



#############################################
### Class for Time related tasks
#############################################
#############################################
class Timeout():
    """Timeout class using ALARM signal."""
    class Timeout(Exception):
        pass

    def __init__(self, sec):
        self.sec = sec

    def __enter__(self):
        signal.signal(signal.SIGALRM, self.raise_timeout)
        signal.alarm(self.sec)

    def __exit__(self, *args):
        signal.alarm(0)    # disable alarm

    def raise_timeout(self, *args):
        raise Timeout.Timeout()


class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.next_call = time.time()
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self.next_call += self.interval
            self._timer = Timer(self.next_call - time.time(), self._run)
            self._timer.start()
            self.is_running = True
         
    def stop(self):
        self._timer.cancel()
        self.is_running = False

        
#############################################
### Deprecated Class. Not USED
#############################################
#############################################
class InfoFrame(Tkinter.Toplevel):
    def __init__(self):
        """Constructor"""
        Tkinter.Toplevel.__init__(self)
        self.geometry("400x250")
        self.title("JJROBOTS (C)2018. SCARA Robot ARM")

        self.resizable(width=FALSE, height=FALSE)
        # create all of the main containers
        mycolor = '#001e38'
        self.top_frame = Frame(self, bg=mycolor, width=400, height=150)#, padx=3, pady=3)
        
        self.top_frame.grid(row=0, column=0, sticky="nsew")

        timg = Image.open("about.png")
        timg = timg.resize((400,150), Image.ANTIALIAS)

        self.im = ImageTk.PhotoImage(timg)
        self.canvas = Canvas(self.top_frame, width=400, height=150, highlightthickness=0, highlightbackground="black")
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.canvas_Image = self.canvas.create_image(0, 0, image=self.im, anchor="nw")

        self.top_frame2 = Frame(self, bg="grey", width=200, height=100)#, padx=3, pady=3)
        self.top_frame2.grid(row=1, column=0, sticky="nsew")
        self.top_frame3 = Frame(self, bg="grey", width=200, height=100)#, padx=3, pady=3)
        self.top_frame3.grid(row=1, column=1, sticky="e")

        self.label0 = Label(self.top_frame2, text='jjRobots (C)2018', fg="white", bg="gray", anchor="w", justify="left")
        self.label1 = Label(self.top_frame2, text='SCARA Robot ARM', fg="white", bg="gray", anchor="w", justify="left")
        self.label2 = Label(self.top_frame2, text='(Datos relevantes, ...)', fg="white", bg="gray", anchor="w", justify="left")
        
        # layout the widgets in the top frame
        self.label0.grid(row=0, column=0, sticky="w")
        self.label1.grid(row=1, column=0, sticky="w")
        self.label2.grid(row=2, column=0, sticky="w")

        
##############################################        
### Class to implement Calibration dialog
##############################################
##############################################
class CalibrationDialog(Tkinter.Toplevel):
    
    mw = None
    image = None
    _image = None
    point_status = 0

    points = {}
    z_points = {}

    angle = 0

    rt = None

    status = 0

    _color = 'Yellow'
    
    def __init__(self, parent, values):
        
        Tkinter.Toplevel.__init__(self)#, parent)
        self.mw = parent
        
        _twidth = 390 + self.mw.camX
        _theight = 165 + self.mw.camY
        
        if platform.system() == 'Windows':
            self.geometry("890x545")
            self.iconbitmap('./icons/scara_app.ico')
        else:
            self.geometry("930x545")

        self.points = self.mw.points
        
        self.title("SCARA CALIBRATION")
        self.resizable(width=FALSE, height=FALSE)
        self.configure(background='black')
        
        self.mid_frame = Frame(self, highlightbackground="black", highlightcolor="black", highlightthickness=2, bg="gray", width=375, height=150, padx=3, pady=3)
        self.mid_frame.grid_rowconfigure(0, weight=1)
        self.mid_frame.grid_columnconfigure(0, weight=1)
        self.mid_frame.grid(row=1, column=0, sticky="nsew")

        self.buttons_frame = Frame(self, bg="black", width=375, height=100, padx=3, pady=3)
        self.buttons_frame.grid_rowconfigure(0, weight=1)
        self.buttons_frame.grid_columnconfigure(0, weight=1)
        self.buttons_frame.grid(row=2, column=0, sticky="nsew")

        self.zOffset = Tkinter.Scale(self.mid_frame, highlightbackground="black", highlightcolor="black", highlightthickness=2,from_=10, to=-10, bg="gray40", label="Z Offset", command=self.update)
        self.zOffset.pack(side=LEFT, padx = 2, pady = 10)
        self.zOffset.set(self.mw.zOFFSET)
        
        self.pinOffset = Tkinter.Scale(self.mid_frame, highlightbackground="black", highlightcolor="black", highlightthickness=2,from_=-90, to=90, bg="gray40", label="Clamp Offset", command=self.update)
        self.pinOffset.pack(side=LEFT, padx = 2, pady = 10)
        self.pinOffset.set(self.mw.pinOFFSET)

        self.canvas_max_area = Canvas(self.mid_frame, width=self.mw.camX, height=self.mw.camY, bg="gray90" ,highlightthickness=2, highlightbackground="black")
        self.canvas_max_area.pack(side=RIGHT, padx = 2, pady = 10)

        self.canvas_max_area.configure(cursor='dotbox')
        
        
        
        if self.mw.CAMERA:# and (self.frame != None): #and (self.control_mode == 1):
            if self.mw.frame:
                self.image = Image.fromarray(self.mw.image_pipe["equalized"])
                self._image = ImageTk.PhotoImage(self.image)
                    
                self.canvasImage = self.canvas_max_area.create_image(2, 2, image=self._image, anchor="nw")
                self.canvas_max_area.create_text(110,30, fill=self._color, text="ALIGN THE TEMPLATE WITH THE ROBOT AND CLICK Get Frame...", anchor='nw')
                self.canvas_max_area.create_text(110,45, fill=self._color, text="Click on point (-100,100)", anchor='nw')

                
                
        else:
            self.canvas_max_area.create_text(200,100,fill="blue", text="Activate VISION MODE to calibrate")
            

        if self.mw.CAMERA:
            self.canvas_max_area.bind("<Button-1>", self.click)
            
        ### BUTTONS ###
        self.recall_button = Tkinter.Button(self.buttons_frame, state=DISABLED, highlightbackground="black", highlightcolor="black", highlightthickness=1 ,bg="gray", text=" X-Y Auto Callibration ", command=self.recall)
        self.recall_button.pack(side=LEFT,padx=4)

        if self.mw.ROBOT:
            self.recall_button['state'] = 'normal'
        
        self.ok_button = Tkinter.Button(self.buttons_frame, highlightbackground="black", highlightcolor="black", highlightthickness=1 ,bg="gray", text="   OK   ", command=self.on_ok)
        self.ok_button.pack(side=RIGHT,padx=4)

        self.frame_button = Tkinter.Button(self.buttons_frame, state=DISABLED, highlightbackground="black", highlightcolor="black", highlightthickness=1 ,bg="gray", text=" Get Frame ", command=self.update_frame)
        self.frame_button.pack(side=RIGHT,padx=4)
        if self.mw.CAMERA:
            self.frame_button['state'] = 'normal'

        self.ZCal_button = Tkinter.Button(self.buttons_frame, highlightbackground="black", highlightcolor="black", highlightthickness=1 ,bg="gray", text=" Calibrate Z ", command=self.calibrate_z)
        self.ZCal_button.pack(side=LEFT,padx=4)
        self.ZCal_next_button = Tkinter.Button(self.buttons_frame, highlightbackground="black", highlightcolor="black", highlightthickness=1 ,bg="gray", text=" Calibrate Z ", command=self.calibrate_z)
        
            
        self.protocol('WM_DELETE_WINDOW', self.on_closing)
        self.bind("<Key>", self.keypress)
        
    def keypress(self, event):
        self.status += 1
        
    def calibrate_z(self):

        
        if self.mw.ROBOT:
            print("Calibrating Z...")
            self.canvas_max_area.delete('all')
            self.canvas_max_area.create_text(200,100,fill="blue", text="Calibrating Z...")
            
            self.mw._pANGLE = (self.mw.translate(self.mw.robotPIN+self.mw.pinOFFSET,-65,65,0,1000))
            self.mw.myRobot.moveXYZ(0,200,0,self.mw._pANGLE,self.mw._pOPEN, t=0.0, sync=False, elbow = self.mw._elbow)

            ### Z axis calibration sequence...
            self.canvas_max_area.create_text(200,150,fill="blue", text="Adjust Z in actual position (200,0) and press Enter key.")
            
            while self.status == 0:
                self.mw.draw_bot()
                root.update_idletasks()
                root.update()

            self.z_points[self.status - 1] = self.mw.zOFFSET

            self.mw._pANGLE = (self.mw.translate(self.mw.robotPIN+self.mw.pinOFFSET,-65,65,0,1000))
            self.mw.myRobot.moveXYZ(0,150,0,self.mw._pANGLE,self.mw._pOPEN, t=0.0, sync=False, elbow = self.mw._elbow)
            self.canvas_max_area.create_text(200,170,fill="blue", text="Adjust Z in actual position (150,0) and press Enter key.")
            
            while self.status == 1:
                self.mw.draw_bot()
                root.update_idletasks()
                root.update()

            self.z_points[self.status - 1] = self.mw.zOFFSET
            
            self.mw._pANGLE = (self.mw.translate(self.mw.robotPIN+self.mw.pinOFFSET,-65,65,0,1000))
            self.mw.myRobot.moveXYZ(0,100,0,self.mw._pANGLE,self.mw._pOPEN, t=0.0, sync=False, elbow = self.mw._elbow)
            self.canvas_max_area.create_text(200,190,fill="blue", text="Adjust Z in actual position (100,0) and press Enter key.")
            
            while self.status == 2:
                self.mw.draw_bot()
                root.update_idletasks()
                root.update()

            self.z_points[self.status - 1] = self.mw.zOFFSET

            ### Return to original position
            self.canvas_max_area.create_text(200,210,fill="blue", text="Z axis calibration end.")
            self.canvas_max_area.create_text(200,230,fill="blue", text="Z Offset [200,0] = {}".format(str(self.z_points[0])))
            self.canvas_max_area.create_text(200,250,fill="blue", text="Z Offset [150,0] = {}".format(str(self.z_points[1])))
            self.canvas_max_area.create_text(200,270,fill="blue", text="Z Offset [100,0] = {}".format(str(self.z_points[2])))


            
            self.mw.myRobot.moveXYZ((self.mw.X*10),(self.mw.Y*10),self.mw.Z,self.mw._pANGLE,self.mw._pOPEN, t=0.0, sync=False, elbow = self.mw._elbow)

            self.mw.z_points = self.z_points

        else:
            print("Connect pyBot to calibrate Z axis.")
            self.canvas_max_area.delete('all')
            self.canvas_max_area.create_text(200,100,fill="blue", text="Connect pyBot to calibrate Z axis.")
        
    def update_frame(self):

        _color = 'Yellow'
        self.canvas_max_area.delete('all')
        
        if self.mw.CAMERA:# and (self.frame != None): #and (self.control_mode == 1):
            if self.mw.frame:
                self.image = Image.fromarray(self.mw.image_pipe["equalized"])
                self._image = ImageTk.PhotoImage(self.image)
                    
                self.canvasImage = self.canvas_max_area.create_image(2, 2, image=self._image, anchor="nw")

        for i in range(0,self.point_status):
            self.canvas_max_area.create_oval(self.points[i][0] - 5, self.points[i][1] - 5, self.points[i][0] + 5, self.points[i][1] + 5)

        if self.point_status == 0:
            
            self.canvas_max_area.create_text(110,30, fill=_color, text="ALIGN THE TEMPLATE WITH THE ROBOT...", anchor='nw')
            self.canvas_max_area.create_text(110,45, fill=_color, text="Click on point (-100,100)", anchor='nw')
            
               
        if self.point_status == 1:
            
            self.canvas_max_area.create_text(110,30, fill=_color, text="ALIGN THE TEMPLATE WITH THE ROBOT...", anchor='nw')
            self.canvas_max_area.create_text(110,45, fill=_color, text="Click on point (-100,100)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,60, fill=_color, text="Click on point ( 100,100)", anchor='nw')

            

        if self.point_status == 2:
            
            self.canvas_max_area.create_text(110,30, fill=_color, text="ALIGN THE TEMPLATE WITH THE ROBOT...", anchor='nw')
            self.canvas_max_area.create_text(110,45, fill=_color, text="Click on point (-100,100)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,60, fill=_color, text="Click on point ( 100,100)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,75, fill=_color, text="Click on point (   0,150)", anchor='nw')
            

        if self.point_status == 3:
            
            self.canvas_max_area.create_text(110,30, fill=_color, text="ALIGN THE TEMPLATE WITH THE ROBOT...", anchor='nw')
            self.canvas_max_area.create_text(110,45, fill=_color, text="Click on point (-100,100)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,60, fill=_color, text="Click on point ( 100,100)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,75, fill=_color, text="Click on point (   0,150)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,90, fill=_color, text="Click on point (   0, 50)", anchor='nw')

        if self.point_status == 4:
            
            self.canvas_max_area.create_text(110,30, fill=_color, text="ALIGN THE TEMPLATE WITH THE ROBOT...", anchor='nw')
            self.canvas_max_area.create_text(110,45, fill=_color, text="Click on point (-100,100)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,60, fill=_color, text="Click on point ( 100,100)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,75, fill=_color, text="Click on point (   0,150)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,90, fill=_color, text="Click on point (   0, 50)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,105, fill=_color, text="Click on point (   0,100)\t", anchor='nw')


        if self.point_status == 5:
            self.canvas_max_area.create_text(110,30, fill=_color, text="ALIGN THE TEMPLATE WITH THE ROBOT...", anchor='nw')
            self.canvas_max_area.create_text(110,45, fill=_color, text="Click on point (-100,100)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,60, fill=_color, text="Click on point ( 100,100)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,75, fill=_color, text="Click on point (   0,150)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,90, fill=_color, text="Click on point (   0, 50)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,105, fill=_color, text="Click on point (   0,100)\t - OK -", anchor='nw')                               
            self.canvas_max_area.create_text(110,120, fill=_color, text="CALIBRATION END", anchor='nw')

            self.canvas_max_area.create_text(110,210, fill=_color, text="X pixel size: " + str(self.mw.pixelX_size), anchor='nw')
            self.canvas_max_area.create_text(110,225, fill=_color, text="Y pixel size: " + str(self.mw.pixelY_size), anchor='nw')
            self.canvas_max_area.create_text(110,240, fill=_color, text="Rotation angle: " + str(math.degrees(self.angle)), anchor='nw')
        
    def click(self, event):


        _color = 'Yellow'
        
        
        self.points[self.point_status] = [event.x,event.y]

        if self.point_status == 0:
            
            self.canvas_max_area.create_text(110,30, fill=_color, text="ALIGN THE TEMPLATE WITH THE ROBOT...", anchor='nw')
            self.canvas_max_area.create_text(110,45, fill=_color, text="Click on point (-100,100)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,60, fill=_color, text="Click on point ( 100,100)", anchor='nw')

            

        if self.point_status == 1:
            
            self.canvas_max_area.create_text(110,30, fill=_color, text="ALIGN THE TEMPLATE WITH THE ROBOT...", anchor='nw')
            self.canvas_max_area.create_text(110,45, fill=_color, text="Click on point (-100,100)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,60, fill=_color, text="Click on point ( 100,100)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,75, fill=_color, text="Click on point (   0,150)", anchor='nw')
            

        if self.point_status == 2:
            
            self.canvas_max_area.create_text(110,30, fill=_color, text="ALIGN THE TEMPLATE WITH THE ROBOT...", anchor='nw')
            self.canvas_max_area.create_text(110,45, fill=_color, text="Click on point (-100,100)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,60, fill=_color, text="Click on point ( 100,100)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,75, fill=_color, text="Click on point (   0,150)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,90, fill=_color, text="Click on point (   0, 50)", anchor='nw')

        if self.point_status == 3:
            
            self.canvas_max_area.create_text(110,30, fill=_color, text="ALIGN THE TEMPLATE WITH THE ROBOT...", anchor='nw')
            self.canvas_max_area.create_text(110,45, fill=_color, text="Click on point (-100,100)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,60, fill=_color, text="Click on point ( 100,100)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,75, fill=_color, text="Click on point (   0,150)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,90, fill=_color, text="Click on point (   0, 50)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,105, fill=_color, text="Click on point (   0,100)\t", anchor='nw')


        if self.point_status == 4:
            self.canvas_max_area.create_text(110,30, fill=_color, text="ALIGN THE TEMPLATE WITH THE ROBOT...", anchor='nw')
            self.canvas_max_area.create_text(110,45, fill=_color, text="Click on point (-100,100)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,60, fill=_color, text="Click on point ( 100,100)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,75, fill=_color, text="Click on point (   0,150)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,90, fill=_color, text="Click on point (   0, 50)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,105, fill=_color, text="Click on point (   0,100)\t - OK -", anchor='nw')
            self.canvas_max_area.create_text(110,120, fill=_color, text="CALIBRATION END", anchor='nw')
                        
            self.mw.points = self.points


            self.mw.mV = (self.points[3][1] - self.points[2][1]) / float((self.points[3][0] - self.points[2][0])+0.01)
            self.mw.mH = (self.points[1][1] - self.points[0][1]) / float((self.points[1][0] - self.points[0][0])+0.01)

            _hp = math.sqrt( ((self.points[0][0] - self.points[1][0])*(self.points[0][0] - self.points[1][0])) + ((self.points[0][1] - self.points[1][1])*(self.points[0][1] - self.points[1][1])) )

            if ((self.points[1][0] - self.points[0][0])+0.01) < 0:
                self.mw.pixelX_size = (200 / float(_hp)) * -1
            else:
                self.mw.pixelX_size = (200 / float(_hp))
                

            if ((self.points[3][1] - self.points[2][1])+0.01) < 0:
                self.mw.pixelY_size = (200 / float(_hp)) * -1
            else:
                self.mw.pixelY_size = (200 / float(_hp))

            a = _hp/float(2)
            b = self.points[1][0] - self.points[4][0]
            c = (self.mw.camY - self.points[1][1]) - (self.mw.camY - self.points[4][1])
            
            number = np.clip((a*a + b*b - c*c) / (2 * a * b),-1.0,1.0)
            self.angle = math.acos(number)

            if self.points[1][1] > self.points[0][1]:
                self.angle = -1 * self.angle

            
            self.mw.cam_angle = self.angle
            
            self.canvas_max_area.create_text(110,210, fill=_color, text="X pixel size: " + str(self.mw.pixelX_size), anchor='nw')
            self.canvas_max_area.create_text(110,225, fill=_color, text="Y pixel size: " + str(self.mw.pixelY_size), anchor='nw')
            self.canvas_max_area.create_text(110,240, fill=_color, text="Rotation angle: " + str(math.degrees(self.angle)), anchor='nw')
            

        
        if self.point_status < 5:  
            self.canvas_max_area.create_oval(event.x - 5, event.y - 5, event.x + 5, event.y + 5)
            self.point_status += 1
   

        
    def on_closing(self, event=None):
        self.mw.CalibrationDialog_show = False
        self.destroy()
        
    def recall(self, event=None):
        if self.mw.ROBOT:
            self.mw.tZ = 0
            self.mw.myRobot.motorsCalibration()
            self.mw.myRobot.setSpeedAcc(self.mw.param_dict["xy_speed"],self.mw.param_dict["z_speed"],self.mw.param_dict["xy_accel"],self.mw.param_dict["z_accel"])
            

    def update(self, event=None):
        self.mw.pinOFFSET = self.pinOffset.get()
        self.mw.zOFFSET = self.zOffset.get()

        if self.mw.ROBOT:
            _pANGLE = int(self.mw.translate(self.mw.robotPIN+self.mw.pinOFFSET,-65,65,0,1000))
            self.mw.myRobot.moveAxis4(_pANGLE,t=0.01, sync=False)
            #print("calibratin Z: ", self.mw.Z+self.mw.zOFFSET)
            self.mw.myRobot.moveAxis3(self.mw.Z+self.mw.zOFFSET,t=0.01, sync=False)
            
        
    def on_ok(self, event=None):
        self.mw.pinOFFSET = self.pinOffset.get()
        self.mw.zOFFSET = self.zOffset.get()
        self.mw.points = self.points

        
        ### save to file ###
        self.mw.callibration['z'] = self.mw.zOFFSET
        self.mw.callibration['pin'] = self.mw.pinOFFSET
        self.mw.callibration['points'] = self.mw.points
        self.mw.callibration['z_points'] = self.mw.z_points
        self.mw.callibration['pixelX_size'] = self.mw.pixelX_size
        self.mw.callibration['pixelY_size'] = self.mw.pixelY_size
        self.mw.callibration['mH'] = self.mw.mH
        self.mw.callibration['mV'] = self.mw.mV
        self.mw.callibration['cam_angle'] = self.mw.cam_angle
        
        
        
        tmp = json.dumps(self.mw.callibration)
        f = open("calibration.json","w")
        f.write(tmp)
        f.close()
        
        self.mw.CalibrationDialog_show = False
        
        self.destroy()

    def show(self):
        self.wm_deiconify()
        self.ok_button.focus_force()

       
##############################################        
### Class to implement SCARA Configuration dialog  
##############################################
##############################################            
class CustomDialog(Tkinter.Toplevel):

    param_dict = {}

    mw = None
    lidarVar = None
    videoVar = None
    
    def __init__(self, parent, values):
        
        Tkinter.Toplevel.__init__(self)#, parent)
        if platform.system() == 'Windows':
            self.geometry("490x355")
            self.iconbitmap('./icons/scara_app.ico')
        else:
            self.geometry("595x350")

        self.mw = parent

        self.lidarVar = Tkinter.BooleanVar()
        self.lidarVar.set(self.mw.LIDAR)
        self.videoVar = Tkinter.BooleanVar()
        self.videoVar.set(self.mw.VIDEO)
        
        self.title("SCARA PARAMETERS CONFIGURATION")
        self.resizable(width=FALSE, height=FALSE)
        self.configure(background='black')

        self.connect_frame = Frame(self, highlightbackground="black", highlightcolor="black", highlightthickness=2, bg="gray", width=525, height=150, padx=3, pady=3)
        self.connect_frame.grid_rowconfigure(0, weight=1)
        self.connect_frame.grid_columnconfigure(0, weight=1)
        self.connect_frame.grid(row=0, column=0, sticky="nsew")
        
        self.robot_frame = Frame(self, highlightbackground="black", highlightcolor="black", highlightthickness=2, bg="gray", width=525, height=150, padx=3, pady=3)
        self.robot_frame.grid_rowconfigure(0, weight=1)
        self.robot_frame.grid_columnconfigure(0, weight=1)
        self.robot_frame.grid(row=1, column=0, sticky="nsew")

        self.screen_frame = Frame(self, highlightbackground="black", highlightcolor="black", highlightthickness=2, bg="gray", width=525, height=150, padx=3, pady=3)
        self.screen_frame.grid_rowconfigure(0, weight=1)
        self.screen_frame.grid_columnconfigure(0, weight=1)
        self.screen_frame.grid(row=2, column=0, sticky="nsew")

        self.buttons_frame = Frame(self, bg="black", width=525, height=100, padx=3, pady=3)
        self.buttons_frame.grid_rowconfigure(0, weight=1)
        self.buttons_frame.grid_columnconfigure(0, weight=1)
        self.buttons_frame.grid(row=3, column=0, sticky="nsew")
    
        
        labelRobot = Tkinter.Label(self.connect_frame, text="Port", bg="gray")
        labelRobot.pack(side=LEFT)

        values_ports = [comport.device for comport in serial.tools.list_ports.comports()]
        values_ports.append("WIFI")
        
        self.combo = ttk.Combobox(self.connect_frame, width=25, state="readonly", values=values_ports)
        self.combo.set(values["connection"])
        self.combo.pack(side=LEFT)
        

        self.buttonLIDAR=Checkbutton(self.connect_frame, bg='gray', text=" LIDAR",height = 2, width = 16, variable = self.lidarVar, anchor = "w", command=self.on_update)
        self.buttonLIDAR.pack(side=RIGHT)
        self.buttonVIDEO=Checkbutton(self.connect_frame, bg='gray', text=" VIDEO",height = 2, width = 16, variable = self.videoVar, anchor = "w", command=self.on_update)
        self.buttonVIDEO.pack(side=RIGHT)
        
        self.xy_speed = Tkinter.Scale(self.robot_frame, highlightbackground="black", highlightcolor="black", highlightthickness=2,from_=100, to=0,bg="gray40", label="XY Speed")
        self.xy_speed.pack(side=LEFT, padx = 2, pady = 10)
        
        self.xy_speed.set(values["xy_speed"])

        self.xy_acc = Tkinter.Scale(self.robot_frame, highlightbackground="black", highlightcolor="black", highlightthickness=2,from_=100, to=0, bg="gray40", label = "XY Accel.")
        self.xy_acc.pack(side=LEFT, padx = 2, pady = 10)
        self.xy_acc.set(values["xy_accel"])

        self.z_acc = Tkinter.Scale(self.robot_frame, highlightbackground="black", highlightcolor="black", highlightthickness=2,from_=100, to=0, bg="gray40", label = "Z Accel.")
        self.z_acc.pack(side=RIGHT, padx = 2, pady = 10)
        self.z_acc.set(values["z_accel"])

        self.z_speed = Tkinter.Scale(self.robot_frame, highlightbackground="black", highlightcolor="black", highlightthickness=2,from_=100, to=0, bg="gray40", label = "Z Speed")
        self.z_speed.pack(side=RIGHT, padx = 2, pady = 10)
        self.z_speed.set(values["z_speed"])

        ### SCREEN ###
        self.scale = Tkinter.Scale(self.screen_frame, highlightbackground="black", highlightcolor="black", highlightthickness=2,from_=100, to=10, bg="gray40", label="Scale Factor", command=self.on_update)
        self.scale.pack(side=LEFT, padx = 2, pady = 10)
        self.scale.set(values["SCALE"])

        self.yOffset = Tkinter.Scale(self.screen_frame, highlightbackground="black", highlightcolor="black", highlightthickness=2,from_=-8, to=8, bg="gray40", label = "Y Screen offset", command=self.on_update)
        self.yOffset.pack(side=LEFT, padx = 2, pady = 10)
        self.yOffset.set(values["yOFFSET"])

        self.scanAREA = Tkinter.Scale(self.screen_frame, highlightbackground="black", highlightcolor="black", highlightthickness=2,from_=5, to=50, bg="gray40", label = "Scan Area", command=self.on_update)
        self.scanAREA.pack(side=LEFT, padx = 2, pady = 10)
        self.scanAREA.set(self.mw._scanRange)
        
        ### BUTTONS ###
        self.ok_button = Tkinter.Button(self.buttons_frame, highlightbackground="black", highlightcolor="black", highlightthickness=1 ,bg="gray", text="   OK   ", command=self.on_ok)
        self.ok_button.pack(side=RIGHT,padx=4)

        self.default_button = Tkinter.Button(self.buttons_frame, highlightbackground="black", highlightcolor="black", highlightthickness=1 ,bg="gray", text=" Default ", command=self.on_default)
        self.default_button.pack(side=RIGHT,padx=4)
        
        
        
        self.protocol('WM_DELETE_WINDOW', self.on_closing)

    def on_default(self, event=None):
        self.mw.param_dict["xy_speed"] = 100
        self.mw.param_dict["z_speed"] = 100
        self.mw.param_dict["xy_accel"] = 100
        self.mw.param_dict["z_accel"] = 100
        self.mw.param_dict["connection"] = "WIFI"
        self.mw.param_dict["LIDAR"] = True
        self.mw.param_dict["VIDEO"] = True
        self.mw.param_dict["SCALE"] = 32
        self.mw.param_dict["yOFFSET"] = 6
        self.mw.param_dict["scanAREA"] = 25

        self.xy_speed.set(100)
        self.z_speed.set(100)
        self.xy_acc.set(100)
        self.z_acc.set(100)
        self.combo.set("WIFI")
        self.lidarVar.set(True)
        self.videoVar.set(True)
        self.scanAREA.set(25)

        self.scale.set(32)
        self.yOffset.set(6)

        self.mw.LIDAR = self.lidarVar.get()
        self.mw.VIDEO = self.videoVar.get()

        self.mw.SCREEN_SCALE = self.scale.get()
        self.mw.yShift = self.yOffset.get()

        self.mw._scanRange = self.scanAREA.get()
        
        self.mw.configure(event=None)
        
        
        
    def on_update(self, event=None):
        self.mw.param_dict["xy_speed"] = self.xy_speed.get()
        self.mw.param_dict["z_speed"] = self.z_speed.get()
        self.mw.param_dict["xy_accel"] = self.xy_acc.get()
        self.mw.param_dict["z_accel"] = self.z_acc.get()
        self.mw.param_dict["connection"] = self.combo.get()
        self.mw.param_dict["LIDAR"] = self.lidarVar.get()
        self.mw.param_dict["VIDEO"] = self.videoVar.get()
        self.mw.param_dict["SCALE"] = self.scale.get()
        self.mw.param_dict["yOFFSET"] = self.yOffset.get()
        self.mw.param_dict["scanAREA"] = self.scanAREA.get()
        
        self.mw.LIDAR = self.lidarVar.get()
        self.mw.VIDEO = self.videoVar.get()

        self.mw.SCREEN_SCALE = self.scale.get()
        self.mw.yShift = self.yOffset.get()

        self.mw._scanRange = self.scanAREA.get()
        
        self.mw.configure(event=None)

        
        
        
    def on_ok(self, event=None):
        self.mw.param_dict["xy_speed"] = self.xy_speed.get()
        self.mw.param_dict["z_speed"] = self.z_speed.get()
        self.mw.param_dict["xy_accel"] = self.xy_acc.get()
        self.mw.param_dict["z_accel"] = self.z_acc.get()
        self.mw.param_dict["connection"] = self.combo.get()
        self.mw.param_dict["LIDAR"] = self.lidarVar.get()
        self.mw.param_dict["VIDEO"] = self.videoVar.get()
        self.mw.param_dict["SCALE"] = self.scale.get()
        self.mw.param_dict["yOFFSET"] = self.yOffset.get()
        self.mw.param_dict["scanAREA"] = self.scanAREA.get()
        
        self.mw.LIDAR = self.lidarVar.get()
        self.mw.VIDEO = self.videoVar.get()

        self.mw._scanRange = self.scanAREA.get()
        
        ### ROBOT PÂRAMETERS ###
        self.mw.connection_type = self.mw.param_dict["connection"]
        
        if self.mw.ROBOT:
            self.mw.myRobot.setSpeedAcc(self.mw.param_dict["xy_speed"],self.mw.param_dict["z_speed"],self.mw.param_dict["xy_accel"],self.mw.param_dict["z_accel"])
            #self.myRobot.setSpeedAcc(100,100,100,100)
        ### save to file ###   
        tmp = json.dumps(self.mw.param_dict)
        f = open("config.json","w")
        f.write(tmp)
        f.close()
        self.mw.CustomDialog_show = False
        
        self.destroy()

    def on_closing(self, event=None):
        self.mw.CustomDialog_show = False
        self.destroy()

    def show(self):
        self.wm_deiconify()
        self.ok_button.focus_force()
        
        return self.param_dict

    
##############################################
### Class to implement VISION Setup dialog
##############################################
##############################################
class VisionDialog(Tkinter.Toplevel):

    param_dict = {}

    mw = None

    low_color = None
    up_color = None

    profiles = {}
    pIndex = 0
    
    
    def __init__(self, parent, values):
        
        Tkinter.Toplevel.__init__(self)#, parent)
        if platform.system() == 'Windows':
            self.geometry("542x578")
            self.iconbitmap('./icons/scara_app.ico')
        else:
            self.geometry("621x578")
        self.title("VISION SYSTEM CONFIGURATION")
        self.resizable(width=FALSE, height=FALSE)
        self.configure(background='black')

        self.mw = parent
        
        self.area_frame = Frame(self, highlightbackground="black", highlightcolor="black", highlightthickness=2, bg="gray", width=613, height=150, padx=3, pady=3)
        self.area_frame.grid_rowconfigure(0, weight=1)
        self.area_frame.grid_columnconfigure(0, weight=1)
        self.area_frame.grid(row=0, column=0, sticky="nsew")
        
        self.color_frame = Frame(self, highlightbackground="black", highlightcolor="black", highlightthickness=2, bg="gray", width=613, height=150, padx=3, pady=3)
        self.color_frame.grid_rowconfigure(0, weight=1)
        self.color_frame.grid_columnconfigure(0, weight=1)
        self.color_frame.grid(row=1, column=0, sticky="nsew")

        self.image_frame = Frame(self, highlightbackground="black", highlightcolor="black", highlightthickness=2, bg="gray", width=613, height=150, padx=3, pady=3)
        self.image_frame.grid_rowconfigure(0, weight=1)
        self.image_frame.grid_columnconfigure(0, weight=1)
        self.image_frame.grid(row=2, column=0, sticky="nsew")

        self.pipeline_frame = Frame(self, highlightbackground="black", highlightcolor="black", highlightthickness=2, bg="gray", width=613, height=150, padx=3, pady=3)
        self.pipeline_frame.grid_rowconfigure(0, weight=1)
        self.pipeline_frame.grid_columnconfigure(0, weight=1)
        self.pipeline_frame.grid(row=3, column=0, sticky="nsew")

        self.buttons_frame = Frame(self, bg="black", width=613, height=100, padx=3, pady=3)
        self.buttons_frame.grid_rowconfigure(0, weight=1)
        self.buttons_frame.grid_columnconfigure(0, weight=1)
        self.buttons_frame.grid(row=4, column=0, sticky="nsew")

        ### object AREA ###
        self.min_area = Tkinter.Scale(self.area_frame, highlightbackground="black", highlightcolor="black", highlightthickness=2,from_=100, to=2000, bg="gray40", label="min area", command=self.update)
        self.min_area.pack(side=LEFT, padx = 2, pady = 10)
        self.min_area.set(self.mw.area_min)

        self.max_area = Tkinter.Scale(self.area_frame, highlightbackground="black", highlightcolor="black", highlightthickness=2,from_=100, to=10000, bg="gray40", label = "max area", command=self.update)
        self.max_area.pack(side=LEFT, padx = 2, pady = 10)
        self.max_area.set(self.mw.area_max)

        self.Xmm = Tkinter.Scale(self.area_frame, state=DISABLED, highlightbackground="black", highlightcolor="black", highlightthickness=2,from_=100, to=1000, bg="gray40", label="Xmm", command=self.update)
        self.Xmm.pack(side=LEFT, padx = 2, pady = 10)
        self.Xmm.set(self.mw.Xmm)

        self.Ymm = Tkinter.Scale(self.area_frame, state=DISABLED, highlightbackground="black", highlightcolor="black", highlightthickness=2,from_=100, to=1000, bg="gray40", label = "Ymm", command=self.update)
        self.Ymm.pack(side=LEFT, padx = 2, pady = 10)
        self.Ymm.set(self.mw.Ymm)


        self.canvas_max_area = Canvas(self.area_frame, width=100, height=102, bg="gray90" ,highlightthickness=2, highlightbackground="black")
        self.canvas_max_area.pack(side=RIGHT, padx = 2, pady = 10)

        (r,g,b) = colorsys.hsv_to_rgb(self.mw.HSVColor['Hu']/180., self.mw.HSVColor['Su']/255., self.mw.HSVColor['Vu']/255.)
        _rgb = "#%02x%02x%02x" % (r*255,g*255,b*255)

        _l = int(math.sqrt(self.mw.area_max))/2
        max_a = self.canvas_max_area.create_rectangle(52 - _l, 51 - _l, 52 + _l, 51 + _l, fill=_rgb)

        (r,g,b) = colorsys.hsv_to_rgb(self.mw.HSVColor['Hl']/180., self.mw.HSVColor['Sl']/255., self.mw.HSVColor['Vl']/255.)
        _rgb = "#%02x%02x%02x" % (r*255,g*255,b*255)

        _l = int(math.sqrt(self.mw.area_min))/2
        min_a = self.canvas_max_area.create_rectangle(52 - _l, 51 - _l, 52 + _l, 51 + _l, fill=_rgb)

        
        
        ### object COLOR ###

        self.lowH = Tkinter.Scale(self.color_frame, highlightbackground="black", highlightcolor="black", highlightthickness=2,from_=0, to=180, bg="gray30", label="H", command=self.update)
        self.lowH.pack(side=LEFT, padx = 2, pady = 10)
        self.lowH.set(self.mw.HSVColor['H'])

        self.lowS = Tkinter.Scale(self.color_frame, highlightbackground="black", highlightcolor="black", highlightthickness=2,from_=0, to=255, bg="gray30", label="S", command=self.update)
        self.lowS.pack(side=LEFT, padx = 2, pady = 10)
        self.lowS.set(self.mw.HSVColor['S'])

        self.lowV = Tkinter.Scale(self.color_frame, highlightbackground="black", highlightcolor="black", highlightthickness=2,from_=0, to=255, bg="gray30", label="V", command=self.update)
        self.lowV.pack(side=LEFT, padx = 2, pady = 10)
        self.lowV.set(self.mw.HSVColor['V'])

        self.wideV = Tkinter.Scale(self.color_frame, highlightbackground="black", highlightcolor="black", highlightthickness=2,from_=0, to=255, bg="gray30", label="V wide", command=self.update)
        self.wideV.pack(side=RIGHT, padx = 2, pady = 10)
        self.wideV.set(self.mw.color_wide_V)

        self.wideS = Tkinter.Scale(self.color_frame, highlightbackground="black", highlightcolor="black", highlightthickness=2,from_=0, to=255, bg="gray30", label="S wide", command=self.update)
        self.wideS.pack(side=RIGHT, padx = 2, pady = 10)
        self.wideS.set(self.mw.color_wide_S)

        self.wideH = Tkinter.Scale(self.color_frame, highlightbackground="black", highlightcolor="black", highlightthickness=2,from_=0, to=180, bg="gray30", label="H wide", command=self.update)
        self.wideH.pack(side=RIGHT, padx = 2, pady = 10)
        self.wideH.set(self.mw.color_wide_H)


        
        
        self.cspace_img = Image.open('./images/hsv_color_space.png')
        self.cspace_img = self.cspace_img.resize((154,104), Image.ANTIALIAS)
        self.im_cspace = ImageTk.PhotoImage(self.cspace_img)

        self.canvas_scolor = Canvas(self.image_frame, width=150, height=102, bg="white" ,highlightthickness=2, highlightbackground="black")
        self.canvas_scolor.pack(side=LEFT, padx = 2, pady = 10)
        self.canvas_csImage = self.canvas_scolor.create_image(0, 0, image=self.im_cspace, anchor="nw")
            

        (r,g,b) = colorsys.hsv_to_rgb(self.mw.HSVColor['Hu']/180., self.mw.HSVColor['Su']/255., self.mw.HSVColor['Vu']/255.)
        _rgb = "#%02x%02x%02x" % (r*255,g*255,b*255)
        self.canvas_ucolor = Canvas(self.image_frame, width=100, height=102, bg=_rgb ,highlightthickness=2, highlightbackground="black")
        self.canvas_ucolor.pack(side=RIGHT, padx = 2, pady = 10)

        (r,g,b) = colorsys.hsv_to_rgb(self.mw.HSVColor['H']/180., self.mw.HSVColor['S']/255., self.mw.HSVColor['V']/255.)
        _rgb = "#%02x%02x%02x" % (r*255,g*255,b*255)
        self.canvas_color = Canvas(self.image_frame, width=100, height=102, bg=_rgb ,highlightthickness=2, highlightbackground="black")
        self.canvas_color.pack(side=RIGHT, padx = 2, pady = 10)

        (r,g,b) = colorsys.hsv_to_rgb(self.mw.HSVColor['Hl']/180., self.mw.HSVColor['Sl']/255., self.mw.HSVColor['Vl']/255.)
        _rgb = "#%02x%02x%02x" % (r*255,g*255,b*255)
        self.canvas_lcolor = Canvas(self.image_frame, width=100, height=102, bg=_rgb ,highlightthickness=2, highlightbackground="black")
        self.canvas_lcolor.pack(side=RIGHT, padx = 2, pady = 10)

        ### VISION PIPELINE
        label_tab = Tkinter.Label(self.pipeline_frame, text="Pipeline stage: ", bg="gray")
        label_tab.pack(side=LEFT)
        
        self.combo = ttk.Combobox(self.pipeline_frame, width=15, state="readonly", values=("image raw BGR","equalized","color mask","image gray","image blur","image binary","object detection"))
        self.combo.set("object detection")
        self.combo.pack(side=LEFT)

        self.combo.bind("<<ComboboxSelected>>", self.update)

        self.bThreshold = Tkinter.Scale(self.pipeline_frame, highlightbackground="black", highlightcolor="black", highlightthickness=2,from_=0, to=255, bg="gray30", label="Binary Threshold", command=self.update)
        self.bThreshold.pack(side=RIGHT, padx = 2, pady = 10)
        self.bThreshold.set(self.mw.b_threshold)
        
        ### BUTTONS ###
        
        self.combo_profiles = ttk.Combobox(self.buttons_frame, width=15, state="readonly")#, values=("Yellow"))
        self.combo_profiles.set("Yellow")
        self.combo_profiles.pack(side=LEFT)

        self.combo_profiles.bind("<<ComboboxSelected>>", self.set_profile)
        
        self.save_button = Tkinter.Button(self.buttons_frame, state='normal', highlightbackground="black", highlightcolor="black", highlightthickness=1 ,bg="gray", text=" Save ", command=self.on_save)
        self.save_button.pack(side=LEFT,padx=4)

        self.delete_button = Tkinter.Button(self.buttons_frame, state=DISABLED, highlightbackground="black", highlightcolor="black", highlightthickness=1 ,bg="gray", text=" Delete ", command=self.on_delete)
        self.delete_button.pack(side=LEFT,padx=4)

        self.profile_name = Tkinter.Text(self.buttons_frame, height=1, width=10)
        self.profile_name.pack(side=LEFT,padx=4)

        self.ok_button = Tkinter.Button(self.buttons_frame, highlightbackground="black", highlightcolor="black", highlightthickness=1 ,bg="gray", text="   OK   ", command=self.on_ok)
        self.ok_button.pack(side=RIGHT,padx=4)

        self.default_button = Tkinter.Button(self.buttons_frame, highlightbackground="black", highlightcolor="black", highlightthickness=1 ,bg="gray", text=" Default ", command=self.on_default)
        self.default_button.pack(side=RIGHT,padx=4)
        
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.mw.buttonCFGVISION.config(state=DISABLED)
        if os.path.isfile('profiles.json'):
            # Load profiles from file...
            with open("profiles.json") as handle:
                tmp = json.loads(handle.read())
                for i in range(len(tmp)):
                    self.profiles[i] = tmp[str(i)]
                    _values = list(self.combo_profiles["values"])
                    self.combo_profiles["values"] = _values + [tmp[str(i)]["name"]]
                    self.pIndex +=1

        else:
            self.profiles[0] = {'amin':500,
                                              'amax':3100,
                                              'H':30,
                                              'S':147,
                                              'V':159,
                                              'Hw':9,
                                              'Sw':108,
                                              'Vw':96,
                                              'bThres':127,
                                                'name':"yellow",
                                                'Xmm':404,
                                                'Ymm':325}
            self.pIndex += 1
            _values = list(self.combo_profiles["values"])
            self.combo_profiles["values"] = _values + ["Yellow"]
            
        

        
    def on_closing(self, event=None):
        self.mw.VisionDialog_show = False
        if self.mw.robotVISION.get():
            self.mw.buttonCFGVISION.config(state='normal')
        self.destroy()
        
    def set_profile(self, event=None):
        
        _profile_name = self.combo_profiles.current()

        
        self.mw.area_min = self.profiles[_profile_name]["amin"]
        self.mw.area_max = self.profiles[_profile_name]["amax"]
        self.mw.l_area_min = int(math.sqrt(self.mw.area_min))/2
        self.mw.l_area_max = int(math.sqrt(self.mw.area_max))/2
        self.min_area.set(self.mw.area_min)
        self.max_area.set(self.mw.area_max)

        self.mw.color_wide_H = self.profiles[_profile_name]["Hw"]
        self.mw.color_wide_S = self.profiles[_profile_name]["Sw"]
        self.mw.color_wide_V = self.profiles[_profile_name]["Vw"]

        self.wideH.set(self.mw.color_wide_H)
        self.wideS.set(self.mw.color_wide_S)
        self.wideV.set(self.mw.color_wide_V)
        # default Yellow color
 
        self.mw.HSVColor['H'] = self.profiles[_profile_name]["H"]
        self.mw.HSVColor['S'] = self.profiles[_profile_name]["S"]
        self.mw.HSVColor['V'] = self.profiles[_profile_name]["V"]
        
        self.mw.HSVColor['Hl'] = np.clip(self.mw.HSVColor['H'] - self.mw.color_wide_H,0,180) # 21
        self.mw.HSVColor['Sl'] = np.clip(self.mw.HSVColor['S'] - self.mw.color_wide_S,0,255) # 39
        self.mw.HSVColor['Vl'] = np.clip(self.mw.HSVColor['V'] - self.mw.color_wide_V,0,255)# 64
        self.mw.HSVColor['Hu'] = np.clip(self.mw.HSVColor['H'] + self.mw.color_wide_H,0,180) # 40
        self.mw.HSVColor['Su'] = np.clip(self.mw.HSVColor['S'] + self.mw.color_wide_S,0,255) # 255
        self.mw.HSVColor['Vu'] = np.clip(self.mw.HSVColor['V'] + self.mw.color_wide_V,0,255) # 255

        self.lowH.set(self.mw.HSVColor['H'])
        self.lowS.set(self.mw.HSVColor['S'])
        self.lowV.set(self.mw.HSVColor['V'])

        self.mw.Xmm = self.profiles[_profile_name]["Xmm"]
        self.Xmm.set(self.mw.Xmm)
        self.mw.Ymm = self.profiles[_profile_name]["Ymm"]
        self.Ymm.set(self.mw.Ymm)
        
        (r,g,b) = colorsys.hsv_to_rgb(self.mw.HSVColor['H']/180., self.mw.HSVColor['S']/255., self.mw.HSVColor['V']/255.)
        _rgb = "#%02x%02x%02x" % (r*255,g*255,b*255)
        self.canvas_color.configure(bg=_rgb)

        (r,g,b) = colorsys.hsv_to_rgb(self.mw.HSVColor['Hl']/180., self.mw.HSVColor['Sl']/255., self.mw.HSVColor['Vl']/255.)
        _rgb = "#%02x%02x%02x" % (r*255,g*255,b*255)
        self.canvas_lcolor.configure(bg=_rgb)

        (r,g,b) = colorsys.hsv_to_rgb(self.mw.HSVColor['Hu']/180., self.mw.HSVColor['Su']/255., self.mw.HSVColor['Vu']/255.)
        _rgb = "#%02x%02x%02x" % (r*255,g*255,b*255)
        self.canvas_ucolor.configure(bg=_rgb)

        self.mw.b_threshold = self.profiles[_profile_name]["bThres"]
        self.bThreshold.set(self.mw.b_threshold)

        self.delete_button['state'] = 'normal'
        self.save_button['state'] = 'normal'
        

    def on_delete(self, event=None):
        if self.combo_profiles.get() != "Yellow":
            _tmpProfiles = {}
            self.profiles.pop(self.combo_profiles.current())

            _idelete = self.combo_profiles.current()
            _values = list(self.combo_profiles["values"])
            del _values[_idelete]
            self.combo_profiles["values"] = _values

            self.pIndex = 0
            for i in self.profiles:
                _tmpProfiles[self.pIndex] = self.profiles[i]
                self.pIndex += 1
                
            self.profiles = _tmpProfiles
            

            tmp = json.dumps(self.profiles)
            f = open("profiles.json","w")
            f.write(tmp)
            f.close()

            self.combo_profiles.set("Yellow")
        
        
        
    def on_save(self, event=None):
        
        _profile_name = ""
        _profile_name = str(self.profile_name.get("1.0",'end-1c'))
        
        if _profile_name != "":
            self.profiles[self.pIndex] = {'amin':self.min_area.get(),
                                   'amax':self.max_area.get(),
                                   'H':self.lowH.get(),
                                   'S':self.lowS.get(),
                                   'V':self.lowV.get(),
                                   'Hw':self.wideH.get(),
                                   'Sw':self.wideS.get(),
                                   'Vw':self.wideV.get(),
                                   'bThres':self.bThreshold.get(),
                                    'name':_profile_name,
                                    'Xmm':self.Xmm.get(),
                                    'Ymm':self.Ymm.get()}
            
            
            self.pIndex += 1
            _values = list(self.combo_profiles["values"])
            self.combo_profiles["values"] = _values + [_profile_name]
            self.combo_profiles.set(_profile_name)
            # Save to file
            tmp = json.dumps(self.profiles)
            f = open("profiles.json","w")
            f.write(tmp)
            f.close()    
            
        else:
            self.profiles[self.combo_profiles.current()] = {'amin':self.min_area.get(),
                                   'amax':self.max_area.get(),
                                   'H':self.lowH.get(),
                                   'S':self.lowS.get(),
                                   'V':self.lowV.get(),
                                   'Hw':self.wideH.get(),
                                   'Sw':self.wideS.get(),
                                   'Vw':self.wideV.get(),
                                   'bThres':self.bThreshold.get(),
                                    'name':self.combo_profiles.get(),
                                    'Xmm':self.Xmm.get(),
                                    'Ymm':self.Ymm.get()}
            
            
            # Save to file
            tmp = json.dumps(self.profiles)
            f = open("profiles.json","w")
            f.write(tmp)
            f.close()

        self.profile_name.delete("1.0",END)
           
    def update(self, event=None):
        self.mw.area_min = self.min_area.get()
        self.mw.area_max = self.max_area.get()
        self.mw.l_area_min = int(math.sqrt(self.mw.area_min))/2
        self.mw.l_area_max = int(math.sqrt(self.mw.area_max))/2

        self.mw.color_wide_H = self.wideH.get()
        self.mw.color_wide_S = self.wideS.get()
        self.mw.color_wide_V = self.wideV.get()

        self.mw.HSVColor['H'] = self.lowH.get()
        self.mw.HSVColor['S'] = self.lowS.get()
        self.mw.HSVColor['V'] = self.lowV.get()

        self.mw.HSVColor['Hl'] = np.clip(self.mw.HSVColor['H'] - self.mw.color_wide_H,0,180) # 21
        self.mw.HSVColor['Sl'] = np.clip(self.mw.HSVColor['S'] - self.mw.color_wide_S,0,255) # 39
        self.mw.HSVColor['Vl'] = np.clip(self.mw.HSVColor['V'] - self.mw.color_wide_V,0,255)# 64
        self.mw.HSVColor['Hu'] = np.clip(self.mw.HSVColor['H'] + self.mw.color_wide_H,0,180) # 40
        self.mw.HSVColor['Su'] = np.clip(self.mw.HSVColor['S'] + self.mw.color_wide_S,0,255) # 255
        self.mw.HSVColor['Vu'] = np.clip(self.mw.HSVColor['V'] + self.mw.color_wide_V,0,255) # 255
        
        (r,g,b) = colorsys.hsv_to_rgb(self.mw.HSVColor['H']/180., self.mw.HSVColor['S']/255., self.mw.HSVColor['V']/255.)
        _rgb = "#%02x%02x%02x" % (r*255,g*255,b*255)
        self.canvas_color.configure(bg=_rgb)

        (r,g,b) = colorsys.hsv_to_rgb(self.mw.HSVColor['Hl']/180., self.mw.HSVColor['Sl']/255., self.mw.HSVColor['Vl']/255.)
        _rgb = "#%02x%02x%02x" % (r*255,g*255,b*255)
        self.canvas_lcolor.configure(bg=_rgb)

        (r,g,b) = colorsys.hsv_to_rgb(self.mw.HSVColor['Hu']/180., self.mw.HSVColor['Su']/255., self.mw.HSVColor['Vu']/255.)
        _rgb = "#%02x%02x%02x" % (r*255,g*255,b*255)
        self.canvas_ucolor.configure(bg=_rgb)

        self.canvas_max_area.delete("all")
        
        (r,g,b) = colorsys.hsv_to_rgb(self.mw.HSVColor['Hu']/180., self.mw.HSVColor['Su']/255., self.mw.HSVColor['Vu']/255.)
        _rgb = "#%02x%02x%02x" % (r*255,g*255,b*255)

        _l = int(math.sqrt(self.mw.area_max))/2
        max_a = self.canvas_max_area.create_rectangle(52 - _l, 51 - _l, 52 + _l, 51 + _l, fill=_rgb)

        (r,g,b) = colorsys.hsv_to_rgb(self.mw.HSVColor['Hl']/180., self.mw.HSVColor['Sl']/255., self.mw.HSVColor['Vl']/255.)
        _rgb = "#%02x%02x%02x" % (r*255,g*255,b*255)

        _l = int(math.sqrt(self.mw.area_min))/2
        min_a = self.canvas_max_area.create_rectangle(52 - _l, 51 - _l, 52 + _l, 51 + _l, fill=_rgb)
        
        self.mw.Xmm = self.Xmm.get()
        self.mw.Ymm = self.Ymm.get()

        self.mw.b_threshold = self.bThreshold.get()
        
        
        self.mw.show_image = self.combo.get()

        
        
    def on_default(self, event=None):
        self.mw.area_min = 500
        self.mw.area_max = 3100
        self.mw.l_area_min = int(math.sqrt(self.mw.area_min))/2
        self.mw.l_area_max = int(math.sqrt(self.mw.area_max))/2
        self.min_area.set(self.mw.area_min)
        self.max_area.set(self.mw.area_max)

        self.mw.color_wide_H = 9
        self.mw.color_wide_S = 108
        self.mw.color_wide_V = 96

        self.wideH.set(self.mw.color_wide_H)
        self.wideS.set(self.mw.color_wide_S)
        self.wideV.set(self.mw.color_wide_V)
        # default Yellow color

        self.mw.HSVColor['H'] = 30
        self.mw.HSVColor['S'] = 147
        self.mw.HSVColor['V'] = 159
        
        self.mw.HSVColor['Hl'] = np.clip(self.mw.HSVColor['H'] - self.mw.color_wide_H,0,180) # 21
        self.mw.HSVColor['Sl'] = np.clip(self.mw.HSVColor['S'] - self.mw.color_wide_S,0,255) # 39
        self.mw.HSVColor['Vl'] = np.clip(self.mw.HSVColor['V'] - self.mw.color_wide_V,0,255)# 64
        self.mw.HSVColor['Hu'] = np.clip(self.mw.HSVColor['H'] + self.mw.color_wide_H,0,180) # 40
        self.mw.HSVColor['Su'] = np.clip(self.mw.HSVColor['S'] + self.mw.color_wide_S,0,255) # 255
        self.mw.HSVColor['Vu'] = np.clip(self.mw.HSVColor['V'] + self.mw.color_wide_V,0,255) # 255

        self.lowH.set(self.mw.HSVColor['H'])
        self.lowS.set(self.mw.HSVColor['S'])
        self.lowV.set(self.mw.HSVColor['V'])
        
        (r,g,b) = colorsys.hsv_to_rgb(self.mw.HSVColor['H']/180., self.mw.HSVColor['S']/255., self.mw.HSVColor['V']/255.)
        _rgb = "#%02x%02x%02x" % (r*255,g*255,b*255)
        self.canvas_color.configure(bg=_rgb)

        (r,g,b) = colorsys.hsv_to_rgb(self.mw.HSVColor['Hl']/180., self.mw.HSVColor['Sl']/255., self.mw.HSVColor['Vl']/255.)
        _rgb = "#%02x%02x%02x" % (r*255,g*255,b*255)
        self.canvas_lcolor.configure(bg=_rgb)

        (r,g,b) = colorsys.hsv_to_rgb(self.mw.HSVColor['Hu']/180., self.mw.HSVColor['Su']/255., self.mw.HSVColor['Vu']/255.)
        _rgb = "#%02x%02x%02x" % (r*255,g*255,b*255)
        self.canvas_ucolor.configure(bg=_rgb)

        self.mw.Xmm = 404
        self.Xmm.set(self.mw.Xmm)

        self.mw.Ymm = 325
        self.Ymm.set(self.mw.Ymm)

        self.mw.b_threshold = 127
        self.bThreshold.set(self.mw.b_threshold)

        self.combo.set("object detection")
        self.mw.show_image = self.combo.get()
        

        
        
    def on_ok(self, event=None):
        self.mw.VisionDialog_show = False
        if self.mw.robotVISION.get():
            self.mw.buttonCFGVISION.config(state='normal')
        self.destroy()

    def show(self):
        self.wm_deiconify()
        self.ok_button.focus_force()
        return self.param_dict




####################################
### MAIN Class
####################################
####################################
class MainWindow():

    VERSION = "32"

    counter = 0

    top_frame = None
    center = None
    center2 = None
    btm_frame = None
    btm_frame2 = None

    reader = None

    model_label = None
    data1_label = None
    data2_label = None

    label0 = None
    label1 = None
    label2 = None

    ctr_left = None
    ctr_right = None
    
    path = ""
    im = None
    im_zoom = None
    tmp_img = None
    grid_img = None
    im_raw = None
    image_GBP = None
    imageGBP_raw = None
    image_GC = None
    
    canvasImage = None

    data = None

    c_level = 0

    ZOOM_F = 100

    ERROR_GENERAL = ""

    IMG_MASK = 1

    canvas_info = "ESCALA DE GRISES"
    canvas_RetAI = ""

    numPixels = 512*512
    width = 512
    height = 512

    RetAI_status = False
    map_swap = 0

    
    myRobot = None
    Z = 0.0
    X = 0.0
    Y = 20.0
    tZ = 0.0

    len1 = 9.161 #9.0
    len2 = 10.592 #11.0

    A1 = 90.0
    A2 = 0.0
    botX = 0.0
    botY = 20.0
    botZ = 0

    control_mode =  1 #0 Real time, 1 Trajectories, 2 leap motion

    LIMIT = 20#len1 + len2
    LIMITmin = 8.5#10.0
    Zlimit = 120

    tSpeed = 40

    dictTrajectories = {}
    objects = {}
    tIndex = 0

    onRoute = False

    leapVar = None
    realVar = None
    traVar = None
    robotVar = None

    buttonLEAP = None
    buttonRT = None
    buttonLines = None
    buttonOBJECT = None
    buttonCFGVISION = None

    ROBOT = False
    WINDOWS = True

    robotSTATUS = 'OFFLINE'

    robotSTOP = False

    pinANGLE = 0
    robotPIN = 0
    pinOFFSET = 0
    zOFFSET = 0

    resizeFACTOR = 1.0
    resizeFACTOR2 = 1.0

    pinOPEN = False

    B2_dist = 0

    vc = None
    frame = None
    CAMERA = False
    
    ##### IMAGES #####
    base_im = None
    arm_im = None
    pinzaL = None
    pinzaL_tk = None
    pinzaR = None
    pinzaR_Op = None
    baseL = None
    baseL_tk = None
    armL = None
    armL_tk = None
    forearmL = None
    forearmL_tk = None

    bboard = None

    
    

    join1_im = None
    join2_im = None

    processing = False

    templateMODE = 0

    PS3eye = False

    animation_rw = {}

    width_shift = 0

    outLIMIT = False

    scale = 48

    RIGHT = True

    KINEMATIC = False

    mX = 0
    mY = 20

    mx = 0
    my = 0

    

    IDLE = True

    mouseIN = False

    blocklyDATA = {}

    blocklyCMD = False
    robotDATA = []

    drawingOBJ = False
    captureOBJ = False

    connection_type = "WIFI"

    HSVColor = {}
    color_wide_H = 10
    color_wide_S = 90
    color_wide_V = 90
    area_min = 500
    l_area_min = int(math.sqrt(300))/2
    area_max = 3100
    l_area_max = int(math.sqrt(600))/2
    Xmm = 404
    Ymm = 325
    b_threshold = 127
    param_dict = {}
    param_vision = {}
    callibration = {}

    binary_image = False
    mask_image = False
    show_image = "object detection"

    EDIT = False

    lidar_data = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    lidar_1d = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    lidar_2d = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    lidar_slope_range = 4
    
    lMax = 20
    LIDAR = True
    VIDEO = True
    LIDAR_ON = False
    lTemp = 0
    lstep = 8

    SCREEN_SCALE = 32
    yShift = 6

    LIDAR_ZERO = 55

    SCAN = False
    scanGETDATA = False

    _scanRange = 25
    _dy = 17.5
    surface_data = np.zeros((2*_scanRange,2*_scanRange))
    scanAstatusY = _dy

    pDY = []
    pDX = []
    pDZ = []
    
    scanIMAGE = None

    camX = 640
    camY = 480

    image_pipe = {}

    points = {0: [203, 380], 1: [445, 379], 2: [325, 318], 3: [326, 442], 4: [326, 380]}

    mV = (points[3][1] - points[2][1]) / float((points[3][0] - points[2][0]))
    mH = (points[1][1] - points[0][1]) / float((points[1][0] - points[0][0]))

    pixelX_size = 0.8
    pixelY_size = 0.8
    
    cam_angle = 0

    CustomDialog_show = False
    VisionDialog_show = False
    CalibrationDialog_show = False

    scanSTOP = False

    recollecting = False

    z_points = {0: 0, 1: 0, 2:0}
    Z_offset = 0.0

    gamepad_ready = False
    leap_ready = False

    pad_count = 0

    XBOX_Pad = False
    
    #########################################       
    ### LIDAR Sensor DATA processing function
    #########################################       
    def get_lidar(self):

        self.LIDAR_ZERO = 50
        
        if self.LIDAR_ON:
            #get LIDAR from API
            _tmpd = int(self.robotDATA[6])

            _tmpd = (abs((self.Z+self.LIDAR_ZERO)-_tmpd))
            data = np.clip(_tmpd,0,120)

            for i in range(0,self.lMax-1):
                # calculate 1d
                self.lidar_1d[i] = self.lidar_data[i+1] - self.lidar_data[i]

                # calculate 2d

                # shift LIDAR data
                self.lidar_data[i] = self.lidar_data[i+1]

            self.lidar_data[self.lMax-1] = data

            if self.scanGETDATA:
                _dt = self.surface_data[np.clip(int((2*self._scanRange-1)-((self._dy*10)-(self.botY*10))),0,(2*self._scanRange-1)),np.clip(int(self.botX*10)+self._scanRange,0,(2*self._scanRange-1))]
                if _dt != 0:
                    data = (data + _dt)/2.0
                self.surface_data[np.clip(int((2*self._scanRange-1)-((self._dy*10)-(self.botY*10))),0,(2*self._scanRange-1)),np.clip(int(self.botX*10)+self._scanRange,0,(2*self._scanRange-1))] = data
                self.pDX.append(np.clip(int(self.botX*10)+self._scanRange,0,(2*self._scanRange-1)))
                self.pDY.append(np.clip(int((2*self._scanRange-1)-((self._dy*10)-(self.botY*10))),0,(2*self._scanRange-1)))
                self.pDZ.append(data)
            
        else:
            # generate synthetic signal
            if abs(self.lTemp) <= 360:
                data = int(50*math.sin(math.radians(self.lTemp))) + 50
                
                self.lTemp += self.lstep
            else:
                self.lTemp = 1
                data = int(50*math.sin(math.radians(self.lTemp))) + 50
                
                self.lTemp += self.lstep

            
            for i in range(0,self.lMax-1):
                # calculate 1d
                self.lidar_1d[i] = self.lidar_data[i+1] - self.lidar_data[i]

                # calculate 2d

                # shift LIDAR data
                self.lidar_data[i] = self.lidar_data[i+1]

            self.lidar_data[self.lMax-1] = data

            if self.scanGETDATA:
                self.surface_data[np.clip(int((2*self._scanRange-1)-((self._dy*10)-(self.Y*10))),0,(2*self._scanRange-1)),np.clip(int(self.X*10)+self._scanRange,0,(2*self._scanRange-1))] = data
                self.pDX.append(np.clip(int(self.X*10)+self._scanRange,0,(2*self._scanRange-1)))
                self.pDY.append(np.clip(int((2*self._scanRange-1)-((self._dy*10)-(self.Y*10))),0,(2*self._scanRange-1)))
                self.pDZ.append(data)   
                
                

    #########################################       
    ### FUNCTION: Draw LIDAR information
    #########################################       
    def draw_lidar(self):

        
        PofX = self.resizeFACTOR
        PofY = 100*self.resizeFACTOR
        

        _w = 5

        if self.LIDAR_ON:
            self.canvas.create_text(25*self.resizeFACTOR + PofX,(self.height-(260*self.resizeFACTOR)) + PofY,fill="white",text="LIDAR ONLINE",anchor="nw")
        else:
            self.canvas.create_text(25*self.resizeFACTOR + PofX,(self.height-(260*self.resizeFACTOR)) + PofY,fill="white",text="LIDAR OFFLINE",anchor="nw")
        
        self.canvas.create_line(25*self.resizeFACTOR + PofX, (self.height-(120*self.resizeFACTOR)) + PofY, 123*self.resizeFACTOR + PofX,(self.height-(120*self.resizeFACTOR)) + PofY,fill="white", width=2)#, dash=(4,2))
        self.canvas.create_line(25*self.resizeFACTOR + PofX,(self.height-(120*self.resizeFACTOR)) + PofY,25*self.resizeFACTOR + PofX,(self.height-(245*self.resizeFACTOR)) + PofY,fill="white", width=2)#, dash=(4,2))

        for i in range(1,13):
            self.canvas.create_text(25*self.resizeFACTOR + PofX,(self.height-((125+i*10)*self.resizeFACTOR)) + PofY,fill="white",text="{}-".format(10*i),anchor="ne", font=("Purisa", 10))

        for i in range(0,self.lMax-1):
             self.canvas.create_line((30+i*_w)*self.resizeFACTOR + PofX,(self.height-(122*self.resizeFACTOR)) + PofY,(30+i*_w)*self.resizeFACTOR + PofX,(self.height-(120+self.lidar_data[i])*self.resizeFACTOR) + PofY,fill="gray"+str((59+(i*2))), width=_w, dash=(4,2))

    #########################################               
    ### FUNCTION: Blockly interface with MAIN Class
    #########################################       
    def setXYZ45(self,ch1,ch2,ch3,ch4,ch5):
        
        if ch1 <> -20000:
            self.blocklyDATA['X'] = ch1/float(10)
        else:
            self.blocklyDATA['X'] = self.X
        if ch2 <> -20000:
            self.blocklyDATA['Y'] = ch2/float(10)
        else:
            self.blocklyDATA['Y'] = self.Y
        if ch3 <> -20000:
            self.blocklyDATA['Z'] = ch3
        else:
            self.blocklyDATA['Z'] = self.Z
        if ch4 <> -20000:
            self.blocklyDATA['ANGLE'] = ch4
        else:
            self.blocklyDATA['ANGLE'] = self.pinANGLE
        if ch5 <> -20000:
            if ch5 == 500:
                self.blocklyDATA['OPEN'] = False
            else:
                self.blocklyDATA['OPEN'] = True
        else:
            self.blocklyDATA['OPEN'] = self.pinOPEN    

        
            
        self.blocklyCMD = True

        while self.blocklyCMD:
            time.sleep(0.1)

        
        
    def load_image(self):
           
        self.path = './images/SCARA.jpg'
        
        if os.path.isfile(self.path):
            self.tmp_img = Image.open(self.path)
            self.tmp_img = self.tmp_img.resize((75,75), Image.ANTIALIAS)
            self.im_zoom = ImageTk.PhotoImage(self.tmp_img)
            self.img_error = False
        else:
            self.tmp_img = Image.open("images/error.png")
            self.tmp_img = self.tmp_img.resize((self.width,self.height), Image.ANTIALIAS)
            self.im = ImageTk.PhotoImage(self.tmp_img)
            self.im_zoom = ImageTk.PhotoImage(self.tmp_img)
            self.img_error = True
            
        
        
        self.canvas_logoImage = self.canvas_logo.create_image(0, 10, image=self.logo, anchor="nw")
        self.canvas_logoImage = self.canvas_logo.create_image(130, 0, image=self.im_zoom, anchor="nw")

        if self.gamepad_ready:
            self.gp_logo = self.canvas_logo.create_image(400, 20, image=self.gamepad_logo, anchor="nw")
        else:
            self.gp_logo = self.canvas_logo.create_image(400, 20, image=self.not_gamepad_logo, anchor="nw")

        if self.leap_ready:
            self.lp_logo = self.canvas_logo.create_image(480, 20, image=self.leap_logo, anchor="nw")
        else:
            self.lp_logo = self.canvas_logo.create_image(480, 20, image=self.not_leap_logo, anchor="nw")

        self.bl_logo = self.canvas_logo.create_image(560, 20, image=self.not_blockly_logo, anchor="nw")
        
        self.draw_bot(self.control_mode)
        
        
    def update_labels(self):
        ### update INFO ####
        self.label0['text'] = "pyBot Robotic Arm V1." + self.VERSION
        if self.robotSTOP:
            self.label2['text'] = 'Status: EMERGENCY STOP'
        else:
            self.label2['text'] = 'Status: ' + self.robotSTATUS
    
        if self.robotSTATUS != "ONLINE":
            self.label2['fg'] = 'red'
        else:
            self.label2['fg'] = 'black'

        if self.RIGHT:
            self.label1['text'] = 'Laterality Configuration: RIGHT'
        else:
            self.label1['text'] = 'Laterality Configuration: LEFT'
            
        self.top_frame2.update()
        

    def update_log(self, text):
        if self.ERROR_GENERAL == "":
            self.label_LOG['text'] = text
        else:
            self.label_LOG['text'] = self.ERROR_GENERAL

        self.canvasTI.insert("end",text + '\n')
        self.canvasTI.see("end")

    #########################################          
    ### FUNCTION: Capture keyboard events
    #########################################       
    def keypress(self, event):
        change = False

        if self.robotSTOP:
            return
        
        if event.char == "0" or event.char == "1" or event.char == "2" or event.char == "3" or event.char == "4" or event.char == "5":
            change = True
           

        if event.char == " ":
            
            self.buttonROBOT.invoke()
            
        if event.keysym == "BackSpace":
            # If Trajectorie MODE. Detect how to rebuild the trajectorie GRAPH after delete a NODE
            if self.EDIT:
            
                index,side = self.get_traject(self.mX,self.mY)
                
                    
                if self.tIndex == 2 and index == 1 and side == 1:
                    self.dictTrajectories[0]['x2'] = self.dictTrajectories[index]['x2']
                    self.dictTrajectories[0]['y2'] = self.dictTrajectories[index]['y2']
                    self.dictTrajectories[0]['z2'] = self.dictTrajectories[index]['z2']
                    self.tIndex -= 1
                    del self.dictTrajectories[self.tIndex]
                else:
                
                    if index > 0 and index < self.tIndex-1:
                        self.dictTrajectories[index-1]['x2'] = self.dictTrajectories[index+1]['x1']
                        self.dictTrajectories[index-1]['y2'] = self.dictTrajectories[index+1]['y1']
                        self.dictTrajectories[index-1]['z2'] = self.dictTrajectories[index+1]['z1']
                        for i in range(index,self.tIndex-1):
                            self.dictTrajectories[i] = self.dictTrajectories[i+1]

                        self.tIndex -= 1
                        del self.dictTrajectories[self.tIndex]
                    else:
                        

                        if index == self.tIndex-1 and side == 2 and self.tIndex > 1: #if last node
                            self.tIndex -= 1
                            del self.dictTrajectories[self.tIndex]
                        else:

                            if index == self.tIndex-1 and side == 1 and self.tIndex > 1: #if last node
                                self.dictTrajectories[index-1]['x2'] = self.dictTrajectories[index]['x2']
                                self.dictTrajectories[index-1]['y2'] = self.dictTrajectories[index]['y2']
                                self.dictTrajectories[index-1]['z2'] = self.dictTrajectories[index]['z2']
                                self.tIndex -= 1
                                del self.dictTrajectories[self.tIndex]
                            else:

                                
                                if index == 0: # if first node
                                    if self.tIndex == 1:
                                        self.clear_trajectories()

                                    else:
                                        
                                        self.dictTrajectories[1]['x1'] = self.dictTrajectories[0]['x2']
                                        self.dictTrajectories[1]['y1'] = self.dictTrajectories[0]['y2']
                                        self.dictTrajectories[1]['z1'] = self.dictTrajectories[0]['z2']
                                        for i in range(1,self.tIndex):
                                            self.dictTrajectories[i-1] = self.dictTrajectories[i]

                                        self.tIndex -= 1
                                        del self.dictTrajectories[self.tIndex]
                
                    
               
        if event.keysym == "Up":
            self.Z = self.Z + 0.1
            if self.Z < 0.0:
                self.Z = 0.0
            if self.tZ > self.Zlimit:
                self.Z = self.Zlinit
            
            self.tZ = self.Z
            
        if event.keysym == "Down":
            self.Z = self.Z - 0.1
            if self.Z < 0.0:
                self.Z = 0.0
            if self.tZ > self.Zlimit:
                self.Z = self.Zlinit
                
            self.tZ = self.Z
            

        if event.keysym == "Right":
            self.pinANGLE += 1
            
        if event.keysym == "Left":
            self.pinANGLE -= 1
            
        
        if event.keysym == "s":
            self.buttonSCAN.invoke()
            
        if event.keysym == "a":
            self.buttonABSANG.invoke()
            
        if event.keysym == "t":
            self.buttonLines.invoke()

        if event.keysym == "r":
            self.buttonRT.invoke()

        if event.keysym == "l":
            self.buttonLEAP.invoke()

        if event.keysym == "i":
            self.buttonLATERAL.invoke()
            
        if event.keysym == "k":
            self.buttonKINEMA.invoke()
            
        if event.keysym == "v":
            self.buttonVISION.invoke()

        if event.keysym == "plus":
            self.SCREEN_SCALE -= 1
            self.yShift -= 1
            self.configure(event=None)
        if event.keysym == "minus":
            self.SCREEN_SCALE += 1
            self.yShift += 1
            self.configure(event=None)
    
        if change:
         
            self.update_labels()

    #########################################       
    ### CLAMP Corner representation related FUNCTIONS
    #########################################       
    def over_pin(self,event):
        if self.WINDOWS:
            x1 = event.x
            y1 = event.y
        else:
            x1 = event.x - 3
            y1 = event.y - 105

        if (x1 < 120) and (y1 < 120):
            return True
        else:
            return False

    def over_pin2(self,event):
        if self.WINDOWS:
            x1 = event.x
            y1 = event.y
        else:
            x1 = event.x
            y1 = event.y

        if (x1 < 120) and (y1 < 120):
            return True
        else:
            return False
    
    def over_pin3(self,x1,y1):

        if (x1 < 140) and (y1 < 140):
            self.canvas.config(cursor="sizing")
            return True
        else:
            if not self.EDIT and not self.SCAN and not self.leapVar.get():
                self.canvas.config(cursor="none")
            return False

    ################################################

    #########################################       
    ### MOUSE Wheel Z axis function
    #########################################       
    def mouse_wheel(self, event):
        grid_base = int(self.width) / float(self.scale/float(2.5))
        
        
        if not self.onRoute and not self.robotVISION.get():
            if self.over_pin(event):
                if event.delta > 0:
                    self.pinANGLE += 1
                else:
                    self.pinANGLE -= 1
                    
                self.draw_bot(self.control_mode)
                
            else:
                if event.delta > 0:
                    if self.WINDOWS:
                        self.tZ = self.tZ + abs(event.delta/100.0)
                    else:
                        self.tZ = self.tZ + abs(event.delta)/10.0
                    
                    if self.tZ > self.Zlimit:
                        self.tZ = self.Zlimit
                    
                else:
                    if self.WINDOWS:
                        self.tZ = self.tZ - abs(event.delta/100.0)
                    else:
                        self.tZ = self.tZ - abs(event.delta)/10.0
                    
                    if self.tZ < 0.0:
                        self.tZ = 0.0
                    

                if self.EDIT:

                    X = (((self.mx-self.width/float(2)) * self.scale)/float(self.width))
                    Y = (((self.height - self.yShift*grid_base - self.my) * self.scale)/float(self.width))
                    index,side = self.get_traject(X,Y)
                    if index != -1:
                        self.dictTrajectories[index]['z'+str(side)] = self.tZ
                        
                        if side == 1 and (index > 0):
                            self.dictTrajectories[index-1]['z2'] = self.tZ
            
                        if side == 2 and (index < self.tIndex-1):
                            self.dictTrajectories[index+1]['z1'] = self.tZ

                    
                
                if self.control_mode != 1:
                    self.Z = self.tZ
                    
            

    #########################################
    ### TRAJECTORIES related functions
    #########################################
    def exec_trajectorie(self,x1,y1,z1,x2,y2,z2):
        self.onRoute = True
        # Number of points...
        
        _dist = abs(math.sqrt( ((x2-x1)*(x2-x1)) + ((y2-y1)*(y2-y1)) ))
        if self.ROBOT:
            if not self.SCAN:
                self.tSpeed = 40 #int(_dist)
            
        else:
            if not self.SCAN:
                self.tSpeed = int(_dist) + 10
           

        if self.tSpeed <= 1:
            self.tSpeed = 10

        
            

    
        #Execute trajectorie
        step = abs(x1 - x2)/float(self.tSpeed-1)
        stepZ = abs(z1 - z2)/float(self.tSpeed-1)
        stepA = (self.pinANGLE - self.robotPIN)/float(self.tSpeed-1)
        
        y = y1
        j = 0
        rw_tmp = None
        x = x1
        y = y1

        if self.RIGHT:
            _elbow = 1
        else:
            _elbow = 0


        
        self.buttonCL['state'] = 'disabled'
        self.buttonOP['state'] = 'disabled'
        self.buttonCLR['state'] = 'disabled'
        self.buttonPLAY['state'] = 'disabled'
        if not self.scanMODEvar.get():
            self.buttonZERO['state'] = 'disabled'
        self.buttonOBJECT['state'] = 'disabled'
        self.buttonCFGVISION['state'] = 'disabled'
        self.buttonCLR['state'] = 'disabled'

        self.buttonLEAP['state'] = 'disabled'
        self.buttonRT['state'] = 'disabled'
        self.buttonLines['state'] = 'disabled'
        self.buttonVISION['state'] = 'disabled'
                            
        
        for i in range(self.tSpeed):
            if self.robotSTOP:
                self.robotSTATUS = 'STOPED'
                self.onRoute = False
                self.buttonZERO['state'] = 'normal'
                return
            if x1 < x2:
                x = x + step
            else:
                x = x - step

            
            
            if np.round(x1,decimals=4) == np.round(x2,decimals=4):
                y += (y2-y1)/float(self.tSpeed)
            else:
                y = (((x-x1)*(y2-y1))/float(x2-x1))+y1

            
            if z1 < z2:
                z = z1 + (i*stepZ)
            else:
                z = z1 - (i*stepZ)

            self.mX = x
            self.mY = y

            if i < (self.tSpeed -1):
                self.check_limits(x,y)    
                self.Z = z
            else:
                self.check_limits(x2,y2)    
                self.Z = z2
                
        
                
            if self.robotAbs.get():
                self.robotPIN = self.pinANGLE + ((self.A1 + self.A2)%360) - 90
            else:
                self.robotPIN += stepA

            
            if self.ROBOT:
                if self.pinOPEN:
                    _ch5 = 0
                else:
                    _ch5 = 500
                
                if i < (self.tSpeed-1):
                    self.myRobot.trajectoryXYZ((self.X*10),(self.Y*10),self.Z+self.get_Z_correction(),ch4=int(self.translate(self.robotPIN+self.pinOFFSET,-65,65,0,1000)),ch5 = _ch5,num_point=i,elbow = _elbow)
                    
                else:
                    self.myRobot.trajectoryXYZ((self.X*10),(self.Y*10),self.Z+self.get_Z_correction(),ch4=int(self.translate(self.robotPIN+self.pinOFFSET,-65,65,0,1000)),ch5 = _ch5,num_point=i,last_point=1,sync=False,elbow = _elbow)
                    
                    
                    j = 0
                    while self.myRobot.isWorking():
                        if j > 5:
                            j = 0
                        self.canvas_logo.delete(rw_tmp)
                        rw_tmp = self.canvas_logo.create_image(300, -10, image=self.animation_rw[j], anchor="nw")
                        j += 1
                        self.draw_bot()
                        root.update_idletasks()
                        root.update()
                        
            else:
                self.draw_bot(0)
                # Draw Trajectories
                
                self.canvas.update()
                
                
                
        self.canvas_logo.delete(rw_tmp)

        self.onRoute = False
        self.update_labels()

        if not self.robotSTOP:
            if self.robotVISION.get():
               
                self.buttonLEAP['state'] = 'normal'
                self.buttonRT['state'] = 'normal'
                self.buttonLines['state'] = 'normal'
                self.buttonVISION['state'] = 'normal'
                self.buttonZERO['state'] = 'normal'

                self.buttonCFGVISION['state'] = 'normal'
                self.buttonOBJECT['state'] = 'normal'

            else:

                self.buttonCL['state'] = 'normal'
                self.buttonOP['state'] = 'normal'
                
                self.buttonPLAY['state'] = 'normal'
                self.buttonCLR['state'] = 'normal'

                self.buttonLEAP['state'] = 'normal'
                self.buttonRT['state'] = 'normal'
                self.buttonLines['state'] = 'normal'
                self.buttonVISION['state'] = 'normal'
                self.buttonZERO['state'] = 'normal'
        else:
            self.buttonZERO['state'] = 'normal'

    def clear_trajectories(self):
        if not self.onRoute:
            self.dictTrajectories = {}
            self.tIndex = 0
            self.canvasTI.delete('1.0', END)

        
        
        # disable EDIT button
        self.buttonEDIT['text'] = 'Edit Traject.\t'
        self.buttonEDIT['state'] = 'disabled'
        self.EDIT = False
        
            
    def play_trajectories(self):
        if not self.onRoute:
            if (self.control_mode == 1) and (self.tIndex > 0) and not self.SCAN:
                
                if not ((self.A1 == 90.0) and (self.A2 == 0.0)):
                    x1 = self.botX
                    y1 = self.botY
                    z1 = self.botZ

                    x2 = self.dictTrajectories[0]['x1']
                    y2 = self.dictTrajectories[0]['y1']
                    z2 = self.dictTrajectories[0]['z1']
                    self.pinANGLE = self.dictTrajectories[0]['pa']

                    self.tZ = z2
                    self.exec_trajectorie(x1,y1,z1,x2,y2,z2)

             
                for i in range(self.tIndex):
                    if self.robotSTOP:
                        break
                    x1 = self.dictTrajectories[i]['x1']
                    y1 = self.dictTrajectories[i]['y1']
                    z1 = self.dictTrajectories[i]['z1']

                    x2 = self.dictTrajectories[i]['x2']
                    y2 = self.dictTrajectories[i]['y2']
                    z2 = self.dictTrajectories[i]['z2']

                    self.pinANGLE = self.dictTrajectories[i]['pa']
                    self.pinOPEN = self.dictTrajectories[i]['po']

                    self.tZ = z2
                    self.exec_trajectorie(x1,y1,z1,x2,y2,z2)

               

            if self.SCAN:
                self.update_log("SCANNING...")
                self.update_labels()
                self.buttonABSANG['state'] = 'disabled'
                self.buttonLATERAL['state'] = 'disabled'
                self.buttonSCAN['state'] = 'disabled'
                self.surface_data = []
                self.pDY = []
                self.pDX = []
                self.pDZ = []
                self.pinOPEN = True
                self.tZ = 40
                self.update_log("GO to INIT position")
                self.update_labels()
                
                self.robotAbs.set(False)
                self.exec_trajectorie(self.X,self.Y,self.Z,-self._scanRange/10.0,self._dy,self.tZ)
                if self.ROBOT:
                    self.myRobot.setSpeedAcc(1,100,100,100)

                # Enable get data
                
                x = range(-self._scanRange,self._scanRange)
                y = range(0,2*self._scanRange)
                X,Y = np.meshgrid(x,y)
                self.surface_data = np.zeros((2*self._scanRange,2*self._scanRange))
                self.scanGETDATA = True
                _y = self._dy
                self.scanAstatusY = _y
                
                self.tSpeed = 40
                self.scanSTOP = False
                for i in range(2*self._scanRange):
                    self.update_log("SCAN data...")
                    self.update_labels()

                    if (i%2) == 0: #EVEN -> RIGHT
                        if self.robotSTOP or self.scanSTOP:
                            break
                        else:
                            self.exec_trajectorie(self.X,self.Y,self.Z,(self._scanRange-1)/10.0,_y-(i/10.0),self.tZ)
                    else:
                        if self.robotSTOP or self.scanSTOP:
                            break
                        else:
                            self.exec_trajectorie(self.X,self.Y,self.Z,-(self._scanRange-1)/10.0,_y-(i/10.0),self.tZ)

                    

                    root.update_idletasks()      
                    self.scanAstatusY = _y-(i/10.0)
                    
                    self.update_log("Processing SCAN data...")
                    self.update_labels()
                    if self.robotSTOP or self.scanSTOP:
                        break
                    else:
                        ## no zeros and no slope
                        k = int(2*self._scanRange - i) - 1
                        for j in range(1,2*self._scanRange-1):
                            if self.surface_data[k,j] == 0.0:
                                if (self.surface_data[k,j-1] != 0.0) or (self.surface_data[k,j+1] != 0.0):
                                    self.surface_data[k,j] = (self.surface_data[k,j-1] + self.surface_data[k,j+1])/2.0

                                  
                        self.plot_scan(X,Y)
                        
                        
                    


                # disable get data
                self.scanGETDATA = False
                self.surface_data = []
                if self.ROBOT:
                    self.myRobot.setSpeedAcc(100,100,100,100)

                self.pinANGLE = 0
                self.pinOPEN = False
                self.tZ = 0
                self.exec_trajectorie(self.X,self.Y,self.Z,0,200,0)
                self.buttonABSANG['state'] = 'normal'
                self.buttonLATERAL['state'] = 'normal'
                self.buttonSCAN['state'] = 'normal'
                
                
    ### DRAW SCAN from LIDAR   
    def plot_scan(self,X,Y):

        fig = plt.figure()

        ax = fig.add_subplot(111, projection='3d')
        ax.set_zlim([0,120])
        ax.set_ylim([0,2*self._scanRange])
        
        surf = ax.scatter3D(self.pDX,self.pDY, self.pDZ, c=self.pDZ, marker='.',cmap='winter')
        plt.savefig('./images/surface.png')#, transparent=True)
        ax.remove()
        _tmp = Image.open('./images/surface.png')
        _imx = self.canvas2_width-(24)*self.resizeFACTOR
        _tmp = _tmp.crop((100,40,590,460))
        self.scanIMAGE = _tmp.resize((int(_imx),int(_imx*0.75)), Image.ANTIALIAS)
        plt.close("all")
        
              
    def draw_trajectories(self,n):

        dot = 5

        grid_base = self.yShift*int(self.width) / float(self.scale/float(2.5))
        
        
        if self.tIndex > 0:
            x1 = self.dictTrajectories[0]['x1']
            y1 = self.dictTrajectories[0]['y1']
            z1 = self.dictTrajectories[0]['z1']
            self.canvas.create_oval(((x1*self.width)/self.scale) +  self.width/2 -1*dot ,self.height -grid_base - ((y1*self.width)/self.scale) -1*dot,
                                    ((x1*self.width)/self.scale) +  self.width/2 + dot,self.height -grid_base - ((y1*self.width)/self.scale) + dot, outline='white')

        self.canvasTI.delete('1.0', END)
        
        for i in range(self.tIndex):
            x1 = self.dictTrajectories[i]['x1']
            y1 = self.dictTrajectories[i]['y1']
            z1 = self.dictTrajectories[i]['z1']

            x2 = self.dictTrajectories[i]['x2']
            y2 = self.dictTrajectories[i]['y2']
            z2 = self.dictTrajectories[i]['z2']
            pa = self.dictTrajectories[i]['pa']
            po = self.dictTrajectories[i]['po']

            # insert text in TI
            self.canvasTI.insert("end",str(i) + "-[" + str(np.round(x2,decimals=1)) + "," + str(np.round(y2,decimals=1)) + "," + str(np.round(z2,decimals=1)) + "," + str(np.round(pa,decimals=0)) + "," + str(po) + "]\n")
            self.canvasTI.see("end")
                        
            if po:
                _color = "blue"
            else:
                _color = "white"

            _x1 = ((x1*self.width)/self.scale) + self.width/2
            _y1 = ((y1*self.width)/self.scale)
            _x2 = ((x2*self.width)/self.scale) + self.width/2
            _y2 = ((y2*self.width)/self.scale)

            self.canvas.create_line(_x1,self.height -grid_base - _y1,_x2,self.height -grid_base - _y2,fill="yellow", dash=(4,2))    
            self.canvas.create_oval(_x2 -1*(5+z2) ,self.height -grid_base - _y2 -1*(5+z2),_x2 + (5+z2),self.height -grid_base - _y2 + (5+z2), outline=_color)

            
            points = [[_x2 -1*(10+z2),self.height -grid_base - _y2],
                      [_x2 + (10+z2),self.height -grid_base - _y2],
                      [_x2 + (10+z2),self.height -grid_base - _y2],
                      [_x2 -1*(10+z2),self.height -grid_base - _y2]]
            center = [_x2,self.height -grid_base - _y2]
            
            A1 = -1*self.InverseKinematicA1(x2, -1*y2,self.len1,self.len2)
            A2 = -1*(self.InverseKinematicA2(x2, -1*y2,self.len1,self.len2)-180)
            if self.robotAbs.get():
                points = self.rotate(points, pa, center)
            else:
                points = self.rotate(points, pa - (A2 + A1) + 90, center)

            self.draw_square(points,color=_color)
            
            
            if i < 10:
                tOffset = 10
            else:
                tOffset = 15

            self.canvas.create_text(_x2 -1*(5+(z2/1.4)+tOffset) ,self.height -grid_base - _y2 -1*(5+(z2/1.4)+tOffset),fill="white",text="{}".format(i+1),anchor="nw")
            
            
            dot = 5 + z2

        if self.EDIT:
            
            index,side = self.get_traject(self.mX,self.mY)
            if index != -1:
                _X = self.dictTrajectories[index]['x'+str(side)]
                _Y = self.dictTrajectories[index]['y'+str(side)]
                _dot = 10 + self.dictTrajectories[index]['z'+str(side)]
                self.canvas.create_oval(((_X*self.width)/self.scale) +  self.width/2 -1*_dot ,self.height -grid_base - ((_Y*self.width)/self.scale) -1*_dot,
                                    ((_X*self.width)/self.scale) +  self.width/2 + _dot,self.height -grid_base - ((_Y*self.width)/self.scale) + _dot, outline='red', width=2.5)

                
    #########################################       
    #########################################       
    #########################################       

    #########################################       
    ### DRAW Mouse pointer target object
    #########################################       
    def draw_target(self, X2, Y2):
        # Draw target
        grid_base = int(self.width) / float(self.scale/float(2.5))
            
        X = (((X2-self.width/float(2)) * self.scale)/float(self.width))
        Y = (((self.height - self.yShift*grid_base - Y2) * self.scale)/float(self.width))
        X0 = self.width/2
        Y0 = self.height
        
        if not self.robotKINEMA.get() and self.mouseIN and not self.over_pin3(X2,Y2) and not self.EDIT and not self.SCAN:
            
            if (self.distance(X,Y) <= self.LIMIT) and (self.distance(X,Y) >= self.LIMITmin) and not self.outLIMIT and not self.robotSTOP:
                self.canvas.create_oval(X2-20, Y2-20, X2+20, Y2+20, outline="yellow", dash=(4,2))
                if self.WINDOWS: #antialiasing 
                    self.canvas.create_oval(X2-40, Y2-40, X2+40, Y2+40, outline="#AAA", width=2.5)
                self.canvas.create_oval(X2-40, Y2-40, X2+40, Y2+40, outline="yellow", width=2)
                
                self.canvas.create_line(X2-40,Y2,X2+40,Y2,fill="yellow", dash=(4,2))
                self.canvas.create_line(X2,Y2-40,X2,Y2+40,fill="yellow", dash=(4,2))

                
            else:
                self.canvas.create_oval(X2-20, Y2-20, X2+20, Y2+20, outline="red", dash=(4,2))
                if self.WINDOWS: #antialiasing
                    self.canvas.create_oval(X2-40, Y2-40, X2+40, Y2+40, outline="orange red", width=2.5)
                self.canvas.create_oval(X2-40, Y2-40, X2+40, Y2+40, outline="red", width=2)
                self.canvas.create_line(X2-40,Y2,X2+40,Y2,fill="red", dash=(4,2))
                self.canvas.create_line(X2,Y2-40,X2,Y2+40,fill="red", dash=(4,2))
                if self.robotSTOP:
                    self.canvas.create_text(X2 - 65,Y2 + 45 ,fill="red",text="EMERGENCY STOP. Press Zero...",anchor="nw")
                    

            dot = 5+self.tZ
            if not self.robotVISION.get():
                self.canvas.create_oval(X2-dot ,Y2-dot,X2+ dot,Y2+ dot, outline="white")
            self.canvas.create_text(X2 + 50,Y2 - 8,fill="white",text="{}".format(np.round(X*10, decimals=1)),anchor="nw")
            self.canvas.create_text(X2 - 10,Y2 - 65 ,fill="white",text="{}".format(np.round(Y*10, decimals=1)),anchor="nw")

            

        

    def go_zero(self):
        self.pinANGLE = 0
        if self.robotSTOP:     
            self.robotSTOP = False

            
            if self.ROBOT:
                self.robotSTATUS = 'ONLINE'
            else:
                self.robotSTATUS = 'OFFLINE'
            
            self.update_labels()

            self.buttonLEAP['state'] = 'normal'
            self.buttonRT['state'] = 'normal'
            self.buttonLines['state'] = 'normal'
            self.buttonVISION['state'] = 'normal'
            self.buttonRG['state'] = 'normal'
            self.buttonUP['state'] = 'normal'
            self.buttonDW['state'] = 'normal'

            if self.scanMODEvar.get():
                self.buttonPLAY['state'] = 'normal'
                

            if self.traVar.get():
                ### enable all interface...
                
                self.buttonDW['state'] = 'normal'
                self.buttonCL['state'] = 'normal'
                self.buttonOP['state'] = 'normal'
                self.buttonCLR['state'] = 'normal'
                self.buttonPLAY['state'] = 'normal'

                self.buttonEDIT['state'] = 'normal'
                self.buttonCLR['state'] = 'normal'

                

            if self.robotVISION.get():
                self.pinOPEN = False
                self.robotAbs.set(False)
                
                self.exec_trajectorie(self.X,self.Y,self.Z,-12.5,-10.,80)

                self.buttonRG['state'] = 'normal'
                self.buttonUP['state'] = 'normal'
                self.buttonDW['state'] = 'normal'
                
                self.buttonCFGVISION.config(state="normal")
                self.buttonOBJECT.config(state="normal")

            


        #else:
            
        if self.scanMODEvar.get():
            if not self.scanSTOP:
                self.scanSTOP = True
            else:
                self.tZ = 0.0
                self.pinOPEN = False
                self.pinANGLE = 0
                self.exec_trajectorie(self.botX,self.botY,self.botZ,0.0,20.0,0.0)
                

        if self.traVar.get():
            if not ((self.A1 == 90.0) and (self.A2 == 0.0)):
                self.tZ = 0.0
                self.pinOPEN = False
                self.pinANGLE = 0
                self.exec_trajectorie(self.botX,self.botY,self.botZ,0.0,20.0,0.0)
                

            
            
            
            if self.control_mode == 1 and self.tIndex > 0:
                # disable EDIT button
                self.buttonEDIT['text'] = 'Edit Traject.\t'
                self.buttonEDIT['state'] = 'normal'
                self.EDIT = False

            self.update_log("LOG: Return ZERO position")

    def get_traject(self, x, y):
        _range = 0.5
        index = -1
        side = 0
        
        for i in range(self.tIndex):
            x1 = self.dictTrajectories[i]['x1']
            y1 = self.dictTrajectories[i]['y1']
            z1 = self.dictTrajectories[i]['z1']

            x2 = self.dictTrajectories[i]['x2']
            y2 = self.dictTrajectories[i]['y2']
            z2 = self.dictTrajectories[i]['z2']

            if (abs(x1-x) < _range) and (abs(y1-y) < _range):
                index = i
                side = 1
                
            if (abs(x2-x) < _range) and (abs(y2-y) < _range):
                index = i
                side = 2

        return index,side
        
    def edit_move(self, event):

        grid_base = int(self.width) / float(self.scale/float(2.5))
        self.mx = event.x
        self.my = event.y
        if self.EDIT:
            X = (((event.x-self.width/float(2)) * self.scale)/float(self.width))
            Y = (((self.height - self.yShift*grid_base - event.y) * self.scale)/float(self.width))
            self.mX = X
            self.mY = Y
                
            if (self.distance(X,Y) <= self.LIMIT) and (self.distance(X,Y) >= self.LIMITmin) and not self.outLIMIT:
       
                index,side = self.get_traject(X,Y)
                if index != -1:
                    self.dictTrajectories[index]['x'+str(side)] = X
                    self.dictTrajectories[index]['y'+str(side)] = Y
                    
                    if side == 1 and (index > 0):
                        self.dictTrajectories[index-1]['x2'] = X
                        self.dictTrajectories[index-1]['y2'] = Y   
                    
                    if side == 2 and (index < self.tIndex-1):
                        self.dictTrajectories[index+1]['x1'] = X
                        self.dictTrajectories[index+1]['y1'] = Y


                self.draw_bot()
                
        
    def mi_go(self, event):
        if self.realVar.get():
            if self.pinOPEN:
                self.pinOPEN = False
            else:
                self.pinOPEN = True
                
            self.draw_bot(0)
            self.draw_target(event.x,event.y)
        else:
            if not self.onRoute and not self.robotSTOP and not self.leapVar.get() and self.traVar.get() and not self.EDIT:
                
                if self.control_mode == 1:
                    grid_base = int(self.width) / float(self.scale/float(2.5))
                    X = (((event.x-self.width/float(2)) * self.scale)/float(self.width))
                    Y = (((self.height - self.yShift*grid_base - event.y) * self.scale)/float(self.width))
                    Z = self.tZ
                    
                    if (self.distance(X,Y) <= self.LIMIT) and (self.distance(X,Y) >= self.LIMITmin):
                        if self.tIndex == 0:
                            lastX = self.X#self.botX
                            lastY = self.Y#self.botY
                            lastZ = self.Z#self.botZ
                        else:
                            lastX = self.dictTrajectories[self.tIndex-1]['x2']
                            lastY = self.dictTrajectories[self.tIndex-1]['y2']
                            lastZ = self.dictTrajectories[self.tIndex-1]['z2']
                        
                        x1 = lastX
                        x2 = X
                        y1 = lastY
                        y2 = Y
                        z1 = lastZ
                        z2 = Z
                        pa = self.pinANGLE
                        po = self.pinOPEN
                
                        
                        
                        
                        self.check_limits(x2,y2,False)
                        if not self.outLIMIT: # and index == -1:
                            #add trajectorie to dict
                            self.dictTrajectories[self.tIndex] = {'x1':x1,'y1':y1,'z1':z1,'x2':x2,'y2':y2,'z2':z2,'pa':pa,'po':po}
                            # enable EDIT button
                            self.buttonEDIT['state'] = 'normal'
                            self.tIndex += 1
                            #Execute trajectorie
                            self.exec_trajectorie(x1,y1,z1,x2,y2,z2)
                            
                        self.draw_target(event.x,event.y) 

    #########################################       
    ### FUNCTION: check intrinsic limitations of the robot
    #########################################       
    def check_limits(self,X,Y, mode=True):

        self.outLIMIT = False
        
        # Track mouse out if LIMITS for RealTime control      
        if (self.distance(X,Y) >= self.LIMIT):
            _angle = math.atan2(Y,X)
            X = self.LIMIT*math.cos((_angle))
            Y = self.LIMIT*math.sin((_angle))
        if (self.distance(X,Y) <= self.LIMITmin):

            
            _angle = abs(math.atan2(Y,X))
            X = self.LIMITmin*math.cos((_angle))
            Y = self.LIMITmin*math.sin((_angle))

        #Check RIGHT or LEFT Handed
        
        if self.RIGHT:
            _A1 = -1*self.InverseKinematicA1(X, -1*Y,self.len1,self.len2)
            _A2 = -1*(self.InverseKinematicA2(X, -1*Y,self.len1,self.len2)-180)
        else:
            _A1 = self.InverseKinematicA1(X, Y,self.len1,self.len2)
            _A2 = (self.InverseKinematicA2(X, Y,self.len1,self.len2)-180)
            
        if _A1 < 0:
            _A1 = 360 + _A1

        if _A2 < 0:
            _A2 = 360 + _A2
    

        if ((_A1 > 336) or (_A1 < 204)):
            # Check limits only or make it real
            if mode:
                
                self.A1 = _A1
                self.A2 = _A2
                self.X = X
                self.Y = Y
        else:
            self.outLIMIT = True

              
        
    def mi_move(self, event):
        
        if not self.onRoute and not self.leapVar.get():
            grid_base = int(self.width) / float(self.scale/float(2.5))
            X = (((event.x-self.width/float(2)) * self.scale)/float(self.width))
            Y = (((self.height - self.yShift*grid_base - event.y) * self.scale)/float(self.width))

            self.mX = X
            self.mY = Y
            self.mx = event.x
            self.my = event.y
            
            
            if self.control_mode == 0 and not self.robotSTOP:
                self.check_limits(X,Y)
                
            else:
                self.check_limits(X,Y, False)
                

            self.draw_bot(self.control_mode)

                
    #########################################       
    ### GEOMETRICS related FUNCTIONS
    #########################################       

    def lawOfCosines(self,a,b,c):
        number = np.clip((a*a + b*b - c*c) / (2 * a * b),-1.0,1.0)
        return math.acos(number)

    def distance(self,x,y):
        return math.sqrt(x*x + y*y)

    def InverseKinematicA1(self,x, y,len1,len2):
        dist = self.distance(x,y);
        D1 = math.atan2(y,x);
        D2 = self.lawOfCosines(dist, len1, len2);
        return np.rad2deg((D1+D2));

    def InverseKinematicA2(self, x, y, len1, len2):
        dist = self.distance(x,y);
        return np.rad2deg(self.lawOfCosines(len1,len2,dist));

    def rotate(self, points, angle, center):
        angle = math.radians(angle)
        cos_val = math.cos(angle)
        sin_val = math.sin(angle)
        cx, cy = center
        new_points = []
        for x_old, y_old in points:
            x_old -= cx
            y_old -= cy
            x_new = x_old * cos_val - y_old * sin_val
            y_new = x_old * sin_val + y_old * cos_val
            new_points.append([x_new + cx, y_new + cy])
        return new_points

    def draw_square(self,points, color="gray", ol="white"):
        self.canvas.create_polygon(points, outline=ol,fill=color)

    def translate(self,value, leftMin, leftMax, rightMin, rightMax):
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return rightMin + (valueScaled * rightSpan)

    #########################################       
    ### COLLECTING Objects related FUNCTIONS
    #########################################       
    def get_object(self,X,Y,Xf=-13.5,Yf=-4.5,Zf=60):
        self.check_limits(X,Y,False)
        if self.outLIMIT:
            self.exec_trajectorie(self.X,self.Y,self.Z,0,20,50)
            self.RIGHT = False
            
            self.check_limits(X,Y,False)
            
        if not self.outLIMIT:
    
            self.robotAbs.set(True)
            if self.RIGHT:
                self.exec_trajectorie(self.X,self.Y,self.Z,-17.5,7,60)
            self.pinOPEN = True
            
            self.exec_trajectorie(self.X,self.Y,self.Z,X,Y,50)
            self.exec_trajectorie(self.X,self.Y,self.Z,X,Y,0)
            self.pinOPEN = False
            self.exec_trajectorie(self.X,self.Y,self.Z,X,Y,50)

            if not self.RIGHT:
                self.exec_trajectorie(self.X,self.Y,self.Z,0,20,50)
                
            self.pinANGLE = 0
            self.robotAbs.set(False)
            self.RIGHT = True
            self.exec_trajectorie(self.X,self.Y,self.Z,Xf,Yf,Zf)
            self.pinOPEN = True
            self.exec_trajectorie(self.X,self.Y,self.Z,Xf,Yf,80)
            
            

    def objects_in_range(self, objects_list):

        for i in range(len(objects_list)):
            if (self.distance(objects_list[i]['x'],objects_list[i]['y']) <= self.LIMIT):
                return True

        return False
            
            
        
    def get_allOBJ(self):
        if self.CAMERA and bool(self.objects):
            _tmpObjects = self.objects
            
            
            #################################
            '''    
            for i in range(len(_tmpObjects)):
                _X = _tmpObjects[i]['x']
                _Y = _tmpObjects[i]['y']
                _angle = _tmpObjects[i]['angle']
            '''
            while self.objects_in_range(self.objects):
                for i in range(len(self.objects)):
                    _X = self.objects[i]['x']
                    _Y = self.objects[i]['y']
                    _angle = self.objects[i]['angle']

                    ### Check angle limits...
                    if (self.distance(_X,_Y) >= self.LIMIT):
                        continue
                    self.check_limits(_X,_Y,False)
                    if self.outLIMIT:
                        if _angle > 90:
                            _angle += 180
                        else:
                            _angle += 90

                    else:
                        if _angle > 90:
                            _angle -= 180
                        else:
                            _angle -= 90
                    
                    self.pinANGLE = _angle

                    #self.recollecting = True
                    self.get_object(_X,_Y,)
                    #self.recollecting = False
                    
                    break
                    
            #################################

                

            self.pinOPEN = False
            self.robotAbs.set(False)
            self.exec_trajectorie(self.X,self.Y,self.Z,-12.5,-10,80)
            
    def camera2real(self, cX, cY):

        
        ### ROTATION CORERCTION
        cos_val = math.cos(self.cam_angle)
        sin_val = math.sin(self.cam_angle)
        cx = self.points[4][0]
        cy = self.points[4][1]
        
        cX -= cx
        cY -= cy
        rX = cX * cos_val - cY * sin_val
        rY = cX * sin_val + cY * cos_val
        rX += cx
        rY += cy
        ### TRANSLATION CORRECTION
        rX = ((rX - self.points[4][0]) * self.pixelX_size)/float(10)
        rY = (100 + ((self.points[4][1]-rY)*self.pixelY_size))/float(10)
        
        if abs(math.degrees(self.cam_angle)) > 90:
            rX = -1*rX
            rY = -1*rY + 20

        return rX,rY

    #########################################       
    ### VISION PIPELINE
    #########################################       
    def draw_cam(self, mode):

        # Draw camera
        if self.CAMERA: #and not self.processing:
            rval, self.image_pipe["image raw BGR"] = self.vc.read()
            # vision processing....
            ############### COLOR DETECTION ################
            
            if rval:
                
                img_yuv = cv2.cvtColor(self.image_pipe["image raw BGR"], cv2.COLOR_BGR2YUV)
                # equalize the histogram of the Y channel
                img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])
                # convert the YUV image back to RGB format
                frame = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
                frame_rgb = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2RGB)
                self.image_pipe["equalized"] = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2RGB)
                
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                
                
                lower_c = np.array([self.HSVColor['Hl'],self.HSVColor['Sl'],self.HSVColor['Vl']]) 
                upper_c = np.array([self.HSVColor['Hu'],self.HSVColor['Su'],self.HSVColor['Vu']])

                (r,g,b) = colorsys.hsv_to_rgb(self.HSVColor['Hu']/180., self.HSVColor['Su']/255., self.HSVColor['Vu']/255.)
                _rgb = "#%02x%02x%02x" % (r*255,g*255,b*255)
                
                # Here we are defining range of bluecolor in HSV 
                # This creates a mask of blue coloured  
                # objects found in the frame. 
                mask = cv2.inRange(hsv, lower_c, upper_c)
                  
                # The bitwise and of the frame and mask is done so  
                # that only the blue coloured objects are highlighted  
                # and stored in res 
                self.image_pipe["color mask"] = cv2.bitwise_and(frame,frame, mask= mask)
                
                h, s, self.image_pipe["image gray"] = cv2.split(self.image_pipe["color mask"]) #### HSV 2 GRAY
                
                # gaussian blur
                self.image_pipe["image blur"] = cv2.GaussianBlur(self.image_pipe["image gray"],(5,5),0)
                ################ CONTOUR DETECTION #############
                
                ret,self.image_pipe["image binary"] = cv2.threshold(self.image_pipe["image blur"],self.b_threshold,255,0)
                if self.WINDOWS:
                    contours, hierarchy = cv2.findContours(self.image_pipe["image binary"],cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
                else:
                    il, contours, hierarchy = cv2.findContours(self.image_pipe["image binary"],cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
                    
                if not self.drawingOBJ:
                    self.captureOBJ = True
                    self.objects = {}
                    i = 0
                    for c in contours:
                        #cv2.drawContours(frame, contours, i, (0,255,0), 3)
                        M = cv2.moments(c)
                        if (float(M["m00"]) <> 0.0) and (cv2.contourArea(c) > self.area_min) and (cv2.contourArea(c) < self.area_max) and not self.recollecting:
                            cv2.drawContours(frame_rgb, [c], -1, (0,255,0), 3)
                            cX = int(M["m10"] / float(M["m00"]))
                            cY = int(M["m01"] / float(M["m00"]))
                            cv2.line(frame_rgb, (0,cY),(cX,cY),(0, 255, 0), 2)
                            cv2.line(frame_rgb, (cX,0),(cX,cY),(0, 255, 0), 2)
                            
                            X,Y = self.camera2real(cX,cY)
                            
                            
                            (x,y),(MA,ma),angle = cv2.fitEllipse(c)
                            
                            self.objects[i] = {'x':X,'y':Y,'angle':round(angle), 'MA':MA, 'ma': ma, 'contour': c, 'color':_rgb}
                            i+=1
                            
                    
                    
                    self.captureOBJ = False    
                ################################################

                ############### MISC OPERATIONS ################
                # grayscale image is used for equalization
                #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # following function performs equalization on input image
                #equ = cv2.equalizeHist(gray)

                # sobel
                #x_sobel = cv2.Sobel(gray,cv2.CV_64F,1,0,ksize=5)
                #y_sobel = cv2.Sobel(gray,cv2.CV_64F,0,1,ksize=5)

                # laplacian
                #lapl = cv2.Laplacian(gray,cv2.CV_64F, ksize=5)

                # gaussian blur
                #blur = cv2.GaussianBlur(gray,(5,5),0)

                # laplacian of gaussian
                #log = cv2.Laplacian(blur,cv2.CV_64F, ksize=5)

                #Canny filter
                #frame = cv2.Canny(frame,100,200)
                #################################################
                # CAMERA calibration lines
                cv2.line(frame_rgb, (self.points[0][0],self.points[0][1]),(self.points[1][0],self.points[1][1]),(255, 255, 255), 2)
                cv2.line(frame_rgb, (self.points[2][0],self.points[2][1]),(self.points[3][0],self.points[3][1]),(255, 255, 255), 2)
                _text = "Rot: {}".format(round(math.degrees(self.cam_angle),2))
                cv2.putText(frame_rgb,_text, (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
                _text = "Pixel: {}mm".format(round(self.pixelX_size,2))
                cv2.putText(frame_rgb,_text, (10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
                
                # draw area min reference
                _offset = self.l_area_min
                cv2.rectangle(frame_rgb, (self.camX/8-(_offset),int(self.camY/4)-(_offset)),(self.camX/8+(_offset),int(self.camY/4)+(_offset)),(0, 0, 255), 2)
                # draw area max reference
                _offset = self.l_area_max
                cv2.rectangle(frame_rgb, (self.camX/8-(_offset),int(self.camY/4)-(_offset)),(self.camX/8+(_offset),int(self.camY/4)+(_offset)),(255, 0, 0), 2)
                self.image_pipe["object detection"] = frame_rgb

                frame_rgb = self.image_pipe[self.show_image]
                        
                    
                _imx = self.canvas2_width-(24)*self.resizeFACTOR
                self.frame = Image.fromarray(frame_rgb).resize((int(_imx),int(_imx*0.75)), Image.ANTIALIAS)
                
                
                
    #########################################       
    ### DRAW template WORK SPACE
    #########################################       
    def draw_template(self, mode):
        radio_base = int(self.width) / float(self.scale/float(5))
        grid_base = int(self.width) / float(self.scale/float(2.5))
        grid_base2 = int(self.width) / float(self.scale/float(1.25))
        _out = False

        if mode == 0:   
            
            self.canvas.configure(background='PaleGreen4')
            self.canvas2.configure(background='PaleGreen4')
            for i in range(1,int(self.width/grid_base)):
                self.canvas.create_line(0, self.height - (grid_base * i),self.width, self.height - (grid_base * i) , fill="gray80")
                self.canvas.create_line(self.width/2 + (grid_base * i), 0, self.width/2 +(grid_base * i), self.height, fill="gray80")
                self.canvas.create_line(self.width/2 - (grid_base * i), 0, self.width/2 -(grid_base * i), self.height, fill="gray80")
                
            for i in range(1,int(self.width/grid_base2)):    
                self.canvas2.create_line(0, self.height - (grid_base2 * i),self.width, self.height - (grid_base2 * i) , fill="gray80")
                self.canvas2.create_line(grid_base2 * i, 0, grid_base2 * i, self.height, fill="gray80")

            self.canvas.create_line(self.width/float(2), self.height,self.width/float(2),0 , fill="white")

            
            for i in range(1,5):
                radio = radio_base*i
                if self.WINDOWS: #antialiasing
                    self.canvas.create_oval((self.width/2)-(radio), (self.height)-(radio+self.yShift*grid_base), (self.width/2)+(radio), (self.height)+(radio-self.yShift*grid_base),outline="gray80", width=2.5)
                    
                self.canvas.create_oval((self.width/2)-(radio), (self.height)-(radio+self.yShift*grid_base), (self.width/2)+(radio), (self.height)+(radio-self.yShift*grid_base),outline="white", width=2)
                self.canvas.create_text(self.width/float(2) + (radio) - 25,self.height - 15,fill="white",text="{}".format(50*i),anchor="nw", font=("Purisa", 10))

                if self.WINDOWS:
                    self.canvas.create_text(self.width/float(2) + 5,self.height - (radio+self.yShift*grid_base) + 5, fill="white",text="{}".format(50*i),anchor="nw", font=("Purisa", 10))
                else:
                    self.canvas.create_text(self.width/float(2),self.height - (radio+self.yShift*grid_base) + 25, angle = 90, fill="white",text="{}".format(50*i),anchor="nw", font=("Purisa", 10))
                
            self.canvas.create_line(self.width, self.height,self.width,0 , fill="black", width = 2)
            self.canvas2.create_line(0, self.height,0,0 , fill="black", width = 2)
          
        if mode == 1:

                
            self.canvasImage = self.canvas.create_image(0, 0, image=self.im, anchor="nw")


            self.canvas2.configure(background='PaleGreen4')
            for i in range(1,int(self.width/grid_base2)):
                self.canvas2.create_line(0, self.height - (grid_base2 * i),self.width, self.height - (grid_base2 * i) , fill="gray80")
                self.canvas2.create_line(grid_base2 * i, 0, grid_base2 * i, self.height, fill="gray80")


        if self.robotKINEMA.get():
            self.canvasImage = self.canvas.create_image(0, 0, image=self.bboard, anchor="nw")
            
            self.canvas_bp_side = self.canvas.create_image(self.width- 250*self.resizeFACTOR, 20*self.resizeFACTOR, image=self.bpSide, anchor="nw")
            self.canvas_bp_plant = self.canvas.create_image(self.width-230*self.resizeFACTOR, self.height - 100*self.resizeFACTOR, image=self.bpPlant, anchor="nw")

        if (self.distance(self.mX,self.mY) > self.LIMIT):
            radio = radio_base*4
            if self.WINDOWS: #antialiasing
                self.canvas.create_oval((self.width/2)-(radio), (self.height)-(radio+self.yShift*grid_base), (self.width/2)+(radio), (self.height)+(radio-self.yShift*grid_base),outline="orange red", width=4.5)
                
            self.canvas.create_oval((self.width/2)-(radio), (self.height)-(radio+self.yShift*grid_base), (self.width/2)+(radio), (self.height)+(radio-self.yShift*grid_base),outline="red", width=4)
            _out = True
            
        if (self.distance(self.mX,self.mY) < self.LIMITmin):
            radio = radio_base*2-20
            if self.WINDOWS: #antialiasing
                self.canvas.create_oval((self.width/2)-(radio), (self.height)-(radio+self.yShift*grid_base), (self.width/2)+(radio), (self.height)+(radio-self.yShift*grid_base),outline="orange red", width=4.5)

            self.canvas.create_oval((self.width/2)-(radio), (self.height)-(radio+self.yShift*grid_base), (self.width/2)+(radio), (self.height)+(radio-self.yShift*grid_base),outline="red", width=4)
            _out = True

        if self.RIGHT:
            _A1 = -1*self.InverseKinematicA1(self.mX, -1*self.mY,self.len1,self.len2)
        else:
            _A1 = self.InverseKinematicA1(self.mX, self.mY,self.len1,self.len2)
            
        if _A1 < 0:
            _A1 = 360 + _A1

        X1 = (self.width/2) + ((9*math.cos(np.deg2rad(_A1)))*self.width)/self.scale
        Y1 = (self.height) - (((9*math.sin(np.deg2rad(_A1)))*self.width)/self.scale) - self.yShift*grid_base


        if self.mouseIN and not self.over_pin3(self.mx,self.my) and not self.EDIT and not self.robotVISION.get() and not self.SCAN:
            if self.outLIMIT or _out:
                
                self.canvas.create_line(self.width/float(2),self.height- self.yShift*grid_base,X1, Y1, fill="red",width=3,dash=(4,4))
                self.canvas.create_line(X1, Y1, self.mx, self.my, fill="red",width=3,dash=(4,4))
            else:
                if self.control_mode == 1 and not self.onRoute:
                    
                    self.canvas.create_line(self.width/float(2),self.height- self.yShift*grid_base,X1, Y1, fill="yellow",width=3,dash=(4,4))
                    self.canvas.create_line(X1, Y1, self.mx, self.my, fill="yellow",width=3,dash=(4,4))

        
        if self.SCAN or self.CustomDialog_show:
            _sr,p_dy = self.mm2pixels(0 - self._scanRange/10.0,self._dy)

            _sr2,p_dy2 = self.mm2pixels(0 + self._scanRange/10.0,self._dy-(2*self._scanRange/10))

            _trash,p_dy3 = self.mm2pixels(0 + self._scanRange/10.0,(self.scanAstatusY))
            
            self.canvasImage = self.canvas.create_rectangle(_sr, p_dy,_sr2,p_dy2, outline='black', width=3)
            
            self.canvasImage = self.canvas.create_rectangle(_sr, p_dy,_sr2,p_dy3, fill= 'blue', outline='black', width=1)
                    
       

    def mm2pixels(self,X,Y):
        grid_base = int(self.width) / float(self.scale/float(2.5))
        X2 = ((X*self.width)/self.scale) +  self.width/2
        Y2 = self.height - ((Y*self.width)/self.scale) - self.yShift*grid_base

        return X2,Y2

    #########################################       
    ### THE BIG ONE. Draw Robot. KINEMATIC STUFF
    #########################################       
    def draw_bot(self, mode=0):

        self._pOPEN = 500
        
        
        
        if mode == 0:
            
            self.botX = self.X
            self.botY = self.Y
            self.botZ = self.Z

        
            
            if self.ROBOT:

                if self.RIGHT:
                    self._elbow = 1
                else:
                    self._elbow = 0
                    
                if self.realVar.get() or self.leapVar.get():

                    if self.pinOPEN:
                        if self.SCAN:
                            self._pOPEN = 450
                        else:
                            self._pOPEN = 0
                        
                    else:
                        self._pOPEN = 500
                        

        if self.realVar.get():        
            if self.robotAbs.get():        
                self.robotPIN = (self.pinANGLE + ((self.A1 + self.A2)%360) - 90)
                if self.robotPIN > 180:
                    self.robotPIN -= 360
                
            else:
                self.robotPIN = self.pinANGLE
            
        if self.ROBOT:
            
            data = self.robotDATA
            self.A1 = float(data[1])/float(10) + 90
            self.A2 = float(data[2])/float(10)
            
            _x1 = self.len1*math.cos(np.deg2rad(self.A1))
            _y1 = self.len1*math.sin(np.deg2rad(self.A1))
            self.botX = _x1 + (self.len2 * math.cos(np.deg2rad(self.A1 + self.A2)))
            self.botY = _y1 + (self.len2 * math.sin(np.deg2rad(self.A1 + self.A2)))
            self.botZ = float(data[3])/float(10)
            

        if not self.realVar.get():        
            if self.robotAbs.get():        
                self.robotPIN = (self.pinANGLE + ((self.A1 + self.A2)%360) - 90)
                if self.robotPIN > 180:
                    self.robotPIN -= 360
                
            else:
                self.robotPIN = self.pinANGLE   
        
        self.get_lidar()   
        
        grid_base = int(self.width) / float(self.scale/float(2.5))
        
       
        X1 = (self.width/2) + ((self.len1*math.cos(np.deg2rad(self.A1)))*self.width)/self.scale
        Y1 = (self.height) - (((self.len1*math.sin(np.deg2rad(self.A1)))*self.width)/self.scale) - self.yShift*grid_base

        
        X2 = ((self.botX*self.width)/self.scale) +  self.width/2
        Y2 = self.height - ((self.botY*self.width)/self.scale) - self.yShift*grid_base
        
        
        
        
        self.canvas.delete("all")
        self.canvas2.delete("all")
        self.draw_template(self.templateMODE)

        
        
        if self.CAMERA and bool(self.objects) and not self.captureOBJ:
            #### DRAW OBJECTS #####
            self.drawingOBJ = True
            for i in range(len(self.objects)):
                _X = self.objects[i]['x']
                _Y = self.objects[i]['y']
                _angle = self.objects[i]['angle']
                _MA = self.objects[i]['MA']
                _ma = self.objects[i]['ma']
                _color = self.objects[i]['color']
                #_c = self.objects['c']
                _size = int(_MA)/2.7
                _size2 = int(_ma)/2.7
                _X = ((_X*self.width)/self.scale) +  self.width/2
                _Y = self.height - ((_Y*self.width)/self.scale) - self.yShift*grid_base
        
                centerP = [_X,_Y]
                pointsP = [[_X-(_size*self.resizeFACTOR2),_Y-(_size2*self.resizeFACTOR2)],
                           [_X+(_size*self.resizeFACTOR2),_Y-(_size2*self.resizeFACTOR2)],
                           [_X+(_size*self.resizeFACTOR2),_Y+(_size2*self.resizeFACTOR2)],
                           [_X-(_size*self.resizeFACTOR2),_Y+(_size2*self.resizeFACTOR2)]]
        
        
            
                pointsP = self.rotate(pointsP, _angle, centerP)
                self.draw_square(pointsP,color=_color, ol='black')

                #cv2.drawContours(frame_rgb, [_c], -1, (0,255,0), 3)
                self.canvas.create_text(_X+25,_Y, text="Obj[{}] - {}º".format(i,_angle),fill="white",anchor="nw")
                
        self.drawingOBJ = False
        #Update parameters
        x1 = 9*math.cos(np.deg2rad(self.A1))
        y1 = 9*math.sin(np.deg2rad(self.A1))
                        
        self.canvas2.create_text(10,5,fill="white",text="Angles: {}º, {}º, {}º".format(np.round(self.A1,decimals=1),np.round(self.A2,decimals=1),np.round(self.robotPIN,decimals=1)),anchor="nw")
        self.canvas2.create_text(10,25,fill="white",text="Position (mm): {},{},{} LIDAR: {}".format(np.round(self.X*10,decimals=1),np.round(self.Y*10,decimals=1),np.round(self.Z,decimals=1),np.round(self.lidar_data[19],decimals=2)),anchor="nw")


        if self.LIDAR and not self.robotKINEMA.get():
            self.draw_lidar()
        ###### draw simple kinematics        
                
       
        
        # Draw PIN
        if self.pinOPEN:
            isOPEN = "OPEN"
        else:
            isOPEN = "CLOSED"
            
        PofX = -15*self.resizeFACTOR
        PofY = -15*self.resizeFACTOR
        points = [[25*self.resizeFACTOR + PofX,65*self.resizeFACTOR + PofY],[125*self.resizeFACTOR + PofX,65*self.resizeFACTOR + PofY],[125*self.resizeFACTOR + PofX,85*self.resizeFACTOR + PofY],[25*self.resizeFACTOR + PofX,85*self.resizeFACTOR + PofY]]
        center = [75*self.resizeFACTOR + PofX,75*self.resizeFACTOR + PofY]
        

        #rotate
        
        pointsR = self.rotate(points, self.robotPIN, center)
        pointsL = self.rotate(points, self.pinANGLE, center)
            
        self.draw_square(pointsL,color='gray29')

        if (self.robotPIN < -65) or (self.robotPIN > 65):
            self.draw_square(pointsR,color='red4')
        else:
            self.draw_square(pointsR,color='grey')
            
            
        
        
        if self.WINDOWS: #antialiasing
            self.canvas.create_oval(25*self.resizeFACTOR + PofX, 25*self.resizeFACTOR + PofY, 125*self.resizeFACTOR + PofX, 125*self.resizeFACTOR + PofY,outline="gray80", width=1.5)
            self.canvas.create_oval(45*self.resizeFACTOR + PofX, 45*self.resizeFACTOR + PofY, 105*self.resizeFACTOR + PofX, 105*self.resizeFACTOR + PofY,outline="gray80", width=1.5)

        self.canvas.create_oval(25*self.resizeFACTOR + PofX, 25*self.resizeFACTOR + PofY, 125*self.resizeFACTOR + PofX, 125*self.resizeFACTOR + PofY,outline="white")
        self.canvas.create_oval(45*self.resizeFACTOR + PofX, 45*self.resizeFACTOR + PofY, 105*self.resizeFACTOR + PofX, 105*self.resizeFACTOR + PofY,outline="white")

        self.canvas.create_line(25*self.resizeFACTOR + PofX, 75*self.resizeFACTOR + PofY, 125*self.resizeFACTOR + PofX,75*self.resizeFACTOR + PofY,fill="white", dash=(4,2))
        self.canvas.create_line(75*self.resizeFACTOR + PofX,25*self.resizeFACTOR + PofY,75*self.resizeFACTOR + PofX,125*self.resizeFACTOR + PofY,fill="white", dash=(4,2))
        if not self.robotKINEMA.get():
            self.canvas.create_text(120*self.resizeFACTOR + PofX,25*self.resizeFACTOR + PofY,fill="white",text="{}º({}º) - {}".format(self.pinANGLE,int(self.robotPIN),isOPEN),anchor="nw")

        
        
       
        
        # Draw joints
        #### Draw PIM ######
        aw,ah = self.pinzaL_r.size
        # Translate coord to (0,0) center
        _X2 = 0
        _Y2 = ah/float(3.5)#15*self.resizeFACTOR

        # Rotate angle
        rot_angle = np.clip(self.robotPIN,-65,65) - ((self.A1+self.A2)%360) + 90
        rot_angle = -rot_angle
        
        # Rotate image from center
        self.pinzaL_tk = self.pinzaL_r.rotate(rot_angle, expand=True)#, resample=Image.BICUBIC)
        aw,ah = self.pinzaL_tk.size
        self.pinzaL_tk = ImageTk.PhotoImage(self.pinzaL_tk)
        # Calculate new offset
        
        convX = (_X2) * math.cos(np.deg2rad(rot_angle)) - (_Y2) * math.sin(np.deg2rad(rot_angle)) 
        convY = (_X2) * math.sin(np.deg2rad(rot_angle)) + (_Y2) * math.cos(np.deg2rad(rot_angle))

        # Translate coord to (0,0) top-left
        convX = aw/float(2) + convX
        convY = ah/float(2) - convY
        
        self.canvasImage = self.canvas.create_image(X2-(convX), Y2-(convY), image=self.pinzaL_tk, anchor="nw")
        
        # Joint 2
        
        aw,ah = self.forearmL_r.size
        # Translate coord to (0,0) center
        _X2 = 0
        _Y2 = ah/float(2) - (ah/float(9))#(30*self.resizeFACTOR)

        # Rotate angle
        rot_angle = self.A2-(90-self.A1)

        # Rotate image from center
        self.forearmL_tk = self.forearmL_r.rotate(rot_angle, expand=True)#, resample=Image.BICUBIC)
        aw,ah = self.forearmL_tk.size
        self.forearmL_tk = ImageTk.PhotoImage(self.forearmL_tk)

        # Calculate new offset
        
        convX = (_X2) * math.cos(np.deg2rad(rot_angle)) - (_Y2) * math.sin(np.deg2rad(rot_angle)) 
        convY = (_X2) * math.sin(np.deg2rad(rot_angle)) + (_Y2) * math.cos(np.deg2rad(rot_angle))

        # Translate coord to (0,0) top-left
        convX = aw/float(2) + convX
        convY = ah/float(2) - convY

        self.canvasImage = self.canvas.create_image(X2-(convX), Y2-(convY), image=self.forearmL_tk, anchor="nw")
        
        
        # Joint 1
        #### Draw BASE #####
        aw,ah = self.armL_r.size
        # Translate coord to (0,0) center
        _X2 = -2
        _Y2 = ah/float(2) - (ah/float(5.2))#(52*self.resizeFACTOR)

        # Rotate angle
        rot_angle = self.A1 - 90

        # Rotate image from center
        self.armL_tk = self.armL_r.rotate(rot_angle, expand=True)#, resample=Image.BICUBIC)
        aw,ah = self.armL_tk.size
        self.armL_tk = ImageTk.PhotoImage(self.armL_tk)

        # Calculate new offset
        
        convX = (_X2) * math.cos(np.deg2rad(rot_angle)) - (_Y2) * math.sin(np.deg2rad(rot_angle)) 
        convY = (_X2) * math.sin(np.deg2rad(rot_angle)) + (_Y2) * math.cos(np.deg2rad(rot_angle))

        # Translate coord to (0,0) top-left
        convX = aw/float(2) + convX
        convY = ah/float(2) - convY

        self.canvasImage = self.canvas.create_image(X1-(convX), Y1-(convY), image=self.armL_tk, anchor="nw")
        
        ####################
        #### Draw BASE #####
        
        aw,ah = self.baseL_r.size
        self.baseL_tk = ImageTk.PhotoImage(self.baseL_r)
        self.canvasImage = self.canvas.create_image(self.width/2 - (aw/2) - (3), self.height - ((ah/float(2.2))+(self.yShift-2)*grid_base), image=self.baseL_tk, anchor="nw")
        
        
        
        
        
        Z_draw = self.botZ - self.Z_offset + self.pinzaR.height()
        
        self.canvasImage = self.canvas2.create_image(self.canvas2.winfo_width()-(self.base_im.width()+self.arm_im.width()/float(17)), self.canvas2.winfo_height()-(self.base_im.height()), image=self.base_im, anchor="nw")
        self.canvasImage = self.canvas2.create_image(self.canvas2.winfo_width()-(self.arm_im.width()), self.canvas2.winfo_height()-(Z_draw+(self.arm_im.height()-(self.base_im.height()/float(6.4)))), image=self.arm_im, anchor="nw")

        if not self.pinOPEN:
            self.canvasImage = self.canvas2.create_image(self.canvas2.winfo_width()-(self.arm_im.width()+(self.pinzaR.width()/float(5))), self.canvas2.winfo_height()-(Z_draw + (self.pinzaR.height()/float(11))), image=self.pinzaR, anchor="nw")
        else:
            self.canvasImage = self.canvas2.create_image(self.canvas2.winfo_width()-(self.arm_im.width()+(self.pinzaR_Op.width()/float(4)) ), self.canvas2.winfo_height()-(Z_draw), image=self.pinzaR_Op, anchor="nw")
        

        #Z Command
        self.canvas2.create_line(30*self.resizeFACTOR, self.canvas2.winfo_height()-1*self.resizeFACTOR, 30*self.resizeFACTOR, self.canvas2.winfo_height()-(self.tZ+(self.pinzaR.height()/float(4))), fill="white", width=2*self.resizeFACTOR, dash=(4,2))
        self.canvas2.create_line(25*self.resizeFACTOR, self.canvas2.winfo_height() - (self.tZ+(self.pinzaR.height()/float(6))), self.canvas2.winfo_width()-(self.arm_im.width()/float(1.14)), self.canvas2.winfo_height()-(self.tZ+(self.pinzaR.height()/float(6))), fill="white", width=2*self.resizeFACTOR)#, dash=(4,2))
        self.canvas2.create_text(15*self.resizeFACTOR, self.canvas2.winfo_height()-(self.tZ+37*self.resizeFACTOR),fill="white",text="{}".format(np.round(self.tZ,decimals=1)),anchor="nw")


        
            
        if self.robotKINEMA.get():
            if self.WINDOWS:
                _font = ("DK Crayon Crumble", 16)#36)
                _font_l = ("DK Crayon Crumble", 16)#24)

                
            else:
                _font = ("DK Crayon Crumble", 24)#36)
                _font_l = ("DK Crayon Crumble", 16)# 24)


            
            _X1 = (self.width/2) + ((15*math.cos(np.deg2rad(self.A1)))*self.width)/self.scale
            _Y1 = (self.height) - (((15*math.sin(np.deg2rad(self.A1)))*self.width)/self.scale) - self.yShift*grid_base

            _Xa1 = (self.width/2) + ((7*math.cos(np.deg2rad(self.A1/float(2))))*self.width)/self.scale
            _Ya1 = (self.height) - (((7*math.sin(np.deg2rad(self.A1/float(2))))*self.width)/self.scale) - self.yShift*grid_base

            self.canvas.create_text(10,160,fill="white",text="Kinematics:",anchor="nw",font=_font_l)
            self.canvas.create_text(10,185,fill="white",text="x1 = L1 cos(A1)",anchor="nw",font=_font_l)
            self.canvas.create_text(10,205,fill="white",text="y1 = L1 sin(A1)",anchor="nw",font=_font_l)
            self.canvas.create_text(10,225,fill="white",text="x2 = x1 + L2 cos(A1+A2)",anchor="nw",font=_font_l)
            self.canvas.create_text(10,245,fill="white",text="y2 = y1 + L2 sin(A1+A2)",anchor="nw",font=_font_l)

            self.canvas.create_text(120*self.resizeFACTOR + PofX,25*self.resizeFACTOR + PofY,fill="white",text="{}({}) - {}".format(self.pinANGLE,int(self.robotPIN),isOPEN),anchor="nw",font=_font)

            
            self.canvas.create_line(self.canvas.winfo_width()/float(2), self.canvas.winfo_height()-(self.yShift*grid_base), self.canvas.winfo_width()/float(2) + (5)*(grid_base), self.canvas.winfo_height()-(self.yShift*grid_base), fill="white", width=4)#, dash=(4,2))
            self.canvas.create_line(self.canvas.winfo_width()/float(2), self.canvas.winfo_height()-(self.yShift*grid_base), self.canvas.winfo_width()/float(2), self.canvas.winfo_height()-((self.yShift+5)*grid_base), fill="white", width=4)#, dash=(4,2))
            self.canvas.create_text(self.canvas.winfo_width()/float(2) + 5*(grid_base) + 20, self.canvas.winfo_height()-(self.yShift*grid_base),fill="white",text="X",anchor="nw",font=_font)
            self.canvas.create_text(self.canvas.winfo_width()/float(2) - grid_base, self.canvas.winfo_height()-((self.yShift+5)*grid_base),fill="white",text="Y",anchor="nw",font=_font)
            
            self.canvas.create_line(self.canvas.winfo_width()/float(2), self.canvas.winfo_height()-(self.yShift*grid_base), _X1, _Y1, fill="white", width=4, dash=(8,2))
            self.canvas.create_text(_Xa1, _Ya1-25,fill="white",text="A1 = {}".format(np.round(self.A1,decimals=1)),anchor="nw",font=_font)

            self.canvas.create_line(X1, Y1, X2, Y2, fill="white", width=4, dash=(8,2))
            self.canvas.create_text(_X1, _Y1-30,fill="white",text="A2 = {}".format(np.round(self.A2,decimals=1)),anchor="nw",font=_font)

            self.canvas.create_text(X1+20, Y1-20,fill="white",text="(x1,y1)={},{}".format(np.round(x1,decimals=1),np.round(y1,decimals=1)),anchor="nw",font=_font)
            self.canvas.create_text(X2, Y2-40,fill="white",text="(x2,y2)={},{}".format(np.round(self.X,decimals=1),np.round(self.Y,decimals=1)),anchor="nw",font=_font)
        
            # Draw L1 and L2
            self.canvas.create_text(X1-((X1 - self.canvas.winfo_width()/float(2))/float(2))-40, Y1 + (((self.canvas.winfo_height()-self.yShift*grid_base) - Y1)/float(2))-40,fill="white",text="L1".format(np.round(self.A2,decimals=1)),anchor="nw",font=_font)
            self.canvas.create_text(X2 - ((X2-X1)/float(2)) - 40, Y2 + ((Y1-Y2)/float(2)) - 40,fill="white",text="L2".format(np.round(self.A2,decimals=1)),anchor="nw",font=_font)


            
            self.canvas.create_arc(self.canvas.winfo_width()/float(2)-(2*grid_base+10), self.canvas.winfo_height()-((self.yShift+2)*grid_base+10), self.canvas.winfo_width()/float(2) + 2*(grid_base+10), self.canvas.winfo_height()-((self.yShift-2)*grid_base)+10, outline="white", width=4, extent=self.A1+5, start=0, style=Tkinter.ARC)
            self.canvas.create_arc(X1 - (2*grid_base), Y1-(2*grid_base+10), X1 + 2*(grid_base), Y1+(2*grid_base), outline="white", width=4, start= self.A1-5, extent=self.A2+5, style=Tkinter.ARC)

        
        
        if self.VIDEO:    
            if self.CAMERA:# and (self.frame != None): #and (self.control_mode == 1):
                
                if self.frame:
                    _imx,_imy = self.frame.size
                    self.canvasImage = self.canvas2.create_rectangle(10*self.resizeFACTOR, 40*self.resizeFACTOR,10*self.resizeFACTOR + _imx + 4,40*self.resizeFACTOR + _imy + 4, fill='black', outline='black', width=3)
                    self._image = ImageTk.PhotoImage(self.frame)
                    
                    self.canvasImage = self.canvas2.create_image(12*self.resizeFACTOR, 42*self.resizeFACTOR, image=self._image, anchor="nw")

            
        if self.SCAN:
            if self.scanIMAGE:
                _imx,_imy = self.scanIMAGE.size
                self.canvasImage = self.canvas2.create_rectangle(10*self.resizeFACTOR, 40*self.resizeFACTOR,10*self.resizeFACTOR + _imx + 4,40*self.resizeFACTOR + _imy + 4, fill='black', outline='black', width=3)
                self._image = ImageTk.PhotoImage(self.scanIMAGE)
                
                self.canvasImage = self.canvas2.create_image(12*self.resizeFACTOR, 42*self.resizeFACTOR, image=self._image, anchor="nw")

            


        if self.control_mode == 1:
            self.draw_trajectories(0)   
        
        if not self.leapVar.get() and not self.onRoute:
            self.draw_target(self.mx,self.my)

        
        self.change_icons_status()
        
    #########################################       
    ### WINDOW Reconfigure EVENT. RESIZE
    #########################################       
    def configure(self, event):
        self.width_shift = 0#(self.canvas.winfo_width()/5)
        root.update_idletasks()
        self.width = self.canvas.winfo_width()
        self.height = self.canvas.winfo_height()
        self.canvas2_width = self.canvas2.winfo_width()

        
        self.scale = (self.width/float(self.height))*(float(self.SCREEN_SCALE)+self.yShift)
        
        
        self.resizeFACTOR = 1 + ((float(self.width - 700))/float(self.width))
        self.resizeFACTOR2 = self.resizeFACTOR

       
        _factor = self.scale/float(11) - ((self.scale/float(11)) /float(100))*((self.resizeFACTOR-1)*100)
        
        
        
        bw,bh = self.base_im_l.size
        
        self.base_im_r = self.base_im_l.resize( ( int((bw/float(_factor))) , int((bh/float(_factor)))), Image.ANTIALIAS)
        self.base_im = ImageTk.PhotoImage(self.base_im_r)

        
        aw,ah = self.arm_im_l.size
        
        self.arm_im_r = self.arm_im_l.resize( ( int((aw/float(_factor))) , int((ah/float(_factor)))), Image.ANTIALIAS)
        self.arm_im = ImageTk.PhotoImage(self.arm_im_r)

        
        aw,ah = self.pinzaR_l.size
        
        self.pinzaR_r = self.pinzaR_l.resize( ( int((aw/float(_factor))) , int((ah/float(_factor)))), Image.ANTIALIAS)
        self.pinzaR = ImageTk.PhotoImage(self.pinzaR_r)

        _factor = 11.3 - (11.3/float(100))*((self.resizeFACTOR-1)*100)
        
        
        aw,ah = self.pinzaR_Op_l.size
        
        self.pinzaR_Op_r = self.pinzaR_Op_l.resize( ( int((aw/float(_factor))) , int((ah/float(_factor)))), Image.ANTIALIAS)
        self.pinzaR_Op = ImageTk.PhotoImage(self.pinzaR_Op_r)


        
        _factor = self.scale/float(21.98) - ((self.scale/float(21.98)) /float(100))*((self.resizeFACTOR-1)*100)
        

        
        
        aw,ah = self.armL_l.size
        self.armL_r = self.armL_l.resize(( int( (aw/float(_factor)) ), int( (ah/float(_factor)) ) ), Image.ANTIALIAS)

        
        aw,ah = self.forearmL_l.size
        self.forearmL_r = self.forearmL_l.resize((int( (aw/float(_factor)) ), int( (ah/float(_factor)))), Image.ANTIALIAS)
        

        
        aw,ah = self.baseL_l.size
        self.baseL_r = self.baseL_l.resize((int( (aw/float(_factor)) ), int( (ah/float(_factor)))), Image.ANTIALIAS)

        
        aw,ah = self.pinzaL_l.size
        self.pinzaL_r = self.pinzaL_l.resize((int( (aw/float(_factor)) ), int( (ah/float(_factor)))), Image.ANTIALIAS)

        
        aw,ah = self.bboard_l.size
        self.bboard_r = self.bboard_l.resize((self.width, self.height), Image.ANTIALIAS)
        self.bboard = ImageTk.PhotoImage(self.bboard_r)
        
        
        self._bp_side_r = self._bp_side.resize((int(250*self.resizeFACTOR),int(150*self.resizeFACTOR)), Image.ANTIALIAS)
        self.bpSide = ImageTk.PhotoImage(self._bp_side_r)

        
        self._bp_plant_r = self._bp_plant.resize((int(215*self.resizeFACTOR),int(90*self.resizeFACTOR)), Image.ANTIALIAS)
        self.bpPlant = ImageTk.PhotoImage(self._bp_plant_r)

        if self.scanIMAGE:
            _tmp = Image.open('./images/surface.png')
            _tmp = _tmp.crop((100,40,590,460))
            _imx = self.canvas2_width-(24)*self.resizeFACTOR
            self.scanIMAGE = self.scanIMAGE.resize((int(_imx),int(_imx*0.75)), Image.ANTIALIAS)
            
        if self.grid_img:

            self.grid_img = self.grid_img.resize((self.width,self.height), Image.ANTIALIAS)
            self.im = ImageTk.PhotoImage(self.grid_img)
        
        self.canvas_logo.delete("all")
        self.load_image()
        
        
    def onCloseInfoFrame(self, otherFrame):
        print("destroy")
        otherFrame.destroy()
        

    #########################################       
    ### DEPRECATED. Not USE
    #########################################       
    def show_info(self, event):
        
        subFrame = InfoFrame()
        handler = lambda: self.onCloseInfoFrame(subFrame)
        
        btn = Tkinter.Button(subFrame.top_frame2, text="  Cerrar  ", command=handler)
        btn.grid(row=3, column=1, sticky="e")

    #########################################       
    ### LEAP MOTION MODE
    #########################################       
    def chLeap(self):
        
        if self.realVar.get():
            self.buttonRT.toggle()
        if self.traVar.get():
            self.buttonLines.toggle()
        if self.robotVISION.get():
            self.buttonVISION.invoke()
        if self.scanMODEvar.get():
            self.buttonSCAN.invoke()

        self.robotAbs.set(True)

         
        # disable buttons
        self.buttonCL['state'] = 'disabled'
        self.buttonOP['state'] = 'disabled'
        self.buttonZERO['state'] = 'disabled'
        self.buttonPLAY['state'] = 'disabled'
        self.buttonCLR['state'] = 'disabled'

        self.control_mode = 0
        self.clear_trajectories()

        self.update_log("LOG: Enter LeapMotion control mode")
        
        self.canvas.config(cursor="arrow")
        
        controller = Leap.Controller()
        


        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);
        

        finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
        bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
        state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']

        pinza = False
        pinzaL = False
     
        last_X = 0.0
        last_Y = 0.0
        last_Z = 0.0

        last_XL = 0.0
        last_YL = 0.0
        last_ZL = 0.0
        
        index = 0
        
        while self.leapVar.get():
            frame = controller.frame()
            for hand in frame.hands:

                if hand.is_right:
                    self.RIGHT = True
                else:
                    self.RIGHT = False

                
                direction = hand.direction

                # Calculate the hand's pitch, roll, and yaw angles
                
                clamp_yaw = self.translate(direction.yaw * Leap.RAD_TO_DEG,-45,45,200,600)

                
                self.pinANGLE = int(np.round(direction.yaw * Leap.RAD_TO_DEG,decimals=0))
                if self.robotAbs.get():
                    self.robotPIN = self.pinANGLE + ((self.A1 + self.A2)%360) - 90
                else:
                    self.robotPIN = self.pinANGLE
                #

                for finger in hand.fingers:
                    if finger.type == 1:
                        
                        findice = frame.pointable(finger.id)
                        if findice.is_extended:
                            self.pinOPEN = True
                            
                        else:
                            self.pinOPEN = False
                            
                # 40,300 --> 0,300
                data = int(hand.palm_position[1])
                if (data < 200):
                    data = 200
                if data > 500 :
                    data = 500
                Z = self.translate(data,200,500,0,120)

                #200-20 --> -200-200
                data = int(hand.palm_position[2])
                if (data < -220):
                    data = -220
                if data > 220 :
                    data = 220
                X = (data)/float(8) - 10 #self.translate(data,-220,220,300,0)
                X =  -X
                #-250-250 --> -50,50
                data = int(hand.palm_position[0])
                if (data < -170):
                    data = -170
                if data > 170 :
                    data = 170
                
                Y = data/float(8) #self.translate(data,-10,320,-170,330)
                #Y = -Y
                
               
                self.check_limits((last_Y*0.4)+(Y*0.6),(last_X*0.4)+(X*0.6))
                if not self.outLIMIT:
                    #self.X = (last_Y*0.4)+(Y*0.6)
                    #self.Y = (last_X*0.4)+(X*0.6)
                    self.Z = (last_Z*0.4)+(Z*0.6)
                    last_X = X
                    last_Y = Y
                    last_Z = Z
                    
                   
            root.update()
            self.draw_bot(0)
            
            
        
    #########################################       
    ### REALTIME MODE
    #########################################       
    def chReal(self):
        if self.leapVar.get():
            self.buttonLEAP.toggle()
        if self.traVar.get():
            self.buttonLines.toggle()
        if self.robotVISION.get():
            self.buttonVISION.invoke()
            
        if self.scanMODEvar.get():
            self.buttonSCAN.invoke()

        # disable buttons
        self.buttonCL['state'] = 'disabled'
        self.buttonOP['state'] = 'disabled'
        self.buttonZERO['state'] = 'disabled'
        self.buttonPLAY['state'] = 'disabled'
        self.buttonCLR['state'] = 'disabled'
            
        self.control_mode = 0
        self.clear_trajectories()

        if GAMEPAD:
            gamepad_connection_handler()

        self.update_log("LOG: Enter RealTime control mode")

        
    #########################################       
    ### TRAYECTORIE MODE
    #########################################       
    def chTra(self):
        
        if self.realVar.get():
            self.buttonRT.toggle()
        if self.leapVar.get():
            self.buttonLEAP.toggle()
        if self.robotVISION.get():
            self.buttonVISION.invoke()
        if self.scanMODEvar.get():
            self.buttonSCAN.invoke()

        self.buttonCL['state'] = 'normal'
        self.buttonOP['state'] = 'normal'
        self.buttonZERO['state'] = 'normal'
        self.buttonPLAY['state'] = 'normal'
        self.buttonCLR['state'] = 'normal'
        
        self.control_mode = 1
        

        self.update_log("LOG: Enter Trajectories control mode")
        
    #########################################       
    ### ENABLE/DISABLE ROBOT Hardware
    #########################################       
    def chRobot(self):
        
        self.buttonROBOT.toggle()

        if self.robotVar.get():
            self.robotVar.set(False)
            self.ROBOT = False
            self.rt2.stop()
            
            self.robotSTATUS = 'OFFLINE'
            self.update_log("LOG: Robot OFFLINE")
            self.LIDAR_ON = False
           
        else:
            self.robotVar.set(True)
            
            
            if self.myRobot == None:
                if not self.WINDOWS:
                    try:
                        with Timeout(5):
                            self.robotSTATUS = 'SEARCHING ROBOT...'
                            self.update_labels()
                            
                            self.myRobot = PyBotArm()
                            
                            if self.connection_type == "WIFI":
                                r_status = self.myRobot.connect()
                            else:
                                r_status = self.myRobot.connect("USB",self.connection_type)
                                
                            if r_status:
                            
                                self.robotSTATUS = 'ONLINE'
                                self.rt2 = RepeatedTimer(0.05, self.get_robotDATA)
                                time.sleep(0.5)
                                self.ROBOT = True
                                #test LIDAR
                                data = self.myRobot.getDistance()
                                if data != -1:
                                    self.LIDAR_ON = True
                                else:
                                    self.LIDAR_ON = False
                            else:
                                self.robotSTATUS = 'ERROR - ROBOT OFFLINE'
                                self.robotVar.set(False)
                                self.ROBOT = False
                                self.myRobot = None
                                self.LIDAR_ON = False

                    except Timeout.Timeout:
                        self.robotVar.set(False)
                        self.ROBOT = False
                        self.robotSTATUS = 'ERROR - OFFLINE'
                        self.update_log("LOG: ERROR Robot NOT Present")
                        self.myRobot = None
                        self.LIDAR_ON = False
                else: # if WINDOWS not signal alarm available... ugly crash!!!
                    self.robotSTATUS = 'SEARCHING ROBOT...'
                    self.update_labels()
                    #self.myRobot = SCARA()
                    self.myRobot = PyBotArm()
                    if self.connection_type == "WIFI":
                        r_status = self.myRobot.connect()
                    else:
                        r_status = self.myRobot.connect("USB",self.connection_type)
                        
                    if r_status:
                        self.robotSTATUS = 'ONLINE'
                        self.rt2 = RepeatedTimer(0.05, self.get_robotDATA)
                        time.sleep(0.5)
                        self.ROBOT = True
                        #test LIDAR
                        data = self.myRobot.getDistance()
                        if data != -1:
                            self.LIDAR_ON = True
                        else:
                            self.LIDAR_ON = False
                            
                    else:
                        self.robotSTATUS = 'ERROR - ROBOT OFFLINE'
                        self.robotVar.set(False)
                        self.ROBOT = False
                        self.myRobot = None
                        self.LIDAR_ON = False
                    #self.robotSTATUS = 'ONLINE'    
                
            else:
                self.robotSTATUS = 'ONLINE'
                self.rt2 = RepeatedTimer(0.05, self.get_robotDATA)
                time.sleep(0.5)
                self.ROBOT = True
                #test LIDAR
                data = self.myRobot.getDistance()
                if data != -1:
                    self.LIDAR_ON = True
                else:
                    self.LIDAR_ON = False
                
            
        if self.ROBOT:
            self.myRobot.setSpeedAcc(self.param_dict["xy_speed"],self.param_dict["z_speed"],self.param_dict["xy_accel"],self.param_dict["z_accel"])
                
        self.update_labels()

    #########################################       
    ### SET CLAMP MODE
    #########################################       
    def chAbs(self):
        self.buttonABSANG.toggle()
        if self.robotAbs.get():
            self.robotAbs.set(False)
            self.update_log("LOG: PIN angle relative mode")
            
        else:
            self.robotAbs.set(True)
            self.update_log("LOG: PIN angle absolute mode")

    #########################################       
    ### SCAN Objects MODE
    #########################################       
    def scanMODE(self):

        if not self.SCAN:
            if self.realVar.get():
                self.buttonRT.toggle()
            if self.leapVar.get():
                self.buttonLEAP.toggle()
            if self.traVar.get():
                self.buttonLines.toggle()
            if self.robotVISION.get():
                self.buttonVISION.invoke()

            # disable buttons
            self.buttonCL['state'] = 'disabled'
            self.buttonOP['state'] = 'disabled'
            self.buttonZERO['state'] = 'disabled'
            
            self.buttonCLR['state'] = 'disabled'
            self.buttonPLAY['state'] = 'normal'
                
            
            self.clear_trajectories()
            self.update_log("LOG: Enter SCAN Mode...")
            
        if self.SCAN:
            self.SCAN = False
        else:
            self.SCAN = True
            self.control_mode = 1
            self.canvas.config(cursor="arrow")
        
    def rbotBLOCKLY(self):
        global b_myRobot
        

        b_myRobot = self
        #Create the HTTP (TCP) server to redirect messages to API robot
        self.BlocklyServer = HTTPServer(('localhost', HTTP2UDP_PORT), HTTP2UDP)
        self.tBlockly = ServerThread(self.BlocklyServer)
        self.tBlockly.setDaemon(True) # don't hang on exit
        self.tBlockly.start()

        self.update_log("LOG: BLOCKLY Server running...")
        
    
            
    #########################################       
    ### VISION MODE
    #########################################       
    def chVision(self):
    
        self.buttonVISION.toggle()
        if self.robotVISION.get():
            self.robotVISION.set(False)
        else:
            self.robotVISION.set(True)
            
        if self.robotVISION.get():
            self.pinANGLE = 0

            self.clear_trajectories()
            
            # disable buttons
            self.buttonCL['state'] = 'disabled'
            self.buttonOP['state'] = 'disabled'
            self.buttonZERO['state'] = 'disabled'
            self.buttonPLAY['state'] = 'disabled'
            self.buttonCLR['state'] = 'disabled'

            
            
            if self.realVar.get():
                self.buttonRT.toggle()
                self.realVar.set(False)
                self.control_mode = 1
            if self.traVar.get():
                self.buttonLines.toggle()
                self.traVar.set(False)
            if self.leapVar.get():
                self.buttonLEAP.toggle()
                self.leapVar.set(False)
            if self.scanMODEvar.get():
                self.buttonSCAN.invoke()
            
            self.processing = False
            self.objects = {}
            if self.PS3eye:
                self.vc = Camera()

            else:
                self.vc = cv2.VideoCapture(0)
                self.vc.set(cv2.CAP_PROP_FRAME_WIDTH, self.camX)
                self.vc.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camY)
            
            self.pinOPEN = False
            self.robotAbs.set(False)
            self.tZ = 80
            self.exec_trajectorie(self.X,self.Y,self.Z,-12.5,-10.,80)

            self.CAMERA = True
            self.rt = RepeatedTimer(0.1, self.draw_cam,0)

            

            self.buttonCFGVISION.config(state="normal")
            self.buttonOBJECT.config(state="normal")
            
            
           
        else:
            self.buttonOBJECT.config(state=DISABLED)
            self.buttonCFGVISION.config(state=DISABLED)
            self.rt.stop()
            
            time.sleep(1.5)
            self.CAMERA = False
            if self.PS3eye:
                self.cv.end()

            else:
                self.vc.release()
            self.objects = {}

            self.go_zero()

            
            
    #########################################       
    ### ROBOT ELBOW set
    #########################################       
    def chLATERAL(self):
        if self.robotLATERAL.get():
            self.RIGHT = True
        else:
            self.RIGHT = False

        self.update_labels()
        
            
    #########################################       
    ### FILES related functions
    #########################################       
    def save_file(self):
        filename = tkfd.asksaveasfilename(initialdir = "./json",title = "Select file",filetypes = (("json files","*.json"),("all files","*.*")))
        if self.WINDOWS:
            filename = filename + ".json"
            
        if self.control_mode == 1 and filename:
            if self.tIndex > 0:
                tmp = json.dumps(self.dictTrajectories, sort_keys=True,indent=4, separators=(',', ': '))
                f = open(filename,"w")
                f.write(tmp)
                f.close()
        
    def open_file(self):
        filename = tkfd.askopenfilename(initialdir = "./json",title = "Select file",filetypes = (("json files","*.json"),("all files","*.*")))
        if self.control_mode == 1 and filename:
            #load trajectories
            with open(filename) as handle:
                tmp = json.loads(handle.read())

            self.tIndex = len(tmp)
            for i in range(len(tmp)):
                self.dictTrajectories[i] = tmp[str(i)]
                
            self.draw_bot(self.control_mode)
            self.buttonEDIT['state'] = 'normal'
            
            
    def open_template(self):
        filename = tkfd.askopenfilename(initialdir = "./templates",title = "Select file",filetypes = (("template files","*.jpg"),("all files","*.*")))
        if filename:
            self.grid_img = Image.open(filename)
            self.grid_img = self.grid_img.resize((self.width,self.height), Image.ANTIALIAS)
            self.im = ImageTk.PhotoImage(self.grid_img)

            self.templateMODE = 1

        else:
            self.templateMODE = 0

    
    #########################################
    ### DIALOGS Calls
    #########################################       
    def vision_config(self):
        if not self.VisionDialog_show:
            self.param_vision = VisionDialog(self, self.param_vision).show()
            self.VisionDialog_show = True
        

    def robot_calibration(self):
        if not self.CalibrationDialog_show:
            CalibrationDialog(self, None).show()
            self.CalibrationDialog_show = True

        
        
    def robot_config(self):
        if not self.CustomDialog_show:
            CustomDialog(self, self.param_dict).show()
            self.CustomDialog_show = True
        
        
    def edit_tra(self):
        if self.EDIT:
            self.canvas.config(cursor="none")
            self.buttonEDIT['text'] = 'Edit Traject.\t'
            self.EDIT = False
        else:
            self.canvas.config(cursor="arrow")
            self.buttonEDIT['text'] = 'End Edition\t'
            self.EDIT = True

    #########################################       
    ### EMERGENCY STOP
    #########################################       
    def robot_stop(self):
        
            
        self.robotSTOP = True
        self.onRoute = False
        # disable EDIT button
        self.buttonEDIT['text'] = 'Edit Traject.\t'
        self.buttonEDIT['state'] = 'disabled'
        self.EDIT = False

        ### Call to API

        ### Disable all interface...
        self.buttonRG['state'] = 'disabled'
        self.buttonUP['state'] = 'disabled'
        self.buttonDW['state'] = 'disabled'
        self.buttonCL['state'] = 'disabled'
        self.buttonOP['state'] = 'disabled'
        self.buttonCLR['state'] = 'disabled'
        self.buttonPLAY['state'] = 'disabled'
        self.buttonOBJECT['state'] = 'disabled'
        self.buttonCFGVISION['state'] = 'disabled'
        self.buttonEDIT['state'] = 'disabled'
        self.buttonCLR['state'] = 'disabled'

        self.buttonLEAP['state'] = 'disabled'
        self.buttonRT['state'] = 'disabled'
        self.buttonLines['state'] = 'disabled'
        self.buttonVISION['state'] = 'disabled'

        self.buttonZERO['state'] = 'normal'

        self.update_labels()
        
    #########################################       
    ### MOUSE related FUNCTIONS
    #########################################       
    def b2_motion(self, event):
        if self.realVar.get():
            X1 = ((self.botX*self.width)/40) +  self.width/2
            Y1 = self.height - ((self.botY*self.width)/40)
            
            
            if self.B2_dist < (X1 - event.x):
                self.pinANGLE -= 1
            else:
                self.pinANGLE += 1

            self.B2_dist = (X1 - event.x)
            
        

    def b2_release(self, event):
        if self.realVar.get():
            X1 = ((self.botX*self.width)/40) +  self.width/2
            Y1 = self.height - ((self.botY*self.width)/40)

            root.event_generate('<Motion>', warp=True, x=X1, y=Y1+self.canvas_logo.winfo_height())
            
    def b2_click(self,event):
        
        if self.traVar.get():
            if self.pinOPEN:
                self.pinOPEN = False
            else:
                self.pinOPEN = True

            if self.EDIT:
                grid_base = int(self.width) / float(self.scale/float(2.5))
                
                X = (((self.mx-self.width/float(2)) * self.scale)/float(self.width))
                Y = (((self.height - self.yShift*grid_base - self.my) * self.scale)/float(self.width))
                index,side = self.get_traject(X,Y)
                
                
                if side == 1 and index != 0:
                    self.dictTrajectories[index-1]['po'] = self.pinOPEN
                if side == 2:
                    self.dictTrajectories[index]['po'] = self.pinOPEN
                    
                    
                    
            self.draw_bot(self.control_mode)
            
            self.draw_target(event.x,event.y)
            
    
    def mouse_enter(self, event):
        self.mouseIN = True

    def mouse_leave(self, event):
        self.mouseIN = False

    ##################################
    # SENT and GET Robot DATA
    ##################################
    def get_Z_correction(self):
        # _x = self.X*10 always ZERO for calibration
        _y = self.Y*10
        ## Three POINTS calibration (Two lines aproximation)
        if (_y > 150.0):
            _y1 = 150
            _y2 = 200
            _z1 = self.z_points[1]
            _z2 = self.z_points[0]
            
        else:
            _y1 = 100
            _y2 = 150
            _z1 = self.z_points[2]
            _z2 = self.z_points[1]


        _Z_offset = (((_y - _y1)/float(_y2 - _y1))*(_z2 - _z1)) + _z1

        #print(_Z_offset)
        self.Z_offset = _Z_offset

        return _Z_offset
            
        
        
    def get_robotDATA(self):
        _status = self.myRobot.getStatus()
        self.robotDATA = _status.split(",")
        
        if self.control_mode == 0:
            
            self._pANGLE = (self.translate(self.robotPIN+self.pinOFFSET,-65,65,0,1000))
               
            self.myRobot.moveXYZ((self.X*10),(self.Y*10),self.Z+self.get_Z_correction(),self._pANGLE,self._pOPEN, t=0.0, sync=False, elbow = self._elbow)

            
    def change_icons_status(self):
        if self.leap_ready:
            self.canvas_logo.delete(self.lp_logo)
            self.lp_logo = self.canvas_logo.create_image(480, 20, image=self.leap_logo, anchor="nw")
        else:
            self.canvas_logo.delete(self.lp_logo)
            self.lp_logo = self.canvas_logo.create_image(480, 20, image=self.not_leap_logo, anchor="nw")

        if self.gamepad_ready:
            self.canvas_logo.delete(self.gp_logo)
            self.gp_logo = self.canvas_logo.create_image(400, 20, image=self.gamepad_logo, anchor="nw")
        else:
            self.canvas_logo.delete(self.gp_logo)
            self.gp_logo = self.canvas_logo.create_image(400, 20, image=self.not_gamepad_logo, anchor="nw")
        
        if self.blocklyCMD:
            self.canvas_logo.delete(self.bl_logo)
            self.bl_logo = self.canvas_logo.create_image(560, 20, image=self.blockly_logo, anchor="nw")
        else:
            self.canvas_logo.delete(self.bl_logo)
            self.bl_logo = self.canvas_logo.create_image(560, 20, image=self.not_blockly_logo, anchor="nw")
            
    #########################################       
    ### INIT MAIN CLASS
    #########################################       
    def __init__(self, main):

        self.main = main
        if self.PS3eye:
            from pseyepy import Camera

        self.counter = 1
        self.INZOOM = False
        self.ERROR_GENERAL = ""

        self.leapVar = Tkinter.BooleanVar()
        self.leapVar.set(False)
        self.realVar = Tkinter.BooleanVar()
        self.realVar.set(False)
        self.traVar = Tkinter.BooleanVar()
        self.traVar.set(True)
        self.robotVar = Tkinter.BooleanVar()
        self.robotVar.set(False)
        self.robotAbs = Tkinter.BooleanVar()
        self.robotAbs.set(False)
        self.robotVISION = Tkinter.BooleanVar()
        self.robotVISION.set(False)

        self.robotLATERAL = Tkinter.BooleanVar()
        self.robotLATERAL.set(True)

        self.robotKINEMA = Tkinter.BooleanVar()
        self.robotKINEMA.set(False)

        self.robotBLOCKLY = Tkinter.BooleanVar()
        self.robotBLOCKLY.set(False)

        self.scanMODEvar = Tkinter.BooleanVar()
        self.scanMODEvar.set(False)

        
        self.param_dict["xy_speed"] = 100
        self.param_dict["z_speed"] = 100
        self.param_dict["xy_accel"] = 100
        self.param_dict["z_accel"] = 100
        self.param_dict["connection"] = "WIFI"
        self.param_dict["LIDAR"] = True
        self.param_dict["VIDEO"] = True
        self.param_dict["SCALE"] = 32
        self.param_dict["yOFFSET"] = 6
        self.param_dict["scanAREA"] = 25
        '''
        self.param_dict["color"] = "Yellow"
        self.param_dict["amin"] = 300
        self.param_dict["amax"] = 600
        '''
        ######## OpenCV ####################
        # default yellow color
        self.color_wide_H = 9
        self.color_wide_S = 108
        self.color_wide_V = 96
        self.HSVColor['H'] = 30
        self.HSVColor['S'] = 147
        self.HSVColor['V'] = 159

        self.HSVColor['Hl'] = int(self.HSVColor['H'] - self.color_wide_H) # 21
        self.HSVColor['Sl'] = int(self.HSVColor['S'] - self.color_wide_S) # 39
        self.HSVColor['Vl'] = int(self.HSVColor['V'] - self.color_wide_V) # 64
        self.HSVColor['Hu'] = int(self.HSVColor['H'] + self.color_wide_H) # 40
        self.HSVColor['Su'] = int(self.HSVColor['S'] + self.color_wide_S) # 255
        self.HSVColor['Vu'] = int(self.HSVColor['V'] + self.color_wide_V) # 255



        if os.path.isfile('config.json'):
            # Load profiles from file...
            with open("config.json") as handle:
                self.param_dict = json.loads(handle.read())

            ### ROBOT PÂRAMETERS ###
            self.connection_type = self.param_dict["connection"]
            self.LIDAR = self.param_dict['LIDAR']
            self.VIDEO = self.param_dict['VIDEO']
            self.SCREEN_SCALE = self.param_dict["SCALE"]
            self.yShift = self.param_dict["yOFFSET"]
            self._scanRange = self.param_dict["scanAREA"]
            

        if os.path.isfile('calibration.json'):
            # Load profiles from file...
            with open("calibration.json") as handle:
                self.callibration = json.loads(handle.read())

            self.zOFFSET = self.callibration['z']
            self.pinOFFSET = self.callibration['pin']
            
            self.points[0] = self.callibration['points']['0']
            self.points[1] = self.callibration['points']['1']
            self.points[2] = self.callibration['points']['2']
            self.points[3] = self.callibration['points']['3']
            self.points[4] = self.callibration['points']['4']

            self.z_points[0] = self.callibration['z_points']['0']
            self.z_points[1] = self.callibration['z_points']['1']
            self.z_points[2] = self.callibration['z_points']['2']
            
            self.pixelX_size = self.callibration['pixelX_size']
            self.pixelY_size = self.callibration['pixelY_size']
            self.mH = self.callibration['mH']
            self.mV = self.callibration['mV']
            self.cam_angle = self.callibration['cam_angle']

            #print(self.points)
            
       
        ################# DETECT OS PLATFORM
        if platform.system() != 'Windows':
            self.WINDOWS = False

        ################# LOAD CSV FILE ##################
        if os.path.isfile("data.cfg"):
            fcfg = open("data.cfg", 'r')
            buff = fcfg.read()
            self.counter = int(buff)
        else:
            self.counter = 0
            
            

       

        
        #if self.ROBOT:
        #    self.myRobot.moveAllAxis(0,0,0,400,1000,t=1, sync=False)
        

        main.title('pyBot Robotic ARM')
        main.resizable(width=TRUE, height=TRUE)
        if self.WINDOWS:
            main.geometry('{}x{}'.format(1400, 800))
        else:
            main.geometry('{}x{}'.format(1400, 800))

        # create all of the main containers
        mycolor = '#001e38'
        self.top_frame = Frame(main, bg='white', width=700, height=70)#, padx=3, pady=3)
        self.top_frame2 = Frame(main, bg='white', width=324, height=70, pady=3)
        self.center = Frame(main, bg='black', width=700, height=512, padx=0, pady=2)
        self.center2 = Frame(main, bg='black', width=324, height=512, padx=0, pady=2)
        self.btm_frame = Frame(main, bg='gray', width=700, height=80, padx=3, pady=3)
        self.btm_frame2 = Frame(main, bg='gray', width=324, height=80, padx=3, pady=3)
        self.btm_frame3 = Frame(main, bg='black', width=700, height=30, padx=3, pady=3)
        self.btm_frame4 = Frame(main, bg='black', width=324, height=30, padx=3, pady=3)

        # layout all of the main containers
        main.grid_rowconfigure(1, weight=1)
        
        main.grid_columnconfigure(0, weight=1)
        main.grid_columnconfigure(1, weight=1)

        

        self.top_frame.grid(row=0, column=0, sticky="nsew")
        self.top_frame2.grid(row=0, column=1, sticky="nsew")
        self.center.grid(row=1, column=0, sticky="nsew")
        self.center2.grid(row=1, column=1, sticky="nsew")
        self.btm_frame.grid(row=2, column=0, sticky="nsew")
        self.btm_frame2.grid(row=2, column=1, sticky="nsew")
        self.btm_frame3.grid(row=3, column=0, sticky="nsew")
        self.btm_frame4.grid(row=3, column=1, sticky="nsew")

        #########################################################
        # create the widgets for the top frame
        
        
        self.canvas_logo = Canvas(self.top_frame, width=700, height=70, bg='white',highlightthickness=0, highlightbackground="white")
        self.canvas_logo.grid(row=0, column=0, sticky="nsew")

        timg = Image.open("./images/logo.jpg")
        timg = timg.resize((350,55), Image.ANTIALIAS)
        self.logo = ImageTk.PhotoImage(timg)

        self.canvas_logoImage = self.canvas_logo.create_image(0, 10, image=self.logo, anchor="nw")
        

        
        

        self.IMAGEN_lbl = "LEAP Motion status: Unknown"
        self.label0 = Label(self.top_frame2, text='pyBot Robotic Arm V1.' +self.VERSION, fg="black", bg="white", anchor="w", justify="left")
        self.label1 = Label(self.top_frame2, text='Laterality Configuration: RIGHT', bg="white", fg="black", anchor="w", justify="left")
        self.label2 = Label(self.top_frame2, text='Status: '+self.robotSTATUS, bg="white", fg="black", anchor="w", justify="left")
        
        # layout the widgets in the top frame
        self.label0.grid(row=0, column=1, sticky="w")
        self.label1.grid(row=2, column=1, sticky="w")
        self.label2.grid(row=1, column=1, sticky="w")
        

        self.update_labels()
        #########################################################
        # create the center widgets
        
        self.center.grid_rowconfigure(0, weight=1)
        self.center.grid_columnconfigure(0, weight=1)
        self.center.grid(row=1, column=0, sticky="nsew")

        
        
        self.canvas = Canvas(self.center, bg='black',width=self.center.winfo_width(), height=self.center.winfo_height(), highlightthickness=0, highlightbackground="black")
        self.canvas.grid_rowconfigure(0, weight=1)
        self.canvas.grid_columnconfigure(0, weight=1)
        self.canvas.grid(row=0, column=0, sticky="nsew")



        self.center2.grid_rowconfigure(0, weight=1)
        self.center2.grid_columnconfigure(1, weight=1)
        self.center2.grid(row=1, column=1, sticky="nsew")

        
        self.canvas2 = Canvas(self.center2,bg='black', width=324, height=512, highlightthickness=0, highlightbackground="black")
        self.canvas2.grid_rowconfigure(0, weight=1)
        self.canvas2.grid_columnconfigure(0, weight=1)
        self.canvas2.grid(row=0, column=1, sticky="nsew")
        
        
        self.path = './images/SCARA.jpg'
        
        
        if os.path.isfile(self.path):
            
            
            self.tmp_img = Image.open(self.path)
            self.tmp_img = self.tmp_img.resize((self.canvas2.winfo_width(),self.canvas2.winfo_height()), Image.ANTIALIAS)
            self.im_zoom = ImageTk.PhotoImage(self.tmp_img)
            self.img_error = False
        else:
            self.tmp_img = Image.open("./images/error.png")
            self.tmp_img = self.tmp_img.resize((self.width,self.height), Image.ANTIALIAS)
            self.im = ImageTk.PhotoImage(self.tmp_img)
            self.im_zoom = ImageTk.PhotoImage(self.tmp_img)
            self.img_error = True
        
        self._bp_side = Image.open("./images/LATERAL Sketch.png")
        self._bp_side_r = self._bp_side.resize((int(210*self.resizeFACTOR),int(140*self.resizeFACTOR)), Image.ANTIALIAS)
        self.bpSide = ImageTk.PhotoImage(self._bp_side_r)

        self._bp_plant = Image.open("./images/PLANTA Sketch.png")
        self._bp_plant_r = self._bp_plant.resize((int(125*self.resizeFACTOR),int(270*self.resizeFACTOR)), Image.ANTIALIAS)
        self.bpPlant = ImageTk.PhotoImage(self._bp_plant_r)
        
        self.base_im_l = Image.open('./images/BASE LATERAL.png')
        bw,bh = self.base_im_l.size
        self.base_im_r = self.base_im_l.resize((int(bw/3),int(bh/3)), Image.ANTIALIAS)
        self.base_im = ImageTk.PhotoImage(self.base_im_r)

        self.arm_im_l = Image.open('./images/BRAZO LATERAL.png')
        aw,ah = self.arm_im_l.size
        self.arm_im_r = self.arm_im_l.resize((aw/3,ah/3), Image.ANTIALIAS)
        self.arm_im = ImageTk.PhotoImage(self.arm_im_r)

        self.pinzaR_l = Image.open('./images/pinzaR.png')
        aw,ah = self.pinzaR_l.size
        self.pinzaR_r = self.pinzaR_l.resize((aw/3,ah/3), Image.ANTIALIAS)
        self.pinzaR = ImageTk.PhotoImage(self.pinzaR_r)

        self.pinzaR_Op_l = Image.open('./images/pinzaR_Op.png')
        aw,ah = self.pinzaR_Op_l.size
        self.pinzaR_Op_r = self.pinzaR_Op_l.resize((aw/3,ah/3), Image.ANTIALIAS)
        self.pinzaR_Op = ImageTk.PhotoImage(self.pinzaR_Op_r)
        
        self.armL_l = Image.open('./images/arm.png')
        aw,ah = self.armL_l.size
        self.armL_r = self.armL_l.resize((int(aw/float(1.82)),int(ah/float(1.82))), Image.ANTIALIAS)
        

        self.forearmL_l = Image.open('./images/Forearm.png')
        aw,ah = self.forearmL_l.size
        self.forearmL_r = self.forearmL_l.resize((int(aw/float(1.82)),int(ah/float(1.82))), Image.ANTIALIAS)
        

        self.baseL_l = Image.open('./images/base.png')
        aw,ah = self.baseL_l.size
        self.baseL_r = self.baseL_l.resize((int(aw/float(1.82)),int(ah/float(1.82))), Image.ANTIALIAS)

        self.pinzaL_l = Image.open('./images/pinzaL.png')
        aw,ah = self.pinzaL_l.size
        self.pinzaL_r = self.pinzaL_l.resize((int(aw/float(1.82)),int(ah/float(1.82))), Image.ANTIALIAS)

        self.canvasImage = self.canvas.create_image(0, 0, image=self.im, anchor="se")

        self.bboard_l = Image.open('./images/blackboard.jpg')

        ######## Load robot animation
        _scl = 1.5
        _sprite = Image.open('./images/Robot walking0.png')
        bw,bh = _sprite.size
        _sprite = _sprite.resize((int(bw/_scl),int(bh/_scl)), Image.ANTIALIAS)
        self.animation_rw[0] = ImageTk.PhotoImage(_sprite)

        _sprite = Image.open('./images/Robot walking1.png')
        bw,bh = _sprite.size
        _sprite = _sprite.resize((int(bw/_scl),int(bh/_scl)), Image.ANTIALIAS)
        self.animation_rw[1] = ImageTk.PhotoImage(_sprite)

        _sprite = Image.open('./images/Robot walking2.png')
        bw,bh = _sprite.size
        _sprite = _sprite.resize((int(bw/_scl),int(bh/_scl)), Image.ANTIALIAS)
        self.animation_rw[2] = ImageTk.PhotoImage(_sprite)

        _sprite = Image.open('./images/Robot walking3.png')
        bw,bh = _sprite.size
        _sprite = _sprite.resize((int(bw/_scl),int(bh/_scl)), Image.ANTIALIAS)
        self.animation_rw[3] = ImageTk.PhotoImage(_sprite)

        _sprite = Image.open('./images/Robot walking4.png')
        bw,bh = _sprite.size
        _sprite = _sprite.resize((int(bw/_scl),int(bh/_scl)), Image.ANTIALIAS)
        self.animation_rw[4] = ImageTk.PhotoImage(_sprite)

        _sprite = Image.open('./images/Robot walking5.png')
        bw,bh = _sprite.size
        _sprite = _sprite.resize((int(bw/_scl),int(bh/_scl)), Image.ANTIALIAS)
        self.animation_rw[5] = ImageTk.PhotoImage(_sprite)

        _gamepad = Image.open('./images/gamepad.png')
        bw,bh = _gamepad.size
        _gamepad = _gamepad.resize((50,40), Image.ANTIALIAS)
        self.gamepad_logo = ImageTk.PhotoImage(_gamepad)

        _gamepad = Image.open('./images/not_gamepad.png')
        bw,bh = _gamepad.size
        _gamepad = _gamepad.resize((50,40), Image.ANTIALIAS)
        self.not_gamepad_logo = ImageTk.PhotoImage(_gamepad)

        _leap = Image.open('./images/leap.png')
        bw,bh = _leap.size
        _leap = _leap.resize((50,40), Image.ANTIALIAS)
        self.leap_logo = ImageTk.PhotoImage(_leap)
        
        _leap = Image.open('./images/not_leap.png')
        bw,bh = _leap.size
        _leap = _leap.resize((50,40), Image.ANTIALIAS)
        self.not_leap_logo = ImageTk.PhotoImage(_leap)

        _blockly = Image.open('./images/blockly.png')
        bw,bh = _blockly.size
        _blockly = _blockly.resize((50,40), Image.ANTIALIAS)
        self.blockly_logo = ImageTk.PhotoImage(_blockly)

        _blockly = Image.open('./images/not_blockly.png')
        bw,bh = _blockly.size
        _blockly = _blockly.resize((50,40), Image.ANTIALIAS)
        self.not_blockly_logo = ImageTk.PhotoImage(_blockly)
        
        root.update_idletasks()
        self.width = self.canvas.winfo_width()
        self.height = self.canvas.winfo_height()
        
        #canvas.bind("<Key>", key)
        self.canvas.bind("<Button-1>", self.mi_go)
        self.canvas.bind("<Button-2>", self.b2_click)
        self.canvas.bind("<Motion>", self.mi_move)
        self.canvas.bind("<Leave>", self.mouse_leave)
        self.canvas.bind("<Enter>", self.mouse_enter)
        #self.canvas.bind("<MouseWheel>", self.mouse_wheel)
        self.canvas.bind("<B2-Motion>", self.b2_motion)
        self.canvas.bind("<B1-Motion>", self.edit_move)
        self.canvas.bind("<ButtonRelease-2>", self.b2_release)
        main.bind("<MouseWheel>", self.mouse_wheel)
        
        main.bind("<Key>", self.keypress)
        

       
        # Trajectorie detail inspector CANVAS
        self.canvasTI = Text(self.btm_frame2,width=42, height=6)
        
        self.canvasTI.pack(side=LEFT, fill=BOTH, expand = YES)
        
        scrollb = Tkinter.Scrollbar(self.btm_frame2, command=self.canvasTI.yview)
        
        scrollb.pack(side=RIGHT, fill=Y)
        self.canvasTI['yscrollcommand'] = scrollb.set

        ##################################### BUTTONS ##################################
        
        self.buttonRG=Button(self.btm_frame, text="Config	", height = 2, width = 10, command = self.robot_config)
        self.buttonRG.grid(row=0, column=4)
        ico = Image.open('./icons/configuration.png').resize((25, 25), Image.ANTIALIAS)
        ico = ImageTk.PhotoImage(ico)
        self.buttonRG.config(image=ico, compound=LEFT,width=90, height=30, padx=12, pady=4, anchor="nw")
        self.buttonRG.image = ico
        
        self.buttonUP=Button(self.btm_frame, text="Calibration	", height = 2, width = 10, command = self.robot_calibration)
        self.buttonUP.grid(row=1, column=4)
        ico = Image.open('./icons/calibration.png').resize((25, 25), Image.ANTIALIAS)
        ico = ImageTk.PhotoImage(ico)
        self.buttonUP.config(image=ico, compound=LEFT,width=90, height=30, padx=12, pady=4, anchor="nw")
        self.buttonUP.image = ico
        
        self.buttonDW=Button(self.btm_frame, text="Template	", height = 2, width = 10, command = self.open_template)
        self.buttonDW.grid(row=2, column=8)
        ico = Image.open('./icons/template.png').resize((25, 25), Image.ANTIALIAS)
        ico = ImageTk.PhotoImage(ico)

        
        self.buttonDW.config(image=ico, compound=LEFT,width=90, height=30, padx=12, pady=4, anchor="nw")
        self.buttonDW.image = ico
        

        self.label_b7 = Label(self.btm_frame, bg='gray', fg='white', text='', anchor="nw", justify="left")
        self.label_b7.grid(row=0, column=5, sticky="w")

        
        self.buttonCL=Button(self.btm_frame, text="Save	", height = 2, width = 10, command = self.save_file)
        self.buttonCL.grid(row=0, column=8)
        ico = Image.open('./icons/save.png').resize((25, 25), Image.ANTIALIAS)
        ico = ImageTk.PhotoImage(ico)
        self.buttonCL.config(image=ico, compound=LEFT,width=90, height=30, padx=12, pady=4, anchor="nw")
        self.buttonCL.image = ico

        self.buttonOP=Button(self.btm_frame, text="Load	", height = 2, width = 10, command = self.open_file)
        self.buttonOP.grid(row=1, column=8)
        ico = Image.open('./icons/load.png').resize((25, 25), Image.ANTIALIAS)
        ico = ImageTk.PhotoImage(ico)
        self.buttonOP.config(image=ico, compound=LEFT,width=90, height=30, padx=12, pady=4, anchor="nw") 
        self.buttonOP.image = ico

        self.buttonCLR=Button(self.btm_frame, text="Clear	", height = 2, width = 10, command = self.clear_trajectories)
        self.buttonCLR.grid(row=2, column=10)
        ico = Image.open('./icons/clear.png').resize((25, 25), Image.ANTIALIAS)
        ico = ImageTk.PhotoImage(ico)
        self.buttonCLR.config(image=ico, compound=LEFT,width=90, height=30, padx=12, pady=4, anchor="nw")
        self.buttonCLR.image = ico

       
        
        self.label_b0 = Label(self.btm_frame, bg='gray', fg='white', text='', anchor="nw", justify="left")
        self.label_b0.grid(row=0, column=9, sticky="w")
        
        self.buttonPLAY=Button(self.btm_frame, text="Start	", height = 2, width = 10, command = self.play_trajectories)
        self.buttonPLAY.grid(row=0, column=10)
        ico = Image.open('./icons/start.png').resize((25, 25), Image.ANTIALIAS)
        ico = ImageTk.PhotoImage(ico)
        self.buttonPLAY.config(image=ico, compound=LEFT,width=90, height=30, padx=12, pady=4, anchor="nw")
        self.buttonPLAY.image = ico

        buttonSTOP=Button(self.btm_frame, text="Stop	", height = 2, width = 10, command = self.robot_stop)
        buttonSTOP.grid(row=1, column=10)
        ico = Image.open('./icons/stop.png').resize((25, 25), Image.ANTIALIAS)
        ico = ImageTk.PhotoImage(ico)
        buttonSTOP.config(image=ico, compound=LEFT,width=90, height=30, padx=12, pady=4, anchor="nw")
        buttonSTOP.image = ico
        
        self.buttonZERO=Button(self.btm_frame, text="Zero	", height = 2, width = 10, command = self.go_zero)
        self.buttonZERO.grid(row=2, column=4)
        ico = Image.open('./icons/zero.png').resize((25, 25), Image.ANTIALIAS)
        ico = ImageTk.PhotoImage(ico)
        self.buttonZERO.config(image=ico, compound=LEFT,width=90, height=30, padx=12, pady=4, anchor="nw")
        self.buttonZERO.image = ico

        self.label_b20 = Label(self.btm_frame, bg='gray', fg='white', text='', anchor="nw", justify="left")
        self.label_b20.grid(row=0, column=11, sticky="w")
        
        self.buttonOBJECT=Button(self.btm_frame, text="Get Objects\t", state=DISABLED, height = 2, width = 10, command = self.get_allOBJ)
        self.buttonOBJECT.grid(row=0, column=12)
        ico = Image.open('./icons/zero.png').resize((25, 25), Image.ANTIALIAS)
        ico = ImageTk.PhotoImage(ico)
        self.buttonOBJECT.config(image=ico, compound=LEFT,width=100, height=30, padx=10, pady=4, anchor="nw")
        self.buttonOBJECT.image = ico

        self.buttonCFGVISION=Button(self.btm_frame, text="Vision Setup\t", state=DISABLED, height = 2, width = 10, command = self.vision_config)
        self.buttonCFGVISION.grid(row=1, column=12)
        ico = Image.open('./icons/zero.png').resize((25, 25), Image.ANTIALIAS)
        ico = ImageTk.PhotoImage(ico)
        self.buttonCFGVISION.config(image=ico, compound=LEFT,width=100, height=30, padx=10, pady=4, anchor="nw")
        self.buttonCFGVISION.image = ico

        self.buttonEDIT=Button(self.btm_frame, text="Edit Traject.\t", state=DISABLED, height = 2, width = 10, command = self.edit_tra)
        self.buttonEDIT.grid(row=2, column=12)
        ico = Image.open('./icons/zero.png').resize((25, 25), Image.ANTIALIAS)
        ico = ImageTk.PhotoImage(ico)
        self.buttonEDIT.config(image=ico, compound=LEFT,width=100, height=30, padx=10, pady=4, anchor="nw")
        self.buttonEDIT.image = ico

        ############ CHECK BUTTONS ##############
        
        
        self.buttonLEAP=Checkbutton(self.btm_frame, bg='gray', text=" Leap Motion", underline=1 ,height = 2, width = 16, variable = self.leapVar, command = self.chLeap, anchor = "w")
        self.buttonLEAP.grid(row=0, column=0)

        self.buttonRT=Checkbutton(self.btm_frame, bg='gray', text=" Real Time Motion", underline=1 ,height = 2, width = 16, variable = self.realVar, command = self.chReal, anchor = "w")
        self.buttonRT.grid(row=1, column=0)

        self.buttonLines=Checkbutton(self.btm_frame, bg='gray', text=" Traject. Motion", underline=1 ,height = 2, width = 16, variable = self.traVar, command = self.chTra, anchor = "w")
        self.buttonLines.grid(row=2, column=0)

        self.buttonROBOT=Checkbutton(self.btm_frame, bg='gray', text=" Activate Robot", height = 2, width = 16, variable = self.robotVar, command = self.chRobot, anchor = "w")
        self.buttonROBOT.grid(row=0, column=1)

        self.buttonABSANG=Checkbutton(self.btm_frame, bg='gray', text=" Absolute", underline=1 ,height = 2, width = 16, variable = self.robotAbs, command = self.chAbs, anchor = "w")
        self.buttonABSANG.grid(row=1, column=1)

        self.buttonVISION=Checkbutton(self.btm_frame, bg='gray', text=" A.Vision", underline=3 ,height = 2, width = 16, variable = self.robotVISION, command = self.chVision, anchor = "w")
        self.buttonVISION.grid(row=2, column=1)

        self.buttonLATERAL=Checkbutton(self.btm_frame, bg='gray', text=" Right/Left", underline=2 ,height = 2, width = 16, variable = self.robotLATERAL, command = self.chLATERAL, anchor = "w")
        self.buttonLATERAL.grid(row=0, column=2)

        self.buttonKINEMA=Checkbutton(self.btm_frame, bg='gray', text=" Kinematics", underline=1 ,height = 2, width = 16, variable = self.robotKINEMA, anchor = "w")
        self.buttonKINEMA.grid(row=1, column=2)

        self.buttonSCAN=Checkbutton(self.btm_frame, bg='gray', text=" Scan mode", underline=1 ,height = 2, width = 16, variable = self.scanMODEvar, command = self.scanMODE, anchor = "w")
        self.buttonSCAN.grid(row=2, column=2)

        self.label_b6 = Label(self.btm_frame4, bg='black', fg='white', text='(C) 2018 jjRobots ltd.', anchor="nw", justify="left")
        self.label_b6.pack(side=RIGHT)

        self.label_LOG = Label(self.btm_frame3, bg='black', fg='white', text='INFO:', anchor="nw", justify="left")
        self.label_LOG.pack(side=LEFT)

        self.canvas.bind("<Configure>", self.configure)
        self.mx = self.width/2

        self.canvas.config(cursor="none")

        self.load_image()
        self.draw_bot(self.control_mode)

        

        #INIT BLOCKLY...
        self.rbotBLOCKLY()

        
        
        
myPAD = None

root = Tk()

mw = MainWindow(root)

running = True



def on_closing():
    if tkMessageBox.askokcancel("SCARA CONTROL", "Do you want to quit?"):
        mw.BlocklyServer.shutdown()
        mw.tBlockly.close()
        mw.tBlockly.join()

        #rt_gamepad.stop()
        
        global running
        running = False
        time.sleep(1)

        if GAMEPAD:
            if pygame.joystick.get_init():
                pygame.joystick.quit()

            pygame.quit()
          
        root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing)

#################################
### LEAP Motion Initialization
#################################  
#class LP_Listener(Leap.Listener):
#    def on_device_change(self, controller):
        #print("Device change")
#        if controller.is_connected:
#            mw.leap_ready = True
#        else:
#            mw.leap_ready = False

        #print(mw.leap_ready)
        #mw.change_icons_status()

  
#if LEAP:
#    listener = LP_Listener()
#    _controller = Leap.Controller()
#    _controller.add_listener(listener)
    
#    time.sleep(1)
#    if _controller.is_connected:
#        mw.leap_ready = True
#else:
#    mw.leap_ready = False

#################################
### GAMEPAD Initialization
#################################
def gamepad_connection_handler():
    global myPAD
    
    pygame.quit()
    pygame.init()
    _pad_count = pygame.joystick.get_count()
    if _pad_count > 0:
        myPAD = pygame.joystick.Joystick(0)
        myPAD.init()
        mw.gamepad_ready = True
        if "XBOX" in myPAD.get_name():
            mw.XBOX_Pad = True
        else:
            mw.XBOX_Pad = False
            
        print("GAMEPAD detected ", myPAD.get_name())
    else:
        mw.gamepad_ready = False
        print("GAMEPAD NOT detected")

    if platform.system() == 'Windows':
        root.iconbitmap('./icons/scara_app.ico')
        #root.tk.call('wm', 'iconphoto', root._w, PhotoImage(file='./icons/scara_app.ico'))

    else:
        root.tk.call('wm', 'iconphoto', root._w, PhotoImage(file='./icons/Scara ICON3.png'))

    mw.pad_count = _pad_count


if GAMEPAD:
    z_gp_step = 0.0
    grip_gp_step = 0.0
    X_gp_step = 0.0
    Y_gp_step = 0.0
    Z_gp_step = 0.0
    '''
    #rt_gamepad = RepeatedTimer(2.0, gamepad_connection_handler)
    pad_count = 0
   
    pygame.init()
    pad_count = pygame.joystick.get_count()
    if pad_count > 0:
        myPAD = pygame.joystick.Joystick(0)
        myPAD.init()
        pad_name = myPAD.get_name()
        print("GAMEPAD detected: ", pad_name)
        mw.gamepad_ready = True
'''

   

if platform.system() == 'Windows':
    root.iconbitmap('./icons/scara_app.ico')
    #root.tk.call('wm', 'iconphoto', root._w, PhotoImage(file='./icons/scara_app.ico'))

else:
    root.tk.call('wm', 'iconphoto', root._w, PhotoImage(file='./icons/Scara ICON3.png'))


    #myPAD = inputs.devices.gamepads

#root.mainloop()
#########################################       
### CUSTOM MAIN LOOP
#########################################       
while running:
    
    if not mw.realVar.get() and not mw.traVar.get() and not mw.robotVISION.get() and not mw.leapVar.get() and not mw.scanMODEvar.get():
        mw.buttonLines.invoke()

    #########################################
    ### PROCESS GAMEPAD input if exist...
    #########################################
    if GAMEPAD:
        if mw.pad_count > 0 and mw.control_mode == 0:
            
            for event in pygame.event.get():
                if event.type == JOYAXISMOTION:
                    axes = myPAD.get_numaxes()

                    ### GAMEPAD LINEAL PROFILE ###
                    ### GRIP ANGLE
                    if abs(round(myPAD.get_axis(0),1)) >= 0.5:
                        grip_gp_step = round(myPAD.get_axis(0),2)
                    else:
                        grip_gp_step = 0.0
                        
                    ### Z AXIS
                    z_gp_step = 0.0
                    if (round(myPAD.get_axis(1),2)) > 0.10:
                        z_gp_step = round(myPAD.get_axis(1),2) - 0.10                    
                    if (round(myPAD.get_axis(1),2)) < -0.10:
                        z_gp_step = round(myPAD.get_axis(1),2) + 0.10
                        
                        
                    ### X,Y AXIS
                    X_gp_step = 0.0
                    if mw.XBOX_Pad and mw.WINDOWS:
                        if (round(myPAD.get_axis(4),2)) > 0.10:
                            X_gp_step = round(myPAD.get_axis(4),2) - 0.10
                        if (round(myPAD.get_axis(4),2)) < -0.10:
                            X_gp_step = round(myPAD.get_axis(4),2) + 0.10
                    else:
                        if (round(myPAD.get_axis(2),2)) > 0.10:
                            X_gp_step = round(myPAD.get_axis(2),2) - 0.10
                        if (round(myPAD.get_axis(2),2)) < -0.10:
                            X_gp_step = round(myPAD.get_axis(2),2) + 0.10
                            
                        
                    Y_gp_step = 0.0    
                    if (round(myPAD.get_axis(3),2)) > 0.10:
                        Y_gp_step = round(myPAD.get_axis(3),2) - 0.10
                    if (round(myPAD.get_axis(3),2)) < -0.10:
                        Y_gp_step = round(myPAD.get_axis(3),2) + 0.10
                
                    
                    ##############################

                if event.type == JOYBUTTONDOWN:
                    if myPAD.get_button(0) == 1:
                        if mw.pinOPEN:
                            mw.pinOPEN = False
                        else:
                            mw.pinOPEN = True

                    if myPAD.get_button(1) == 1:
                        mw.buttonLATERAL.invoke()
                        
                    if myPAD.get_button(3) == 1:
                        mw.buttonABSANG.invoke()
                        
                    #print(myPAD.get_button(0))
                    
            mw.pinANGLE += grip_gp_step
            if mw.pinANGLE > 65:
                mw.pinANGLE = 65
            if mw.pinANGLE < -65:
                mw.pinANGLE = -65
            

            mw.tZ -= z_gp_step
            if mw.tZ > mw.Zlimit:
                mw.tZ = mw.Zlimit
            if mw.tZ < 0.0:
                mw.tZ = 0.0

            mw.Z = mw.tZ

            mw.mX += X_gp_step
            mw.mY -= Y_gp_step

            mw.check_limits(mw.mX,mw.mY)
            mw.mX  = mw.X
            mw.mY = mw.Y
            
 
    #########################################       
    ### PROCESS Blockly commands
    #########################################       
    if mw.blocklyCMD:
        print(mw.blocklyDATA)
        mw.pinANGLE = mw.blocklyDATA["ANGLE"]
        mw.pinOPEN = mw.blocklyDATA["OPEN"]
        mw.tZ = mw.blocklyDATA["Z"]
        mw.exec_trajectorie(mw.botX,mw.botY,mw.botZ,mw.blocklyDATA["X"],mw.blocklyDATA["Y"],mw.blocklyDATA["Z"])
        # Blockly CMD finished
        mw.blocklyCMD = False
   
    #########################################       
    ### DRAWING
    #########################################              
    mw.draw_bot()
    root.update_idletasks()
    root.update()
    
