# -*- coding: cp1252 -*-
# JJROBOTS PYBOT SCARA PYTHON API CLASS
#
# Robot could be connected via wifi (adhoc wifi: default password for Wifi network JJROBOTS_XX is 87654321) or via USB
# Before running the script you need to connect the robot to the PC (USB or wifi connection)

# author: JJROBOTS 2018-2019
# API version: 1.15 (17/09/2019)
# Licence: Open Source (GNU LGPLv3)

import math
import socket
import time
import struct
import platform
# We make the socketserver compatible between python 2 and 3
if platform.python_version()[0] == '2':
    import SocketServer
    socketserver = SocketServer
else:
    import socketserver
import threading
import serial
import sys

# Robot definition
ROBOT_ARM1_LENGTH=91.61
ROBOT_ARM2_LENGTH=105.92

NODATA = -20000

#debug=[]

# UDP SERVER CLASS
class MyUDPRequestHandler(socketserver.DatagramRequestHandler):
    # Override the handle() method
    def handle(self):
        #datagram = self.rfile.readline().strip()
        pass

class UDPMessageServer(socketserver.UDPServer):
    message = ""
    def __init__(self, server_address, handler_class=MyUDPRequestHandler):
        try:
            self.server = socketserver.UDPServer.__init__(self,server_address,handler_class)
        except Exception as e:
            print(e)

    # Return the request on a server object property
    def process_request(self,request,client_address):
        #print(request)
        self.message = request[0]
        #debug.append(self.message.rstrip())

    def clean(self):
        self.message=""
        #print("UDP clean")

    def get_message(self):
        return self.message

    #def stop(self):
    #    try:
    #        self.server.shutdown()
    #    except:
    #        print("Fail shutting down UDP server...")

# SERIAL SERVER CLASS
class SerialServer():
    message = ""
    def __init__(self,s):
        self.s = s
        self.message=""
        self.port_ok=True
    def run(self):
        line = SerialReadLine(self.s)
        while self.port_ok:
            try:
                if (self.s.in_waiting>0):
                    aux_message = line.readline()
                    # Avoid debug messages and pass only status messages ($$...)
                    if (aux_message.startswith("$$")):
                        self.message = aux_message
                        #debug.append(self.message.rstrip())
                    #if (self.message.startswith('-')):
                    #    sys.stdout.write(self.message) # DEBUG
                else:
                    time.sleep(0.005)
            except:
                self.port_ok = False
    def clean(self):
        self.message=""

    def get_message(self):
        return self.message

    def stop(self):
        self.port_ok = False

class SerialReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)


# CLASS to control JJROBOTS SCARA ARM ROBOT
class PyBotArm(object):
  LEN1 = ROBOT_ARM1_LENGTH
  LEN2 = ROBOT_ARM2_LENGTH
  IP = "192.168.4.1" # Default ROBOT IP (with BROBOT JJROBOTS_XX wifi)
  PORT = 2222        # Default ROBOT port
  IN_PORT = 2223     # Default ROBOT input port
  COM_BAUD = 115200
  mode = 0           # MODE:0 WIFI (UDP connection), MODE:1 USB
  connected = False
  ready = False      # Robot connected and ready
  working = False    # Robot is busy (working)
  working_counter=0
  working_timestamp=0
  NODATA = NODATA
  telemetry = ""     # Last valid telemetry value
  msg = ""
  distance = 0
  target_A1 = 0
  target_A2 = 0
  target_Z = 0

  def __init__(self):
    self.mode=0
    self.connected=False
    self.ready=False
    self.telemetry = ""
    self.t = None # Server thread
    self.ser = None
    
  # Connect to ROBOT (modes : WIFI,USB) in USB mode user should define serial port (as string)
  def connect(self,user_mode="WIFI",COM_PORT="COM3",verbose=True):
    self.mode=0
    self.connected=False
    self.ready=False
    self.working=False
    self.telemetry = ""
    # WIFI MODE
    if (user_mode=="WIFI"):
      if verbose:
          print("WIFI mode")
      self.mode=0
      # Create default socket with UDP protocol
      self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
      # Create UDP server on a different thread
      self.UDPServerObject = UDPMessageServer(("",self.IN_PORT), MyUDPRequestHandler)
      try:
        # Create a thread to handle UDP messages (UDP server)
        self.t = threading.Thread(target=self.UDPServerObject.serve_forever)
        self.t.setDaemon(True) # don't hang on exit
        self.t.start()
        if verbose:
            print ("Started UDP server on port "+str(self.IN_PORT))
        # Send a wellcome message (HELLO) to robot
        self.sock.sendto(b'JJAH0000000000000000',(self.IP,self.PORT))
        time.sleep(1)
        self.connected = True
      except Exception as e:
        print(e)
        self.UDPServerObject.shutdown()
        self.UDPServerObject.message = ""
        print ('shutting down the UDP server!')
        time.sleep(1)
        connected=False
        ready=False
        if verbose:
            print("NOT READY")
        return False

    # USB Mode
    if (user_mode=="USB"):
      if verbose:
          print("USB mode "+COM_PORT)
      self.mode = 1
      try:
        # Create a thread to handle Serial incoming messages
        #print(COM_PORT,self.COM_BAUD)
        self.ser = serial.Serial(COM_PORT, self.COM_BAUD, timeout=1)
        time.sleep(1)
        if self.ser.is_open:
            #self.ser.timeout = 0.2 # Serial Timeout
            if verbose:
                print ("Created Serial connection on "+str(COM_PORT))
            self.SerialServer = SerialServer(self.ser)
            self.t = threading.Thread(target=self.SerialServer.run)
            self.t.setDaemon(True) # don't hang on exit
            self.t.start()
            # Send a wellcome message (HELLO) to robot
            self.ser.write(b'JJAH0000000000000000')
            time.sleep(0.2)
            self.connected = True
      except Exception as e:
        print(e)
        if hasattr(self.ser,"is_open"):
            if self.ser.is_open:
                self.ser.close()
        if hasattr(self,"SerialServer"):
            self.SerialServer.message = ""
            self.SerialServer.close()
        print ('shutting down serial server')
        time.sleep(1)
        connected=False
        ready=False
        if verbose:
            print("NOT READY")
        return False
    if verbose:
        print("Checking robot status...")
    for i in range(10):  # Read robot status (several times until we see a valid message from robot)
        status = self.getStatus()
        if self.ready:
            break
        time.sleep(0.1)

    if not self.ready:
        if (user_mode=="WIFI"):
            self.UDPServerObject.shutdown()
            self.UDPServerObject.message = ""
            self.connected=False
        else:
            # Close serial connection?
            self.connected=False

    #print("init",self.ready)
    if verbose:
        if self.ready:
            print("READY")
        else:
            print("NOT READY")
    return self.ready

  def close(self,verbose=True):
    if self.mode==0:
      if hasattr(self,"UDPServerObject"):
        self.UDPServerObject.shutdown()
      if verbose:
          print ('Close: shutting down the UDP server!')
      time.sleep(0.5)
      connected = False
      ready = False
    if self.mode==1:
      if hasattr(self.ser,"is_open"):
        if self.ser.is_open:
          self.ser.close()
      if hasattr(self,"SerialServer"):
        self.SerialServer.close()
      if verbose:
          print ('Close: shutting down the serial server!')
      time.sleep(0.5)
      connected = False
      ready = False

  # Is robot connected and working?
  def isReady(self):
    return self.ready

  # Send a command to the robot
  def sendCommand(self,Header,p1,p2,p3,p4,p5,p6,p7,p8,t=0,sync=True):
    if not self.ready:
      self.getStatus()
      if not self.isReady():
        print ("SC: Robot not ready!!")
        return
    #print (Header,p1,p2,p3,p4,p5,p6,p7,p8) # DEBUG
    base = bytearray(Header)  # message
    param1 = bytearray(struct.pack(">h",p1))
    param2 = bytearray(struct.pack(">h",p2))
    param3 = bytearray(struct.pack(">h",p3))
    param4 = bytearray(struct.pack(">h",p4))
    param5 = bytearray(struct.pack(">h",p5))
    param6 = bytearray(struct.pack(">h",p6))
    param7 = bytearray(struct.pack(">h",p7))
    param8 = bytearray(struct.pack(">h",p8))
    message = base+param1+param2+param3+param4+param5+param6+param7+param8
    if self.mode==0:
      # Send UPD message
      self.sock.sendto(message,(self.IP,self.PORT))
    else:
      # Send SERIAL message
      self.ser.write(message)
    #Update working status variables
    self.working = True  # We supose that the robot is actually working on the new command
    self.working_counter = 0
    self.working_timestamp=0
    if (Header == b'JJAM') or (Header == b'JJAT'):
      if p1 == NODATA:
        self.target_A1 = NODATA
      else:
        self.target_A1 = p1/100.0  # parameter in 0.01 degrees
      if p2 == NODATA:
        self.target_A2 = NODATA
      else:
        self.target_A2 = p2/100.0
      if p3 == NODATA:
        self.target_Z = NODATA
      else:
        self.target_Z = p3/100.0   # parameter in 0.01 mm
    # Sync mode (default): wait until robot execute command...
    if sync:
      #debug.append("SYNC "+Header+" "+str(p8)+" "+str(self.target_A1)+","+str(self.target_A2)+","+str(self.target_Z))
      si=0
      #print("S",Header,p1,p2,si)
      #time.sleep(0.1)
      while (self.isWorking()):
        #time.sleep(0.05)
        si+=1
      #print("S",Header,p1,p2,si)

      #debug.append("SYNCEND "+Header)
    # delay time?
    if t>0:
      time.sleep(t)

  def getStatus(self):
    if self.mode == 0 and self.connected:
      self.msg = self.UDPServerObject.get_message()
    elif self.mode == 1 and self.connected:
      self.msg = self.SerialServer.get_message()
    else:
      self.msg = ""
    if (self.msg.startswith('$$')):
      self.telemetry = self.msg  # This property store the last valid telemetry value
      self.ready = True
    return self.telemetry

  def getTimestamp(self):
    try:
        self.timestamp = getStatus().split(",")[8]
    except:
        return self.timestamp

  def getDistance(self,debug=False):
    if self.mode == 0 and self.connected:
      self.msg = self.UDPServerObject.get_message()
    elif self.mode == 1 and self.connected:
      self.msg = self.SerialServer.get_message()
    else:
      self.msg = ""
    if debug: print("get distance",self.msg,self.mode)
    if (self.msg.startswith('$$')):
      if debug: print("update")
      self.distance = int(self.msg.split(",")[6])
      self.ready = True
    return self.distance

  # The robot is working until it reachs the desired position. Timeout exception?
  def isWorking(self):
    if self.mode == 0 and self.connected:
      self.msg = self.UDPServerObject.get_message()
    elif self.mode == 1 and self.connected:
      self.msg = self.SerialServer.get_message()
    if (self.msg.startswith('$$')):
      try:
        if self.msg.startswith('$$0'):
          timestamp = int(self.msg.split(",")[7])
          if (timestamp != self.working_timestamp):
            self.working_counter += 1
            self.working_timestamp = timestamp # Update last timestamp
            #debug.append("IW NEWTS:"+str(self.working_counter)+" "+str(self.working_timestamp)+" "+self.msg)
          # If we have received at least 20 messages of $$0 (no working) even if we have no reach the target angles we accept that the robot is non working
          if (self.working_counter>20):
            self.working = False
            print("API IW:C END",self.target_Z,self.msg)
            return self.working
          if (self.target_A1 == NODATA):
              A1 = self.target_A1
          else:
              A1 = int(self.msg.split(",")[1])/10.0
          if (self.target_A2 == NODATA):
              A2 = self.target_A2
          else:
              A2 = int(self.msg.split(",")[2])/10.0
          if (self.target_Z == NODATA):
              Z = self.target_Z
          else:
              Z = int(self.msg.split(",")[3])/10.0
          if (abs(self.target_A1-A1)<=0.2) and (abs(self.target_A2-A2)<=0.2) and (abs(self.target_Z-Z)<=0.2):
            #debug.append("IW REACH")
            self.working = False
          else:
            self.working = True
        else:
          self.working = True
      except:
        print("ERROR isWorking",self.msg)
    return self.working

  # Check if robot is running OK...
  def checkRobot(self):
    self.ready=False
    self.telemetry=""
    for i in range(10):  # Try 10 times to get info from robot to check status
        self.getStatus()
        if self.ready:
            break
        time.sleep(0.05)
    return self.ready

  # Direct kinematics: Move manual mode (message with 8 channels)
  # ch1, ch2 in degrees, ch3=z (in mm), ch4(orientation) from 0 to 1000, ch5(gripper) from 0 to 1000
  def moveAllAxis(self,ch1=NODATA,ch2=NODATA,ch3=NODATA,ch4=NODATA,ch5=NODATA,ch6=NODATA,ch7=NODATA,ch8=NODATA,t=0,sync=True):
    if ch1!=NODATA: ch1*=100 # Rescale to 1/100 degrees
    if ch2!=NODATA: ch2*=100 # Rescale to 1/100 degrees
    if ch3!=NODATA: ch3*=100 # Rescale to 1/100 mm
    #print "SCARA Manual mode",ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8
    self.sendCommand(b'JJAM',ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8,t,sync)

  def moveAxis1(self,ch1=0,t=0,sync=True):
    #print "SCARA Axis1:",ch1
    self.sendCommand(b'JJAM',int(ch1*100),NODATA,NODATA,NODATA,NODATA,NODATA,NODATA,NODATA,t,sync)

  def moveAxis2(self,ch2=0,t=0,sync=True):
    #print "SCARA Axis2:",ch2
    self.sendCommand(b'JJAM',NODATA,int(ch2*100),NODATA,NODATA,NODATA,NODATA,NODATA,NODATA,t,sync)

  def moveAxis3(self,ch3=0,t=0,sync=True):
    #print "SCARA Axis3:",ch3
    self.sendCommand(b'JJAM',NODATA,NODATA,int(ch3*100),NODATA,NODATA,NODATA,NODATA,NODATA,t,sync)

  def moveAxis4(self,ch4=500,t=0,sync=True):
    #print ("SCARA Axis4:",ch4)
    self.sendCommand(b'JJAM',NODATA,NODATA,NODATA,int(ch4),NODATA,NODATA,NODATA,NODATA,t,sync)

  def moveAxis5(self,ch5=500,t=0,sync=True):
    #print "SCARA Axis5:",ch5
    self.sendCommand(b'JJAM',NODATA,NODATA,NODATA,NODATA,int(ch5),NODATA,NODATA,NODATA,t,sync)

  # Inverse Kinematics (move robot to X,Y,Z point in mm)
  def moveXYZ(self,x=0,y=150,z=10,ch4=NODATA,ch5=NODATA,elbow=0,t=0,sync=True,verbose=False):
    #print "SCARA Move XYZ:",x,y,z
    A1,A2 = self.IK(x,y,elbow)
    if (verbose):
      print ("> XYZ:",x,y,z," IK:",A1,A2)
    self.moveAllAxis(A1,A2,z,ch4,ch5,NODATA,NODATA,NODATA,t,sync)

  # Inverse Kinematics (move robot to X,Y point in mm)
  def moveXY(self,x=0,y=150,ch4=NODATA,ch5=NODATA,elbow=0,t=0,sync=True):
    #print "SCARA Move XY:",x,y
    A1,A2 = self.IK(x,y,elbow)
    self.moveAllAxis(A1,A2,NODATA,ch4,ch5,NODATA,NODATA,NODATA,t,sync)

  # Inverse Kinematics (move robot to Z point in mm)
  def moveZ(self,z=10,t=0,sync=True): # z in mm
    #print "SCARA Z:",z
    self.sendCommand(b'JJAM',NODATA,NODATA,int(z*100),NODATA,NODATA,NODATA,NODATA,NODATA,t,sync)

  # Define points of a trajectory (until the robot receives the last_point=1 and then executes the trajectory) [max 50 points]
  def trajectoryXYZ(self,x=0,y=150,z=10,ch4=NODATA,ch5=NODATA,elbow=0,num_point=0,last_point=0,t=0,sync=True,verbose=False):
    A1,A2 = self.IK(x,y,elbow)
    if last_point==0: # Intermediate points don\B4t have sync mode
        sync=False
    if (verbose):
      print (">Traj XYZ:",x,y,z," IK:",A1,A2)
    self.sendCommand(b'JJAT',int(A1*100),int(A2*100),int(z*100),ch4,ch5,elbow,num_point,last_point,t,sync)

  # Set robot Speed and acceleration
  def setSpeedAcc(self,xy_speed=100,z_speed=100,xy_acc=100,z_acc=100,traj_speed=50):
    self.sendCommand(b'JJAS',int(xy_speed),int(z_speed),int(xy_acc),int(z_acc),int(traj_speed),NODATA,NODATA,NODATA,t=0,sync=False)

  # Emergency STOP
  def emergencyStop(self):
    print("Emergency Stop!")
    self.sendCommand(b'JJAE',NODATA,NODATA,NODATA,NODATA,NODATA,NODATA,NODATA,NODATA,t=0,sync=False)

  def motorsCalibration(self):
    print("Robot motors calibration...")
    self.sendCommand(b'JJAC',NODATA,NODATA,NODATA,NODATA,NODATA,NODATA,NODATA,NODATA,t=0,sync=False)

  # Resolve the Inverse Kinematics of the robot (x,y) => Angle1, Angle2
  #  x,y in milimetes (mm). Return angle A1, A2 in degrees.
  #  there are two posible solutions (depends on elbow configuration)
  def IK(self,x=0,y=0,elbow=0,verbose=False):
    len1 = self.LEN1
    len2 = self.LEN2
    if (elbow==1):  # inverse elbow solution: reverse X axis, and final angles.
      x = -x;
    dist = math.sqrt(x*x+y*y);  # distance
    if (dist > (len1+len2)):
      dist = (len1+len2)-0.001;
      if (verbose):
        print("IK overflow->limit");
    D1 = math.atan2(y,x);
    # Law of cosines (a,b,c) = acos((a*a+b*b-c*c)/(2*a*b))
    D2 = math.acos((dist*dist + len1*len1 - len2*len2) / (2.0 * dist*len1))
    A1 = math.degrees(D1+D2)-90 # Our robot configuration
    A2 = math.acos((len1*len1 + len2*len2 - dist*dist) / (2.0 * len1*len2))
    A2 = math.degrees(A2) - 180    # Our robot configuration
    if (elbow==1):
      A1 = -A1
      A2 = -A2
    if (verbose):
      print("IK xy:",x,y,"elbow:",elbow," A1A2:",A1,A2)
    return A1,A2

  # Robot Direct kinematic (from axis angles to cartesian x,y)
  def DK(self,A1=0,A2=0,verbose=False):
    A1 = A1 + 90 # Our robot configuration
    x1 = self.LEN1*math.cos(math.radians(A1))
    y1 = self.LEN1*math.sin(math.radians(A1))
    x = x1 + (self.LEN2 * math.cos(math.radians(A1+A2)))
    y = y1 + (self.LEN2 * math.sin(math.radians(A1+A2)))
    if (verbose):
      print("DK A1A2:",A1,A2," xy:",x,y)
    return x,y

  # For DEBUG only...
  #def printDebug(self):
  #  for i in range(len(debug)):
  #      print(debug[i])
