import socket
import cv2
import numpy as np
import threading
import re

class tello(object):
    # A class containing the controller for the DJI Tello

    def __init__(self):
        #Capture Data, helped with code and my mental sanity checks
        self.capture = None
        self.captureStatus = False

        #Drone flight variables all variables found in DJI Tello SDK, all incoming messages from the state port
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.xSpeed = 0.0
        self.ySpeed = 0.0
        self.zSpeed = 0.0
        self.lTemp = 0.0
        self.hTemp = 0.0
        self.dTOF = 0.0
        self.height = 0.0
        self.battery = 0.0
        self.barometer = 0.0
        self.uptime = 0.0
        self.xAcceleration = 0.0
        self.yAcceleration = 0.0
        self.zAcceleration = 0.0

        #Variables to connect to drone
        self.host_Address = '0.0.0.0'
        self.tello_Address = '192.168.10.1'
        self.command_Port = 8889
        self.state_Port = 8890
        self.video_Port = 11111
        self.connected = False;
        self.drone_speed = 100

        #sockets for communicating to the drone
        self.command_Socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_Socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.video_Socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        #Binding sockets correctly according to Tello SDK 2
        try:
            self.command_Socket.bind(('', 8889))
        except socket.error:
            print("Error binding command Socket")

        try:
            self.state_Socket.bind(('',8890))
        except socket.error:
            print("Error binding state Socket")


        #new threads for listener processes
        self.stateListener = threading.Thread(target = self.startStateListener, args = ())
        self.videoListener = threading.Thread(target = self.startVideoFeed, args = ())

        #make them daemons to run in the background
        self.stateListener.daemon = True
        self.videoListener.daemon = True


    def startProcesses(self):
        self.stateListener.start()
        self.videoListener.start()

    def getAddress(self):
        print(self.tello_Address)

    def sendCommand(self, msg):
        #Sends the command to the drone in the correct encoding used a as wrapper from other arguements
        msg = msg.encode(encoding = "utf-8")
        send = self.command_Socket.sendto(msg,(self.tello_Address,self.command_Port))
        #Used to obtain feed back from the drone to check if hte message got sent correctly, will reply with OK
        try:
            data,server = self.command_Socket.recvfrom(1518)
            print(data.decode(encoding = "utf-8"))
        except Exception:
            print("Error with connection to drone")
            
    def startStateListener(self, print = 0):
        while self.connected:
            #Recieve data from drone with a 1024 buffer
            state = self.state_Socket.recv(1024)
            decoded = state.decode(encoding = "utf-8")
            #regex search to find all the data in the output string
            output = re.findall(r"[-+]?\d*\.\d+|\d+", decoded)
            #Assigning variables into for drone
            self.pitch = float(output[0])
            self.roll = float(output[1])
            self.yaw = float(output[2])
            self.xSpeed = float(output[3])
            self.ySpeed = float(output[4])
            self.zSpeed = float(output[5])
            self.lTemp = float(output[6])
            self.hTemp = float(output[7])
            self.dTOF = float(output[8])
            self.height = float(output[9])
            self.battery = float(output[10])
            self.barometer = float(output[11])
            self.uptime = float(output[12])
            self.xAcceleration = float(output[13])
            self.yAcceleration = float(output[14])
            self.zAcceleration = float(output[15])

            if(print == 1):
                self.printState()
            
            


    def startVideoFeed(self):
        width = 1280
        height = 720

        self.capture = cv2.VideoCapture('udp://@0.0.0.0:11111')

        self.captureStatus = self.capture.isOpened()


        while self.connected & (self.captureStatus==True) :
            recieved, frame = self.capture.read()

            resized = cv2.resize(frame,(width,height))

            bwVideo = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

            cv2.imshow("Video Feed", bwVideo)
            if(cv2.waitKey(1) & 0xFF == ord('q')):
                self.land()
                break



    def startConnection(self):
        print("Connected to SDK")
        self.sendCommand("command")
        self.connected = True

    def closeConnection(self):
        self.connected = False
        self.command_Socket.close()
        self.state_Socket.close()
        self.video_Socket.close()
        
    def setSpeed(self, speed):
        self.drone_speed = speed
        combine = ("speed " + str(speed))
        print("Setting flight speed to: "+ combine)
        self.sendCommand(combine)
    
    def getSpeed(self):
        print("Current speed is: " + str(self.drone_speed))
# automatic commands for drone
    def takeOff(self):
        self.sendCommand("takeoff")

    def land(self):
        self.sendCommand("land")
#3d Flight controls for drone
    def flyForward(self):
        self.sendCommand("forward 20")
    
    def flyBack(self):
        self.sendCommand("back 20")

    def flyLeft(self):
        self.sendCommand("left 20")

    def flyRight(self):
        self.sendCommand("right 20")

    def flyUp(self):
        self.sendCommand("up 20")

    def flyDown(self):
        self.sendCommand("down 20")

#basic adjustment for drone
#increase complexity to allow for custom angles
    def turn45Right(self):
        self.sendCommand("cw 45")

    def turn90Right(self):
        self.sendCommand("cw 90")

    def turn45Left(self):
        self.sendCommand("ccw 45")

    def turn90Left(self):
        self.sendCommand("ccw 90")

#turn on video feed from the drone
    def streamOn(self):
        print('open streaming')
        self.sendCommand("streamon")

    def streamOff(self):
        self.sendCommand("streamoff")

#turn of motors incase of emergency
    def emergency(self):
        self.sendCommand("emergency")



    def printState(self):
        print("\n pitch" + str(self.pitch)+
            "\n roll" + str(self.roll) +
            "\n yaw" + str(self.yaw) +
            "\n xSpeed" + str(self.xSpeed) +
            "\n ySpeed" + str(self.ySpeed) +
            "\n zSpeed" + str(self.zSpeed) +
            "\n lTemp" + str(self.lTemp) +
            "\n hTep" + str(self.hTemp) +
            "\n dTOF" + str(self.dTOF) +
            "\n height" + str(self.height) +
            "\n battery" + str(self.battery) +
            "\n barometer" + str(self.barometer) +
            "\n uptime" + str(self.uptime) +
            "\n xAcceleration" + str(self.xAcceleration) +
            "\n yAcceleration" + str(self.yAcceleration) +
            "\n zAcceleration" + str(self.zAcceleration), end = '\r')