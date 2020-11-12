from Tello import tello
import sys
import cv2
import socket
import threading


drone = tello()

drone.startConnection()
drone.streamOn()
print('begining processes')
drone.startProcesses()


