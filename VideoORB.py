from VideoDisplay import VideoDisplay
import numpy as np

np.warnings.filterwarnings('ignore')
from Helpers import Helpers
import time
from Tello import tello

"""RANDON MATRIX PARAMETERS FOR TESTING"""
width = 1920
height = 1080
focal = 300

matrix = [[focal, 0, width // 2],
          [0, focal, height // 2],
          [0, 0, 1]]

inverseMatrix = np.linalg.inv(matrix)

"""-------------------------------------"""

def droneTesting():
    print("Drone Testing")
    drone = tello()
    drone.startConnection()
    drone.streamOn()
    time.sleep(2)
    drone.startProcesses()


def slamTesting():
    processing = VideoDisplay().videoCapture()


if __name__ == "__main__":
    droneTesting()
