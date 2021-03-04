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


matrix = [[focal, 0, width//2],
          [0,focal, height//2],
          [0,0,1]]

inverseMatrix = np.linalg.inv(matrix)

"""-------------------------------------"""
def testing():
    point = [[1, 2],
             [3, 4],
             [5, 6]]

    point = np.array(point)
    print("Point")
    print(point)

    print("normalised")
    norm = normalise(point)
    print(norm)

    print("denorm")
    denorm = denormalise(norm)
    print(denorm)


def normalise(points):
    """Normalises using inverse camera matrix and then returns the data points """
    append1 = Helpers.catone(points)
    output = np.dot(inverseMatrix, append1.T)
    output = output.T[:, 0:2]
    return output

def denormalise(points):
    """Denomarlise points for use in porjection on display"""
    append1 = Helpers.catone(points)
    output = np.dot(matrix, append1.T)
    output = output.T[:,0:2]
    return output

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
    slamTesting()