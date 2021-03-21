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
    output = output.T[:, 0:2]
    return output


def poseTesting():
    pose1 = np.array([[0.99999848, - 0.00013367, 0.001737, -0.72392802],
             [0.00013463, 0.99999984, -0.00055517, -0.16171898],
             [-0.00173693, 0.0005554, 0.99999834, 0.67065281]])

    pose2 = np.array([[0.99999929, -0.00042971, 0.00111203, -0.79004067],
             [0.00042917, 0.99999979, 0.00048403, 0.49975325],
             [-0.00111224, -0.00048355, 0.99999926, 0.35508087]])
    print("Pose1: \n",pose1)
    print("\n Pose2: \n",pose2)

    pose1Coords = np.eye(4)
    pose1Coords[:3,:4] = pose1
    print("\n Pose1Coords: \n", pose1Coords)

    pose2Coords = np.eye(4)
    pose2Coords[:3,:4] = pose2
    print("\n Pose2Coords: \n", pose2Coords)
    pose2Coords[:3,3] = np.add(pose1Coords[:3,3], pose2Coords[:3,3])
    print("\n Pose2Coords: \n", pose2Coords)

    #outputPose[:3,3] = np.add(outputPose[:3,3],pose2[:3,3])
    #print(outputPose)


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
