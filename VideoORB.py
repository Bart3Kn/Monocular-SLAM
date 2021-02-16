from VideoDisplay import VideoDisplay
import numpy as np
import Helpers
import time
from Tello import tello



def main():
    capture = VideoDisplay()
    capture.videoCapture()

def normalize(keypoints):
    focal = 1
    width = 1920
    height = 1080

    intrinsicMatrix = np.array(([focal, 0, width // 2],
                                [0, focal, height // 2],
                                [0, 0, 1]))

    matrixInverse = np.linalg.inv(intrinsicMatrix)
    return np.dot(matrixInverse, Helpers.Helpers.catone(keypoints).T).T[:, 0:2]


def denormalize(keypoints):

    focal = 1
    width = 1920
    height = 1080


    intrinsicMatrix = np.array(([focal, 0, width // 2],
                                [0, focal, height // 2],
                                [0, 0, 1]))

    ret = np.dot(intrinsicMatrix, np.array([keypoints[0], keypoints[1], 1.0]))
    return ret

def testing():

    point = [[1, 2],
             [3, 4],
             [5, 6]]

    point = np.array(point)


    output = normalize(point)
    print(output)

    output2 = denormalize(output)
    print(output2)

def droneTesting():

    print("Drone Testing")
    drone = tello()

    drone.startConnection()
    drone.streamOn()
    time.sleep(2)
    drone.startProcesses()

def slamTestsing():
    processing = VideoDisplay().videoCapture()



if __name__ == "__main__":
    slamTestsing()