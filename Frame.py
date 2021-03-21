import numpy as np

class FrameObject(object):
    """
    This is a class used to contain the frames and their data so that they can be used at a later time
    """
    def __init__(self, pose, keypoints, descriptors, queryIDx, trainIDx, matches=None, points3D=None):


        self.pose = pose
        self.keypoints = keypoints
        self.descriptors = descriptors
        self.matches = matches
        self.queryIdx = queryIDx
        self.trainIdx = trainIDx
        self.points3D = points3D