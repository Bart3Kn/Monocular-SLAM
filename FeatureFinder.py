import cv2
import numpy as np
import Helpers
from skimage.measure import ransac
from skimage.transform import FundamentalMatrixTransform, EssentialMatrixTransform

class FeatureFinder(object):
    def __init__(self):
        self.ORB = cv2.ORB_create()
        self.bfMatcher = cv2.BFMatcher(cv2.NORM_HAMMING)


        self.intrinsicMatrix = None
        self.matrixInverse = None


    def makeMatrix(self, width, height, focal):
        """
        Parameters
        ----------
        width = frame width
        height = frame height
        focal = focal length of the camera

        Returns
        -------
        generates the camera matrix and the inverse of the camera matrix for the code

        """
        self.intrinsicMatrix = np.array(([focal,0,width//2],
                                        [0,focal,height//2],
                                        [0,0,1]))
        self.matrixInverse = np.linalg.inv(self.intrinsicMatrix)

    def normalize(self, keypoints):

        normD = Helpers.catone(keypoints)
        keypoints = np.dot(self.intrinsicMatrix, normD).T[:, 0:2]
        return keypoints

    def denormalize(self, keypoints):
        keypoints = np.dot(self.intrinsicMatrix, keypoints).T[:, 0:2]
        return keypoints

    def featureFinder(self, frame):
        #Finds the features
        features = cv2.goodFeaturesToTrack(frame, 2000, qualityLevel= 0.01, minDistance= 7)

        #extracts keypoints
        keypoints = [cv2.KeyPoint(x=feature[0][0], y=feature[0][1], _size=20) for feature in features]
        keypoints, descriptors = self.ORB.compute(frame, keypoints)

        return keypoints, descriptors



    def featureMatcher(self, frame1Keypoints, frame1Descriptors, frame2Keypoints, frame2Descriptors):
        matches = self.bfMatcher.knnMatch(frame1Descriptors, frame2Descriptors, k=2)

        goodMatches = []


        #Lowe's Ratio finder
        for m,n in matches:
            if m.distance < 0.5*n.distance:
                keypoint1 = frame1Keypoints[m.queryIdx].pt
                keypoint2 = frame2Keypoints[m.trainIdx].pt

                goodMatches.append((keypoint1, keypoint2))

        return  goodMatches

    def  cameraMatrix(self, matches):

        """
        model, inliers = ransac((ret[:,0], ret[:,1]),
                                FundamentalMatrixTransform,
                                min_samples= 8,
                                residual_threshold= 1,
                                max_trials=100)
        """
        return None


    def matchTextExtractor(self, matches):
        for m in matches:
            print("Distance: ", m[0].distance, "  QueryIdx: ", m[0].queryIdx, "  TrainIdx: ", m[0].trainIdx)

