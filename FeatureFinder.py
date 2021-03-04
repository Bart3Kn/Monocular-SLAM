import cv2
import numpy as np
from Helpers import Helpers
from skimage.measure import ransac
from skimage.transform import FundamentalMatrixTransform, EssentialMatrixTransform

class FeatureFinder(object):
    def __init__(self):
        self.ORB = cv2.ORB_create()
        self.bfMatcher = cv2.BFMatcher(cv2.NORM_HAMMING)


        self.matrix = None
        self.inverseMatrix = None
        self. width = 0
        self.height = 0
        self.focal = 0

        self.focalEstimate = []

    def makeMatrix(self, width, height, focal):
        """
        Parameters:
        width = frame width
        height = frame height
        focal = focal point of the camera in pixels

        generates the camera matrix and the inverse of the camera matrix
        for the code used for nomralisation and denorm
        """
        self.width = width
        self.height = height
        self.focal = focal

        self.matrix = np.array(([focal,0,width//2],
                                [0,focal,height//2],
                                [0,0,1]))
        self.inverseMatrix = np.linalg.inv(self.matrix)

    def normalise(self, points):
        """Normalises using inverse camera matrix and then returns the data points """
        append1 = Helpers.catone(points)
        output = np.dot(self.inverseMatrix, append1.T)
        output = output.T[:, 0:2]
        return output

    def denormalise(self, points):
        """Denomarlise points for use in projection on display"""
        append1 = (points[0],points[1],1.0)
        output = np.dot(self.matrix, append1)
        point1 = int(round(output[0]))
        point2 = int(round(output[1]))
        return point1, point2

    def adjustKeypoints(self, x1, x2, y1, y2):
        """
        Keypoints are normalised to original frame rather than the resized one,
        they have to be moved from centre to top left
        """
        x1 = int(x1 - (self.width // 2))
        x2 = int(x2 - (self.width // 2))
        y1 = int(y1 - (self.height // 2))
        y2 = int(y2 - (self.height // 2))
        return x1, x2, y1, y2

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

        return goodMatches

    def ransacFit(self, matches):
        matches = np.array(matches)

        matches[:,0,:] = self.normalise(matches[:,0,:])
        matches[:,1,:] = self.normalise(matches[:,1,:])


        model, inlier = ransac((matches[:,0], matches[:,1]),
                               EssentialMatrixTransform,
                               #FundamentalMatrixTransform,
                               min_samples= 8,
                               residual_threshold=0.001,
                               max_trials=250)

        matches = matches[inlier]

        """        
        s,v,d = np.linalg.svd(model.params)
        f_est = np.sqrt(2)/((v[0]+v[1])/2)
        self.focalEstimate.append(f_est)
        print(np.median(self.focalEstimate))
        """
        transformation = self.extractTransformation(model.params)
        #transformation = None

        return matches, transformation

    def extractTransformation(self, model):
        W = np.mat([[0.0,-1.0,0.0],
                    [1.0,0.0,0.0],
                    [0.0,0.0,1.0]])
        U,d,Vt = np.linalg.svd(model)
        if(np.linalg.det(U)>0):
            if(np.linalg.det(Vt)<0):
                Vt = Vt * (-1.0)
        R = np.dot(np.dot(U,W),Vt)
        if(np.sum(R.diagonal()) < 0):
            R = np.dot(np.dot(U, W.T), Vt)
        t = U[:, 2]
        Rt = np.concatenate([R, t.reshape(3,1)], axis = 1)
        return Rt


    def matchTextExtractor(self, matches):
        """
        QueryIdx = match found in second frame, contains x,y coords
        TrainIdx = match from the first frame, contains x,y coords
        Distance = length between qIDX and tIdx using pythag to calculate change in x and y
        """
        for m in matches:
            print("Distance: ", m[0].distance, "  QueryIdx: ", m[0].queryIdx, "  TrainIdx: ", m[0].trainIdx)

