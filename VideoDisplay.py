import cv2
import time
import sys
import threading
from FeatureFinder import FeatureFinder
from Frame import FrameObject
from PointCloud import PointCloud
import numpy as np

# Stops scientific display of values
np.set_printoptions(suppress=True)


class VideoDisplay(object):
    def __init__(self, source=None):
        # Frame Analysis
        self.oldFrame = None
        self.newFrame = None

        # Frame Processing
        self.scale = 0.5
        self.oldFrameKeypoints = None
        self.oldFrameDescriptors = None
        self.newFrameKeypoints = None
        self.newFrameDescriptors = None

        # FPS Counters
        self.oldFrameTime = 0
        self.newFrameTime = 0
        self.fps = 0
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        # VideoSource
        self.videoSource = source
        self.framesArray = []

        self.viewer = PointCloud()


    def resize2Gray(self, frame):

        width = int(frame.shape[1] * self.scale)
        height = int(frame.shape[0] * self.scale)
        dim = (width, height)
        resized = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

        return resized, gray

    def videoCapture(self):
        if self.videoSource is None:
            print("Video")
            video = cv2.VideoCapture(r"Video Sample/Forest Drive.mp4")
        else:
            print("Drone")
            video = self.videoSource

        if not video.isOpened():
            print("Error opening file")

        ff = FeatureFinder()

        """
            VERY IMPORTANT TO GET THIS CORRECT
            FOCAL POINT CHANGES BASED ON INPUT FEED
            CAN BE CALCULATED USING FUNDAMENTAL MATRIX
            """

        # Make Matrix
        width = video.get(3)
        height = video.get(4)
        # for LondonBusRide
        # focalPoint = 205

        focalPoint = 350
        ff.makeMatrix(width, height, focalPoint)


        goodpointsArray = []

        while video.isOpened():
            ret, frame = video.read()

            if ret:

                resized, gray = self.resize2Gray(frame)

                # FPS Calculator
                self.newFrameTime = time.time()
                self.fps = 1 / (self.newFrameTime - self.oldFrameTime)
                self.oldFrameTime = self.newFrameTime
                self.fps = str(int(self.fps))

                """
                    CALCULATED INITIAL FRAME SO THAT IT CAN BE USED IN FEATURE MATCHING
                    """
                if self.oldFrameKeypoints is None:
                    self.oldFrameKeypoints, self.oldFrameDescriptors = ff.featureFinder(gray)
                    pose1 = np.eye(4)
                    frame1 = FrameObject(pose1, self.oldFrameKeypoints, self.oldFrameDescriptors, None, None, None)
                    self.oldFrame = frame1
                    self.framesArray.append(frame1)
                    print("Frame 1 done")
                    print(self.framesArray[0])

                if self.newFrameKeypoints is None:
                    self.newFrameKeypoints, self.newFrameDescriptors = ff.featureFinder(gray)
                    matches, queryIdx, trainIdx = ff.featureMatcher(self.oldFrameKeypoints, self.oldFrameDescriptors,
                                                                    self.newFrameKeypoints, self.newFrameDescriptors)

                    inliers, translation, queryIdx, trainIdx = ff.ransacFit(matches, queryIdx, trainIdx)

                    nextPose = ff.translationAdjustment(self.framesArray[-1].pose, translation)
                    nextFrame = FrameObject(nextPose, self.newFrameKeypoints, self.newFrameDescriptors, queryIdx,
                                            trainIdx, inliers)

                    self.framesArray.append(nextFrame)

                else:

                    self.newFrameKeypoints, self.newFrameDescriptors = ff.featureFinder(gray)
                    matches, queryIdx, trainIdx = ff.featureMatcher(self.oldFrameKeypoints, self.oldFrameDescriptors,
                                                                    self.newFrameKeypoints, self.newFrameDescriptors)

                    """RANSAC TO REMOVE OUTLIERS"""
                    inliers, translation, queryIdx, trainIdx = ff.ransacFit(matches, queryIdx, trainIdx)

                    nextPose = ff.translationAdjustment(self.framesArray[-1].pose, translation)

                    nextFrame = FrameObject(nextPose, self.newFrameKeypoints, self.newFrameDescriptors, queryIdx,
                                            trainIdx, inliers)
                    self.framesArray.append(nextFrame)

                    pts4d = cv2.triangulatePoints(self.oldFrame.pose[:3], nextFrame.pose[:3], inliers[:, 0].T, inliers[:, 1].T)
                    pts4d /= pts4d[3]
                    good_pts4d = (np.abs(pts4d[:, 3]) > 0.005) & (pts4d[:, 2] > 0)

                    # Keypoint 1 is  new Keypoint, keypoint 2 is old
                    for keypoint1, keypoint2 in inliers:
                        x1, y1 = ff.denormalise(keypoint1)
                        x2, y2 = ff.denormalise(keypoint2)

                        cv2.circle(resized, (x1, y1), color=(0, 0, 255), radius=2)
                        # Line of translation between kp1 and kp2
                        cv2.line(resized, (x1, y1), (x2, y2), color=(255, 255, 0))


                    cv2.putText(resized, self.fps, (7, 70), self.font, 1, (100, 255, 0), 3, cv2.LINE_AA)
                    cv2.imshow("ORB SLAM", resized)

                    if cv2.waitKey(25) & 0xFF == ord('q'):
                        print("\n You have closed the window: ")
                        sys.exit(1)

                self.oldFrameKeypoints = self.newFrameKeypoints
                self.oldFrameDescriptors = self.newFrameDescriptors
                self.oldFrame = nextFrame

            else:
                self.viewer.createViewer(self.framesArray)
                print("End of video file")

        video.release()
        cv2.destroyAllWindows()
