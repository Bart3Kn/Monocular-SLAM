import cv2

class FeatureFinder(object):
    def __init__(self):
        self.ORB = cv2.ORB_create()
        self.bfMatcher = cv2.BFMatcher()


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

        for m,n in matches:
            if m.distance < 0.75*n.distance:
                keypoint1 = frame1Keypoints[m.queryIdx].pt
                keypoint2 = frame2Keypoints[m.trainIdx].pt

                goodMatches.append((keypoint1, keypoint2))

        return  goodMatches

    def  cameraMatrix(self):
        print("In Progress")


    def matchTextExtractor(self, matches):
        for m in matches:
            print("Distance: ", m[0].distance, "  QueryIdx: ", m[0].queryIdx, "  TrainIdx: ", m[0].trainIdx)

