import cv2
import numpy as np
import glob
from matplotlib import pyplot as plt


"""
ALL COLOUR IN THE CODE IS IN THE BGR FORMAT AS IT WAS IMPLEMENTED BY OPENCV IN SUCH A WAY TO SUPPORT BETTER CAMERA/VIDEO INPUTS

Work on Camera Matrix for calibration, displaying matches on screen
"""

def featureFinder(frame, orb):

    #Manually finds features
    features = cv2.goodFeaturesToTrack(frame, 2000, qualityLevel = 0.01, minDistance = 7)

    #Finds the location of keypoint in the frame
    keypoints = [cv2.KeyPoint(x=feature[0][0], y=feature[0][1], _size=20) for feature in features]
    keypoints, descriptors = orb.compute(frame, keypoints)
    
    return keypoints, descriptors


def featureMatcher(f1D, f2D):

    bfMatcher = cv2.BFMatcher()

    matches = bfMatcher.knnMatch(f1D, f2D, k=2)

    goodMatches = []

    #FROM OPENCV BRUTE FORCE MATCHER
    for m,n in matches:
        if m.distance < 0.75*n.distance:
            goodMatches.append([m])

    print("Good Features found: ", len(goodMatches))

    return goodMatches


def cameraMatrix():
    print("Good luck with this")

def manualTracking():

    #Storage containers for feature detection
    f1Descriptor = None
    f2Descriptor = None
    f1Keypoints = None
    f2Keypoints = None

    oldFrame = None

    # Resize Parameters
    scale = 0.5

    #Sample Video feed
    video = cv2.VideoCapture(r"C:\Users\barte\source\repos\Bart3Kn\TelloScan\Video Sample\London Bus Ride.mp4")

    #Redundacy check for testing purposes
    if (video.isOpened()== False):  
      print("Error opening video  file") 


    orb = cv2.ORB.create()

    while(video.isOpened()): 

        ret, frame = video.read()

        if(ret):

            #Resized and convert to gray
            width = int(frame.shape[1] * scale)
            height = int(frame.shape[0] * scale)
            dim = (width, height)
            resized = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
            gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

            if(f1Keypoints == None):
                f1Keypoints, f1Descriptor = featureFinder(gray, orb)
                print("Frame 1 done")
                oldFrame = gray

            else:
                f2Keypoints, f2Descriptor = featureFinder(gray, orb)
                print("Next frame done")
                

                matches = featureMatcher(f1Descriptor, f2Descriptor)

                #for m in matches[:5]: 
                    #print("Distance: ", m[0].distance, "  QueryIdx: ", m[0].queryIdx, "  TrainIdx: ", m[0].trainIdx)

                #Red is old frame
                cv2.drawKeypoints(gray, f1Keypoints, resized, color = (0,0,255))
                #Blue is new frame
                cv2.drawKeypoints(resized, f2Keypoints, resized, color = (255,0,0))
                #Display on image
                cv2.imshow("ORB_1", resized)

                output = cv2.drawMatchesKnn(oldFrame,f1Keypoints, gray, f2Keypoints, matches[:20], None, flags = 2)
                cv2.imshow("ORB_2", output)


                f1Keypoints = f2Keypoints
                f1Descriptor = f2Descriptor
                oldFrame = gray

                if cv2.waitKey(25) & 0xFF == ord('q'): 
                        break

        else:
            print("End of video file")
            break


    video.release() 
    cv2.destroyAllWindows()

if __name__ == "__main__":
    #main()
    manualTracking()