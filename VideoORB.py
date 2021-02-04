import cv2
import numpy as np
from matplotlib import pyplot as plt


"""
ALL COLOUR IN THE CODE IS IN THE BGR FORMAT AS IT WAS IMPLEMENTED BY OPENCV IN SUCH A WAY TO SUPPORT BETTER CAMERA VIDEO
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
    for m,n in matches:
        if m.distance < 0.75*n.distance:
            goodMatches.append([m])

    print("Good Features found: ", len(goodMatches))
    return goodMatches

def manualTracking():


    f1Descriptor = None
    f2Descriptor = None
    f1Keypoints = None
    f2Keypoints = None


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

            else:
                f2Keypoints, f2Descriptor = featureFinder(gray, orb)
                print("Next frame done")
                

                matches = featureMatcher(f1Descriptor, f2Descriptor)

                #Red is old frame
                cv2.drawKeypoints(gray, f1Keypoints, resized, color = (0,0,255))
                #Blue is new frame
                cv2.drawKeypoints(resized, f2Keypoints, resized, color = (255,0,0))



                cv2.imshow("ORB", resized)
            
                f1Keypoints = f2Keypoints
                f1Descriptor = f2Descriptor
            


                if cv2.waitKey(25) & 0xFF == ord('q'): 
                        break
        else:
            print("No frame recieved,  breaking code")
            break
    video.release() 
    cv2.destroyAllWindows()


if __name__ == "__main__":
    #main()
    manualTracking()