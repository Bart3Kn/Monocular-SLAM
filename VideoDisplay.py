import cv2
import time
from FeatureFinder import FeatureFinder
import sys

class VideoDisplay(object):
        def __init__(self, source = None):
            #Frame Analysis
            self.oldFrame = None
            self.newFrame = None

            #Frame Processing
            self.scale = 0.5
            self.oldFrameKeypoints = None
            self.oldFrameDescriptors = None
            self.newFrameKeypoints = None
            self.newFrameDescriptors = None

            #FPS Counters
            self.oldFrameTime = 0
            self.newFrameTime = 0
            self.fps = 0
            self.font = cv2.FONT_HERSHEY_SIMPLEX

            #VideoSource
            self.videoSource = source

            print("Frame extractor initialised")

        def resize2Gray(self, frame):

            width = int(frame.shape[1] * self.scale)
            height = int(frame.shape[0] * self.scale)
            dim = (width, height)
            resized = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
            gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

            return resized, gray

        def videoCapture(self):

            if self.videoSource == None:
                print("Video")
                video = cv2.VideoCapture(r"Video Sample/London Bus Ride.mp4")
            else:
                print("Drone")
                video = self.videoSource

            ff = FeatureFinder()

            #Make Matrix
            width = video.get(3)
            height = video.get(4)
            focalPoint = 1
            ff.makeMatrix(width,height, focalPoint)

            if video.isOpened() == False:
                print("Error opening file")

            while(video.isOpened()):
                ret, frame = video.read()

                if (ret):

                    resized, gray = self.resize2Gray(frame)

                    #FPS Calculator
                    self.newFrameTime = time.time()
                    self.fps = 1/(self.newFrameTime - self.oldFrameTime)
                    self.oldFrameTime = self.newFrameTime
                    self.fps = str(int(self.fps))


                    if(self.oldFrameKeypoints == None):
                        self.oldFrameKeypoints, self.oldFrameDescriptors = ff.featureFinder(gray)
                        print("Frame 1 done")

                    else:
                        self.newFrameKeypoints, self.newFrameDescriptors = ff.featureFinder(gray)
                        matches = ff.featureMatcher(self.oldFrameKeypoints, self.oldFrameDescriptors, self.newFrameKeypoints, self.newFrameDescriptors)
                        print("Next frame done, Matches found: ", len(matches))


                        #Keypoint 1 is  new Keypoint, keypoint 2 is old
                        for keypoint1, keypoint2 in matches:
                            x1, y1 = map(lambda x: int(round(x)), keypoint1)
                            x2, y2 = map(lambda x: int(round(x)), keypoint2)
                            cv2.circle(resized, (x1,y1), color=(0,0,255), radius=2)
                            #Line of translation between kp1 and kp2
                            cv2.line(resized,(x1,y1),(x2,y2), color=(255,255,0))

                        cv2.putText(resized, self.fps, (7, 70), self.font, 1, (100, 255, 0), 3, cv2.LINE_AA)
                        cv2.imshow("ORB", resized)

                        self.oldFrameKeypoints = self.newFrameKeypoints
                        self.oldFrameDescriptors = self.newFrameDescriptors
                        self.oldFrame = gray


                        if cv2.waitKey(25) & 0xFF == ord('q'):
                            sys.exit(1)

                else:
                    print("End of video file")
                    break

            video.release()
            cv2.destroyAllWindows()