import cv2
import time
from FeatureFinder import FeatureFinder

class VideoDisplay(object):
        def __init__(self):
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

            print("Frame extractor initialised")

        def videoCapture(self):

            video = cv2.VideoCapture(r"Video Sample/London Bus Ride.mp4")
            ff = FeatureFinder()

            if video.isOpened():
                print("Error opening file")

            while(video.isOpened()):
                ret, frame = video.read()

                if (ret):
                    width = int(frame.shape[1] * self.scale)
                    height = int(frame.shape[0] * self.scale)
                    dim = (width, height)
                    resized = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
                    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

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

                        for keypoint1, keypoint2 in matches:
                            x1, y1 = map(lambda x: int(round(x)), keypoint1)
                            x2, y2 = map(lambda x: int(round(x)), keypoint2)
                            cv2.circle(resized, (x1,y1), color=(0,0,255), radius=2)
                            cv2.line(resized,(x1,y1),(x2,y2), color=(255,255,0))

                        # Red is old frame Blue is new frame
                        # Keypoint comparison
                        #cv2.drawKeypoints(resized, self.oldFrameKeypoints, resized, color=(0, 0, 255))
                        #cv2.drawKeypoints(resized, self.newFrameKeypoints, resized, color=(255, 0, 0))

                        cv2.putText(resized, self.fps, (7, 70), self.font, 1, (100, 255, 0), 3, cv2.LINE_AA)
                        cv2.imshow("ORB", resized)

                        self.oldFrameKeypoints = self.newFrameKeypoints
                        self.oldFrameDescriptors = self.newFrameDescriptors
                        self.oldFrame = gray

                        if cv2.waitKey(25) & 0xFF == ord('q'):
                            break

                else:
                    print("End of video file")
                    break

            video.release()
            cv2.destroyAllWindows()
