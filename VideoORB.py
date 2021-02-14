from FrameExtractor import FrameExtractor
import cv2 as cv

def main():
    capture = FrameExtractor()
    capture.videoCapture()

if __name__ == "__main__":
        main()