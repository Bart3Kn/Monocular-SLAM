import sys
sys.path.append('build/Pangolin/build/src/')
import pypangolin  as pango
"""
HUGE THANKS TO MARKO ELEZ FOR FIXING PANGOLIN WITH FFMPEG 4.0
https://github.com/markoelez

"""
import numpy as np
from colour import Color
import OpenGL.GL as gl


class PointCloud(object):
    def __init__(self):
        self.pointCoords = []
        self.cameraCoords = []

    def updatePoints(self, points):
        self.pointCoords.append(points)

    def updateCamera(self, cameraLoc):
        self.cameraCoords.append(cameraLoc)

    def createViewer(self, framesArray):
        window = pango.CreateWindowAndBind("3d Map", 640, 480)
        gl.glEnable(gl.GL_DEPTH_TEST)
        pm = pango.ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000000)
        mv = pango.ModelViewLookAt(-2, -2, -2, 0, 0, 0, pango.AxisZ)
        s_cam = pango.OpenGlRenderState(pm, mv)

        handler = pango.Handler3D(s_cam)
        d_cam = (pango.CreateDisplay().SetBounds(pango.Attach(0),
                                                 pango.Attach(1),
                                                 pango.Attach(0),
                                                 pango.Attach(1)
                                                 ).SetHandler(handler)
        )

        while not pango.ShouldQuit():
            gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
            d_cam.Activate(s_cam)
            gl.glPointSize(5)

            blue = Color("blue")
            colors = list(blue.range_to(Color("red"),len(framesArray)))
            rgb=[]
            for color in colors:
                rgb.append(color.rgb)


            poseArray =[]

            for frame in framesArray:
                pose = np.array(frame.pose[:3,3])
                poseArray.append(pose)

            pango.DrawPoints(poseArray, rgb)
            #pango.DrawPoints(framesArray,rgb)


            pango.FinishFrame()


"""if __name__ == "__main__":
    array = [[27.94813938,  51.06715327, -44.58486212], 
             [26.97708054,  51.2329502,  -44.756781], 
             [26.05509088,  51.55704844, -44.96866636], 
             [25.08672418,  51.7507552,  -45.12596743]] 

    viewer = PointCloud()
    viewer.createViewer(array)"""
