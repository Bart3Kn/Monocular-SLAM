import sys
sys.path.append('build/Pangolin/build/src/')
import pypangolin  as pango
"""
Huge tanks to Marko Elez for fixing 
https://github.com/markoelez
"""
import numpy as np
from colour import Color
import OpenGL.GL as gl


class PointCloud(object):
    def __init__(self):
        self.pointCoords = []
        self.cameraCoords = []



    def createViewer(self, framesArray):
        window = pango.CreateWindowAndBind("3d Map", 640, 480)
        gl.glEnable(gl.GL_DEPTH_TEST)
        pm = pango.ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000000)
        mv = pango.ModelViewLookAt(-2, -2, -2, 0, 0, 0, pango.AxisY)
        s_cam = pango.OpenGlRenderState(pm, mv)

        handler = pango.Handler3D(s_cam)
        d_cam = (pango.CreateDisplay().SetBounds(pango.Attach(0),
                                                 pango.Attach(1),
                                                 pango.Attach(0),
                                                 pango.Attach(1)
                                                 ).SetHandler(handler))

        while not pango.ShouldQuit():
            gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
            d_cam.Activate(s_cam)

            poseArray = []
            pointsArray = []

            for frame in framesArray:
                pose = np.array(frame.pose[:3,3])
                poseArray.append([pose[2], pose[1], pose[0]])
                points = frame.points3D
                if points is not None:
                    pointsArray.extend(points)

            gl.glPointSize(5)
            blue = Color("blue")
            colors = list(blue.range_to(Color("red"), len(framesArray)))
            rgb = []
            for color in colors:
                rgb.append(color.rgb)
            #BLUE TO RED, BLUE IS START, RED IS END
            pango.DrawPoints(poseArray, rgb)

            #Draw keypoints
            gl.glPointSize(1)
            gl.glColor3f(1,1,1)
            pango.DrawPoints(pointsArray)

            pango.FinishFrame()
