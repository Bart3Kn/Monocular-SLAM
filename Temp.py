import sys
sys.path.append('build/Pangolin/build/src/')
import pypangolin  as pango
"""
HUGE THANKS TO MARKO ELEZ FOR FIXING PANGOLIN WITH FFMPEG 4.0
https://github.com/markoelez
"""
import numpy as np
import OpenGL.GL as gl




def main():
    window = pango.CreateWindowAndBind("3d Map", 640, 480)
    gl.glEnable(gl.GL_DEPTH_TEST)
    pm = pango.ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000)
    mv = pango.ModelViewLookAt(-0, 0.5, -3, 0, 0, 0, pango.AxisY)
    s_cam = pango.OpenGlRenderState(pm, mv)


    handler = pango.Handler3D(s_cam)
    d_cam = (
        pango.CreateDisplay().SetBounds(pango.Attach(0),
                                        pango.Attach(1),
                                        pango.Attach(0),
                                        pango.Attach(1)
        ).SetHandler(handler)
    )

    while not pango.ShouldQuit():
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        d_cam.Activate(s_cam)

        points = np.random.random((10000, 3))
        colors = np.zeros((len(points), 3))
        colors[:, 1] = 1 - points[:, 0]
        colors[:, 2] = 1 - points[:, 1]
        colors[:, 0] = 1 - points[:, 2]
        points = points * 3 + 1
        gl.glPointSize(3)

        pango.DrawPoints(points, colors)
        pango.FinishFrame()

if __name__ == "__main__":
    main()