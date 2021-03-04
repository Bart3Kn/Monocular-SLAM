import numpy as np

class Helpers(object):
    def catone(x):
        """
        Converts any [[X,Y]...] array into a [[X,Y,1]...] array
        Works on any size
        This is done to make homogeneous coordinates
        """
        rows = x.shape[0]
        onesARR = np.ones(shape=[rows, 1])
        output = np.concatenate((x, onesARR), axis=1)
        return output