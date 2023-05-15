import scipy.io as scio
import numpy as np

camera_parameters = {}
camera_parameters['IntrinsicMatrix'] = np.array([
    [788.5933, 0, 0],
    [-1.5284, 790.1151, 0],
    [267.2851, 257.6332, 1]])
camera_parameters['RadialDistortion'] = np.array([0.2647, -1.0395, 2.6651])
camera_parameters['TangentialDistortion'] = np.array([0.0031, 0.0104])

