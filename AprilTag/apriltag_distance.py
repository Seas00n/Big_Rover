import numpy as np
import apriltag
import depthai as dai
import cv2
from scipy.spatial.transform import Rotation as R

camera_parameters = {}
camera_parameters['IntrinsicMatrix'] = np.array([
    [788.5933, 0, 0],
    [-1.5284, 790.1151, 0],
    [267.2851, 257.6332, 1]])
camera_parameters['RadialDistortion'] = np.array([0.2647, -1.0395, 2.6651])
camera_parameters['TangentialDistortion'] = np.array([0.0031, 0.0104])
np.save('img/calibrate/parameters.npy', camera_parameters)
K = camera_parameters['IntrinsicMatrix']


def create_pipeline():
    pipeline = dai.Pipeline()
    camRgb = pipeline.create(dai.node.ColorCamera)
    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutRgb.setStreamName("rgb")
    camRgb.setPreviewSize(400, 400)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
    camRgb.preview.link(xoutRgb.input)
    return pipeline


def create_detector():
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    return detector


def draw_tag(r, image, eularangle):
    (ptA, ptB, ptC, ptD) = r.corners
    ptB = (int(ptB[0]), int(ptB[1]))
    ptC = (int(ptC[0]), int(ptC[1]))
    ptD = (int(ptD[0]), int(ptD[1]))
    ptA = (int(ptA[0]), int(ptA[1]))
    # draw the bounding box of the AprilTag detection
    cv2.line(image, ptA, ptB, (0, 255, 0), 2)
    cv2.line(image, ptB, ptC, (0, 255, 0), 2)
    cv2.line(image, ptC, ptD, (0, 255, 0), 2)
    cv2.line(image, ptD, ptA, (0, 255, 0), 2)
    # draw the center (x, y)-coordinates of the AprilTag
    (cX, cY) = (int(r.center[0]), int(r.center[1]))
    cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
    # draw the tag family on the image
    tagFamily = r.tag_family.decode("utf-8")
    cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    ARROW_LENGTH = 120
    dirangle = (eularangle[2] - 5) * np.pi / 180 * 1.8
    deltax = np.sin(dirangle) * ARROW_LENGTH
    deltay = ARROW_LENGTH / 2 * np.cos(dirangle)
    newcneter = r.center + np.array([deltax, deltay])
    cv2.circle(image, tuple(newcneter.astype(int)), 8, (255, 0, 0), 5)
    cv2.line(image, tuple(newcneter.astype(int)),
             tuple(r.center.astype(int)),
             (255, 0, 0), 2)


pipeline = create_pipeline()
detector = create_detector()
qRgb: dai.DataOutputQueue
inRgb: dai.ImgFrame
tag: apriltag.Detection
with dai.Device(pipeline) as device:
    print('Connected cameras:', device.getConnectedCameraFeatures())
    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

    while True:
        inRgb = qRgb.get()
        image = inRgb.getCvFrame()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        results = detector.detect(gray)
        for tag in results:
            homo = tag.homography
            num, Rs, Ts, Ns = cv2.decomposeHomographyMat(homo, K)
            r = R.from_matrix(Rs.T)
            eularangle = r.as_euler('xyz').T * 180 / np.pi
