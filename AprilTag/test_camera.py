import cv2
import depthai as dai
pipeline = dai.Pipeline()
camRgb = pipeline.create(dai.node.ColorCamera)
xoutRgb = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("rgb")
camRgb.setPreviewSize(300, 300)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

camRgb.preview.link(xoutRgb.input)

with dai.Device(pipeline) as device:
    print('Connected cameras:', device.getConnectedCameraFeatures())
    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

    while True:
        inRgb = qRgb.get()
        cv2.imshow("rgb", inRgb.getCvFrame())
        if cv2.waitKey(1) == ord('q'):
            break
