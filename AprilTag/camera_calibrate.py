import cv2
import depthai as dai

pipeline = dai.Pipeline()
camRgb = pipeline.create(dai.node.ColorCamera)
xoutRgb = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("rgb")
camRgb.setPreviewSize(500, 500)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

camRgb.preview.link(xoutRgb.input)

img_calibrate_path = '/home/yuxuan/Project/Big_Rover/AprilTag/april_tag/src/calibration/src/'
num_of_cal = 50
num_img = 0
with dai.Device(pipeline) as device:
    print('Connected cameras:', device.getConnectedCameraFeatures())
    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

    while True:
        inRgb = qRgb.get()
        img = inRgb.getCvFrame()
        cv2.imshow("rgb", img)
        key = cv2.waitKey(1)
        if key == ord('t'):
            if num_img < num_of_cal:
                if num_img<10:
                    cv2.imwrite(img_calibrate_path + "chessboard-0{}.jpg".format(num_img), img)
                else:
                    cv2.imwrite(img_calibrate_path+"chessboard-{}.jpg".format(num_img), img)
                print("Calibrate {}".format(num_img))
                num_img += 1
        elif key == ord('q'):
            break
