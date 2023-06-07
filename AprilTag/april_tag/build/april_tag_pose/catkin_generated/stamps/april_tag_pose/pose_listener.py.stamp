import rospy
from std_msgs.msg import String
import tf2_ros
from tf2_geometry_msgs import PointStamped
import gatt
import sys
import numpy as np
import cv2
from scipy import io


if __name__=="__main__":
    rospy.init_node("listener_pose")
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    rate = rospy.Rate(100)
    
    imu_buffer = np.memmap("/media/yuxuan/SSD/Big_Rover_Data/imu_april_tag/imu_buffer.npy",dtype='float32',mode='r',shape=(12,))
    data_buffer = np.zeros((20,))
    img = np.zeros((10, 10), np.uint8)
    # 浅灰色背景
    img.fill(200)
    while not rospy.is_shutdown():
        try:
            tfs = buffer.lookup_transform("world","son2",rospy.Time(0))
            point_source = PointStamped()
            rospy.loginfo("son2 in world")
            rospy.loginfo("相对坐标:x=%.2f, y=%.2f, z=%.2f",
                        tfs.transform.translation.x,
                        tfs.transform.translation.y,
                        tfs.transform.translation.z)
            rospy.loginfo("IMU data:x=%.2f, y=%.2f, z=%.2f",
                          imu_buffer[9],
                          imu_buffer[10],
                          imu_buffer[11])
            data_temp = np.zeros((20,))
            for i in range(12):
                data_temp[i] = imu_buffer[i]
            data_temp[12] = tfs.transform.translation.x
            data_temp[13] = tfs.transform.translation.y
            data_temp[14] = tfs.transform.translation.z
            data_temp[15] = tfs.transform.rotation.x
            data_temp[16] = tfs.transform.rotation.y
            data_temp[17] = tfs.transform.rotation.z
            data_temp[18] = tfs.transform.rotation.w
            data_temp[19] = rospy.get_time()
            print(rospy.get_time())
            data_buffer = np.vstack(data_buffer,data_temp)
            cv2.imshow("rgb",img)
            if cv2.waitKey(1)==ord('q'):
                io.savemat("/media/yuxuan/SSD/Big_Rover_Data/imu_april_tag/data.mat",{"data":data_buffer})
        except Exception as e:
            rospy.logerr("异常%s",e)
        rate.sleep()