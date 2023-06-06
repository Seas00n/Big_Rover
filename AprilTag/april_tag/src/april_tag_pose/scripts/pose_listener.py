import rospy
from std_msgs.msg import String
import tf2_ros
from tf2_geometry_msgs import PointStamped
import gatt
import sys
from imu_ import AnyDevice






if __name__=="__main__":
    rospy.init_node("listener_pose")
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    rate = rospy.Rate(100)
    
    mac_address = "6B:C3:BA:65:E3:86"
    port = 7890
    print("Connecting bluetooth...")
    
    manager = gatt.DeviceManager(adapter_name='hci0')
    device = AnyDevice(manager=manager, mac_address=mac_address)
    device.sock_pc = None
    device.parse_imu_flage = True
    device.connect()
    manager.run()


    while not rospy.is_shutdown():
        try:
            tfs = buffer.lookup_transform("world","son2",rospy.Time(0))
            point_source = PointStamped()
            rospy.loginfo("son2 in world")
            rospy.loginfo("相对坐标:x=%.2f, y=%.2f, z=%.2f",
                        tfs.transform.translation.x,
                        tfs.transform.translation.y,
                        tfs.transform.translation.z)
        except Exception as e:
            rospy.logerr("异常%s",e)
        rate.sleep()