import rospy
from std_msgs.msg import String
import tf2_ros
from tf2_geometry_msgs import PointStamped

if __name__=="__main__":
    rospy.init_node("listener_pose")
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    rate = rospy.Rate(100)
    point_source1 = PointStamped()
    point_source1.header.frame_id = "son1"
    point_source1.point.x = 0.001
    point_source1.point.y = 0.001
    point_source1.point.z = 0.001

    point_source2 = PointStamped()
    point_source2.header.frame_id = "son2"
    point_source2.point.x = 0.001
    point_source2.point.y = 0.001
    point_source2.point.z = 0.001

    point_source3 = PointStamped()
    point_source3.header.frame_id = "son3"
    point_source3.point.x = 0.001
    point_source3.point.y = 0.001
    point_source3.point.z = 0.001

    point_source4 = PointStamped()
    point_source4.header.frame_id = "son4"
    point_source4.point.x = 0.001
    point_source4.point.y = 0.001
    point_source4.point.z = 0.001
    while not rospy.is_shutdown():
        try:
            point_target = buffer.transform(point_source1,"world",rospy.Duration(1))
            rospy.loginfo("转换结果:x=%.2f,y=%.2f,z=%.2f",
                          point_target.point.x,
                          point_target.point.y,
                          point_target.point.z)
            
        except Exception as e:
            rospy.logerr("异常%s",e)