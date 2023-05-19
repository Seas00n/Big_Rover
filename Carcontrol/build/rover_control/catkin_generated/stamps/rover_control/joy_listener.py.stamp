import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math





f = 3
z = 0.3
r = 0.02
k1 = z/math.pi/f
k2 = 1/((2*math.pi*f)*(2*math.pi*f))
k3 = r*z/(2*math.pi*f)

car_vnew = 0
car_wnew = 0

def damp_joy(x_previous,x_new,y,yd,dt):
    xd_new = (x_new-x_previous)/dt
    y = y+dt*yd
    yd = yd+dt*(x_new+k3*xd_new-y-k1*yd)/k2
    x_previous = x_new
    return x_previous,y,yd



def callback(data):
    global car_vnew, car_wnew
    rospy.loginfo("axes %s",data.axes)
    rospy.loginfo("button %s",data.buttons)
    axes1 = data.axes[0]
    axes2 = data.axes[1]
    if axes2>0:
        car_vnew = math.sqrt(axes1*axes1+axes2*axes2)*2
    else:
        car_vnew = -math.sqrt(axes1*axes1+axes2*axes2)*2
    axes3 = data.axes[3]
    car_wnew = axes3*5


msg = Twist()

if __name__=="__main__":
    rospy.init_node("joy_listener",anonymous=True)
    rospy.Subscriber("joy",Joy,callback)
    pub = rospy.Publisher("/turtle1/cmd_vel",Twist, queue_size=100)
    rate = rospy.Rate(100)
    dt = 0.01
    car_vp = 0
    car_wp = 0
    msg_vp = 0
    msg_ap = 0
    msg_wp = 0
    msg_wap = 0
    while not rospy.is_shutdown():
        car_vp, msg_vp, msg_ap = damp_joy(car_vp,car_vnew,msg_vp,msg_ap,dt)
        car_wp, msg_wp, msg_wap = damp_joy(car_wp,car_wnew,msg_wp,msg_wap,dt)
        msg.linear.x = msg_vp
        msg.angular.z = msg_wp
        pub.publish(msg)
        rate.sleep()
    rospy.spin()