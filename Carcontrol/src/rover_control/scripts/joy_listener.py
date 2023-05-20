import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math
from rover_control.msg import rover
from enum import Enum



f = 8
z = 1.2
r = 0.0
k1 = z/math.pi/f
k2 = 1/((2*math.pi*f)*(2*math.pi*f))
k3 = r*z/(2*math.pi*f)


State = {'GO_AHEAD':0, 'CHANGE_TO_TURN':1, "TURN": 2, "CHANGE_TO_GO": 3}

car_vp = 0 #当前期望速度
car_wp = 0 #当前期望角速度
car_vnew = 0 #下一时刻速度
car_wnew = 0 #下一时刻角速度
car_statenew = 0



def damp_joy(x_previous,x_new,y,yd,dt):
    xd_new = (x_new-x_previous)/dt
    y = y+dt*yd
    yd = yd+dt*(x_new+k3*xd_new-y-k1*yd)/k2
    x_previous = x_new
    return x_previous,y,yd



def callback(data):
    global car_vnew, car_wnew, car_statenew
    # rospy.loginfo("axes %s",data.axes)
    # rospy.loginfo("button %s",data.buttons)
    axes1 = data.axes[0]
    axes2 = data.axes[1]
    axes3 = data.axes[3]
    if axes2>0:
        car_vnew = math.sqrt(axes1*axes1+axes2*axes2)*4
    else:
        car_vnew = -math.sqrt(axes1*axes1+axes2*axes2)*4
    
    car_wnew = axes3*2

    if abs(data.buttons[3]-1)<0.001 and car_statenew == 3: #Y pressed
        car_statenew = State["GO_AHEAD"]
        car_wnew = 0
    elif abs(data.buttons[1]-1)<0.001 and car_statenew == 0: #B pressed
        car_statenew = State["CHANGE_TO_TURN"]
        car_vnew = 0
        car_wnew = 0
    elif abs(data.buttons[0]-1)<0.001 and car_statenew == 1:#A pressed
        car_statenew = State["TURN"]
        car_vnew = 0
    elif abs(data.buttons[2]- 1)<0.001 and car_statenew == 2:#X pressed
        car_statenew = State["CHANGE_TO_GO"]
        car_vnew = 0
        car_wnew = 0


msg = rover()
if __name__=="__main__":
    rospy.init_node("joy_listener",anonymous=True)
    rospy.Subscriber("joy",Joy,callback)
    pub = rospy.Publisher("rover_state",rover,queue_size=100)
    rate = rospy.Rate(100)
    dt = 0.01
    msg_vp = 0 #经过二阶系统平滑后的下发速度
    msg_ap = 0 #经过二阶段系统平滑后的下发加速度
    msg_wp = 0 #经过二阶系统平滑后的下发角速度
    msg_wap = 0 #经过二阶段系统平滑后的下发角加速度
    while not rospy.is_shutdown():
        car_vp, msg_vp, msg_ap = damp_joy(car_vp,car_vnew,msg_vp,msg_ap,dt)
        car_wp, msg_wp, msg_wap = damp_joy(car_wp,car_wnew,msg_wp,msg_wap,dt)
        msg.rover_vx = msg_vp
        msg.rover_w = msg_wp
        msg.rover_state = car_statenew
        print(msg_vp)
        pub.publish(msg)
        rate.sleep()
    rospy.spin()