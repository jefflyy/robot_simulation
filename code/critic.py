#!/usr/bin/env python
import rospy

from tf_conversions import transformations
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import String
from math import pi, sin, cos
import tf

class Robot:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        try:
            self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(1.0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return
    def get_pos(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf Error")
            return None
        euler = transformations.euler_from_quaternion(rot)
        x = trans[0]
        y = trans[1]
        ang = euler[2]
        return x, y, ang

def set_spd(x, y, z, wx, wy, wz):
    print('set', x, y, wz)
    global vpub
    tw = Twist()
    tw.linear.x = x
    tw.linear.y = y
    tw.linear.z = z
    tw.angular.x = wx
    tw.angular.y = wy
    tw.angular.z = wz
    vpub.publish(tw)

def subcallback(msg):
    print(msg)
    global robot
    r = rospy.Rate(100)

    coef_x = 10
    coef_y = 10
    coef_a = 10

    def d_ang(a, b):
        if a > b:
            if a - b < b + pi + pi - a:
                return a - b
            else:
                return -(b + pi + pi - a)
        if a < b:
            if b - a < a + pi + pi - b:
                return a - b
            else:
                 return a + pi + pi - b

    ok_cnt = 0
    while True:
        x, y, a = robot.get_pos()
        tx, ty, ta = msg.x, msg.y, msg.theta
        sin_a = sin(a)
        cos_a = cos(a)
        print('now', x, y, a)
        print('aim', tx, ty, ta)
        dx = cos_a * (tx - x) + sin_a * (ty - y)
        dy = -sin_a * (tx - x) + cos_a * (ty - y)
        da = d_ang(ta, a)
        if abs(tx - x) < 0.01 and abs(ty - y) < 0.01 and abs(da) < 0.02:
            ok_cnt += 1
            if ok_cnt >= 40:
                break
        else:
            ok_cnt = 0
            set_spd(dx * coef_x, dy * coef_y, 0, 0, 0, da * coef_a)
        r.sleep()
    
    set_spd(0, 0, 0, 0, 0, 0)
    global rpub
    rpub.publish('ok')
    

rospy.init_node('get_pos',anonymous=True)
rospy.Subscriber('critic_goal', Pose2D, subcallback)

vpub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=10)
rpub = rospy.Publisher('critic_done', String, queue_size=10)
robot = Robot()

# r = rospy.Rate(100)
# while not rospy.is_shutdown():
#     print(robot.get_pos())
#     r.sleep()

rospy.spin()
