#!/usr/bin/env python
# encoding: utf-8

import sys
import math

import rospy
import signal

# image
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# move
from move_base_msgs.msg import MoveBaseAction
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from actionlib import SimpleActionClient

# arm
import moveit_commander


def kb_interrupt_handler(signum, frame):
    print("quit")
    rospy.signal_shutdown("closed!")
    exit(0)


# arm & gripper
moveit_commander.roscpp_initialize(sys.argv)
arm = moveit_commander.MoveGroupCommander("arm")
gripper = moveit_commander.MoveGroupCommander("gripper")

def set_arm(js):
    jstate = list(js)
    arm.set_joint_value_target(jstate)
    plan = arm.plan()
    arm.execute(plan, wait=True)
    print('arm done')

def set_gripper(ang):
    ang *= math.pi / 180
    target = [-ang, ang, -ang, -ang, ang, ang]
    gripper.go(target)


# move
critic_done = False
def moveto(args):
    cp = Pose2D()
    cp.x, cp.y, cp.theta = args
    global critic_done
    critic_done = False
    global pub
    pub.publish(cp)
    while not critic_done:
        pass


class Height:
    a = 0.12934936
    b = 0.12941872
    def __init__(self, init_jd):
        # init_j: rad, y_thres: pix
        init_j = init_jd * math.pi / 180
        self.js = init_j
        self.h = math.cos(init_j[1]) * self.a + math.cos(init_j[1] + init_j[2]) * self.b
        self.sa = init_j[1] + init_j[2] + init_j[3]
    def update(self, d0, d1):
        self.js[0] += d0 * math.pi / 180
        self.js[1] += d1 * math.pi / 180
        self.js[2] = math.acos((self.h - math.cos(self.js[1]) * self.a) / self.b) - self.js[1]
        self.js[3] = self.sa - self.js[1] - self.js[2]
        return self.js


class Pick:
    br = CvBridge()
    def __init__(self, hsv, coef, aim, jd0):
        self.hsv_l, self.hsv_h = hsv
        self.x_coef, self.y_coef = coef
        self.aimX, self.aimY = aim
        self.jd0 = jd0
        self.picking = False
    def start(self):
        self.n_ok = 0
        self.plat = Height(self.jd0)
        set_arm(self.jd0 * math.pi / 180)
        self.picking = True
    def work(self, msg):
        if not self.picking:
            return
        img = self.br.imgmsg_to_cv2(msg)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self.hsv_l, self.hsv_h)
        if not mask.any():
            print('wtf')
            return
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        roi = min(contours, key=lambda x:abs(np.mean(x[:, 0, 0])- self.aimX))
        mean_x = np.mean(roi[:, 0, 0])
        max_y = np.min(roi[:, 0, 1])
        ok_x = abs(mean_x - self.aimX) < 10
        ok_y = abs(max_y - self.aimY) < 10
        dx = (mean_x - self.aimX) * self.x_coef if not ok_x else 0
        dy = (self.aimY - max_y) * self.y_coef if not ok_y else 0
        if ok_x and ok_y:
            self.n_ok += 1
        else:
            self.n_ok = 0
            jt = self.plat.update(dx, dy)
            print('now_x', mean_x, 'aim_x', self.aimX)
            print('now_y', max_y, 'aim_y', self.aimY)
            set_arm(jt)
        if self.n_ok >= 20:
            self.picking = False


# now picking
nowpick = None

class Run:
    def __init__(self, ball, pick, dest, mid0 = [], mid1 = []):
        self.ball = ball
        self.pick = pick
        self.dest = dest
        self.mid0 = mid0 # before picking
        self.mid1 = mid1 # before dropping
    def work(self):
        set_gripper(80)
        set_arm([0, 0, 0, math.pi / 2])
        for x in self.mid0:
            moveto(x)
        moveto(self.ball)
        print('aprroaching ball')
        # raw_input('in position')
        global nowpick
        nowpick = self.pick
        self.pick.start()
        while self.pick.picking:
            pass
        # raw_input('ready to pick')
        set_gripper(20)
        set_arm([0, 0, 0, math.pi / 2])
        print('picked!')
        for x in self.mid1:
            moveto(x)
        moveto(self.dest)
        set_arm([0, math.pi / 12.0, math.pi * 5 / 12.0, 0])
        set_gripper(80)
        print('dropped')
        set_arm([0, 0, 0, math.pi / 2])


# red
red_l = np.array([0, 184, 52])
red_h = np.array([0, 255, 255])
# blue
blue_l = np.array([90, 154, 0])
blue_h = np.array([147, 255, 255])

jd1 = np.array([0, -30, 90, 50], dtype=float)   # highest platform
jd2 = np.array([0, 10, 70, 32], dtype=float)  # far platform
jd3 = np.array([0, -10, 110, 10], dtype=float)  # near platform
jd4 = np.array([0, -30, 100, 40], dtype=float)  # circle platform

x_coef = 2.7e-3
y_coef = 7e-3

pick1r = Pick((red_l, red_h), (x_coef, y_coef), (320, 350), jd1)
pick1b = Pick((blue_l, blue_h), (x_coef, y_coef), (320, 350), jd1)
pick2r = Pick((red_l, red_h), (x_coef, y_coef), (320, 350), jd2)
pick2b = Pick((blue_l, blue_h), (x_coef, y_coef), (320, 350), jd2)
pick3r = Pick((red_l, red_h), (x_coef, y_coef), (320, 350), jd3)
pick3b = Pick((blue_l, blue_h), (x_coef, y_coef), (320, 350), jd3)
pick4r = Pick((red_l, red_h), (x_coef, y_coef), (320, 350), jd4)
pick4b = Pick((blue_l, blue_h), (x_coef, y_coef), (320, 350), jd4)


def img_cbk(msg):
    # print('watcher callback')
    global nowpick
    if nowpick is not None:
        nowpick.work(msg)

def critic_cbk(msg):
    print(msg)
    global critic_done
    critic_done = True


# start
rospy.init_node('watcher', anonymous=False)
signal.signal(signal.SIGINT, kb_interrupt_handler)

# move
actionlib_client = SimpleActionClient('move_base', MoveBaseAction)
actionlib_client.wait_for_server()

# watch
rospy.Subscriber('/usb_cam/image_raw', Image, img_cbk)
rospy.Subscriber('critic_done', String, critic_cbk)

# critic
pub = rospy.Publisher('critic_goal', Pose2D, queue_size=10)

# go!
pi = math.pi
hpi = math.pi / 2
dest = (0, -0.4, hpi)

runs = [
        Run((-0.3, -0.82, -0.05), pick4b, dest),
        Run((0.0, -0.5, -hpi), pick4b, dest),
        Run((0.3, -0.8, pi - 0.05), pick4r, dest),
        Run((0.0, -1.1, hpi), pick4r, dest, mid0=[(-0.3, -0.5, hpi), (-0.3, -1.1, hpi)], mid1=[(0.3, -1.1, hpi), (0.3, -0.5, hpi)]),

        Run((-0.34, -1.05, -hpi), pick3r, dest, mid0=[(-0.5, -0.5 , -hpi)], mid1=[(-0.5, -0.5, 0.0)]),
        Run((-0.25, -1.05, -hpi), pick3b, dest, mid0=[(-0.5, -0.5, -hpi)], mid1=[(-0.5, -0.5, 0.0)]),
        Run((0.34, -1.05, -hpi), pick2r, dest, mid0=[(0.5, -0.5, -hpi)], mid1=[(0.5, -0.5, 0.0)]),
        Run((0.25, -1.05, -hpi), pick2b, dest, mid0=[(0.5, -0.5, -hpi)], mid1=[(0.5, -0.5, 0.0)]),
        
        Run((-0.13, -1.1, -hpi), pick1b, dest, mid0=[(-0.5, -0.5, 0.0), (-0.5, -1.0, -hpi)], mid1=[(-0.5, -1.0, 0.0), (-0.5, -0.5, hpi)]),
        Run((0.13, -1.1, -hpi), pick1b, dest, mid0=[(0.5, -0.5, 0.0), (0.5, -1.0, -hpi)], mid1=[(0.5, -1.0, 0), (0.5, -0.5, hpi)]),
        Run((0.05, -1.1, -hpi), pick1r, dest, mid0=[(0.5, -0.5, 0.0), (0.5, -1.0, -hpi)], mid1=[(0.5, -1.0, 0), (0.5, -0.5, hpi)]),
        Run((-0.05, -1.1, -hpi), pick1r, dest, mid0=[(-0.5, -0.5, 0.0), (-0.5, -1.0, -hpi)], mid1=[(-0.5, -1.0, 0.0), (-0.5, -0.5, hpi)])
    ]

print('start')
for r in runs:
    r.work()

print('all done')

