#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms

import threading as thd

import numpy as np



def ccws_perp(array):
    x = array[0]
    y = array[1]
    return np.array([-y, x])


LOCK = thd.Lock()
position = np.array(rp.get_param('initial_position'))
velocity = None

XMIN = rp.get_param('xmin')
XMAX = rp.get_param('xmax')
YMIN = rp.get_param('ymin')
YMAX = rp.get_param('ymax')

rp.init_node('integrator')
FREQUENCY = 3e1
RATE = rp.Rate(FREQUENCY)
TIME_STEP = 1.0/FREQUENCY
pub = rp.Publisher('position', gms.Point, queue_size=10)

def cmdvel_callback(msg):
    global velocity
    LOCK.acquire()
    velocity = np.array([msg.x, msg.y])
    LOCK.release()
rp.Subscriber('cmdvel', gms.Vector, callback=cmdvel_callback)

start_flag = False
while not rp.is_shutdown() and not start_flag:
    LOCK.acquire()
    if not velocity is None:
        start_flag = True
    else:
        rp.logwarn('waiting for cmdvel')
    LOCK.release()
    pub.publish(gms.Point(x=position[0], y=position[1]))
    RATE.sleep()
while not rp.is_shutdown():
    LOCK.acquire()
    newpos = position+velocity*TIME_STEP
    #if newpos[0] >= XMIN and newpos[0] <= XMAX and newpos[1] >= YMIN and newpos[1] <= YMAX:
    #    position = newpos
    position = newpos
    velocity = np.zeros(2)
    LOCK.release()
    pub.publish(gms.Point(x=position[0], y=position[1]))
    RATE.sleep()
