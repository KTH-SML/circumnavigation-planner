#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms

import threading as thd

import numpy as np


LOCK = thd.Lock()
#Initial position
position = np.array(rp.get_param('initial_position'))
#Velocity
velocity = None


XMIN = rp.get_param('xmin') 
XMAX = rp.get_param('xmax')
YMIN = rp.get_param('ymin')
YMAX = rp.get_param('ymax')


rp.init_node('integrator')

FREQUENCY = 10e1
RATE = rp.Rate(FREQUENCY)
TIME_STEP = 1.0/FREQUENCY

#Publisher
pub = rp.Publisher('position', gms.Point, queue_size=10)


def cmdvel_callback(msg):
    global velocity
    LOCK.acquire()
    velocity = np.array([msg.x, msg.y])
    LOCK.release()
rp.Subscriber(
    name='cmdvel',
    data_class=gms.Vector,
    callback=cmdvel_callback,
    queue_size=10)


start = False
while not rp.is_shutdown() and not start:
    LOCK.acquire()
    if not velocity is None:
        start = True
    #else:
        #rp.logwarn('waiting for cmdvel')
    LOCK.release()
    #Initial position publishing
    pub.publish(gms.Point(x=position[0], y=position[1]))
    RATE.sleep()
while not rp.is_shutdown():
    LOCK.acquire()
    #Integration
    position = position+velocity*TIME_STEP
    LOCK.release()
    #Position publishing
    pub.publish(gms.Point(x=position[0], y=position[1]))
    RATE.sleep()
