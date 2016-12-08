#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import std_msgs.msg as sms
import threading as thd
import numpy as np


position = None
estimate = np.array(rp.get_param('initial_estimate'))
neighbor_bearing_measurement = None
bearing_measurement = None

LOCK = thd.Lock()

rp.init_node('planner')



def position_callback(msg):
    global position
    LOCK.acquire()
    position = np.array([msg.x, msg.y])
    LOCK.release()
rp.Subscriber(
    name='position',
    data_class=gms.Point,
    callback=position_callback,
    queue_size=1)


def neighbor_bearing_measurement_callback(msg):
    global neighbor_bearing_measurement
    LOCK.acquire()
    neighbor_bearing_measurement = np.array([msg.x, msg.y])
    LOCK.release()
rp.Subscriber(
    name='neighbor_bearing_measurement',
    data_class=gms.Vector,
    callback=neighbor_bearing_measurement_callback,
    queue_size=1)


def bearing_measurement_callback(msg):
    global bearing_measurement
    LOCK.acquire()
    bearing_measurement = np.array([msg.x, msg.y])
    LOCK.release()
rp.Subscriber(
    name='bearing_measurement',
    data_class=gms.Vector,
    callback=bearing_measurement_callback,
    queue_size=1)


RATE = rp.Rate(30.0)
start = False
pub = rp.Publisher(
    name='cmdvel',
    data_class=gms.Vector,
    queue_size=1)

while not rp.is_shutdown() and not start:
    LOCK.acquire()
    if all([not data is None for data in [position, neighbor_bearing_measurement, bearing_measurement]]):
           start = True
    else:
        rp.logwarn('waiting for measurements')
    LOCK.release()
    RATE.sleep()
while not rp.is_shutdown():
    LOCK.acquire()
    msg = gms.Vector(x=0.01, y=0.01)
    pub.publish(msg)
    LOCK.release()
    RATE.sleep()
