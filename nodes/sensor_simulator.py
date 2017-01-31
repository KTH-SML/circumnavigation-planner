#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import std_msgs.msg as sms
import threading as thd
import numpy as np



rp.init_node('sensor_simulator')

position = None
TARGET_POSITION = np.array(rp.get_param('target_position')) #from the .yaml file

LOCK = thd.Lock()

#Subscriber
def position_callback(msg):
    global position
    LOCK.acquire()
    position = np.array([msg.x, msg.y])
    LOCK.release()
rp.Subscriber(
    name='position',
    data_class=gms.Point,
    callback=position_callback,
    queue_size=10)


RATE = rp.Rate(100.0)
start = False
#Publisher
pub = rp.Publisher(
    name='bearing_measurement',
    data_class=gms.Vector,
    queue_size=10)

while not rp.is_shutdown() and not start:
    LOCK.acquire()
    if not position is None:
        start = True
    #else:
        #rp.logwarn('waiting for position')
    LOCK.release()
    RATE.sleep()
while not rp.is_shutdown():
    LOCK.acquire()
    #Bearing vector (phi)
    bearing = (TARGET_POSITION-position)/np.linalg.norm(TARGET_POSITION-position)
    msg = gms.Vector(*bearing)
    #Bearing vector publishing
    pub.publish(msg)
    LOCK.release()
    RATE.sleep()
