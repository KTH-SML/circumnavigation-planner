#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import std_msgs.msg as sms
import threading as thd
import numpy as np

#Paramaters
estimate_gain=rp.get_param('estimate_gain') #from the .yaml file
#Initial estimate
estimate=np.array(rp.get_param('initial_estimate')) #from the .launch file
position=None
bearing_measurement=None
#Lock
LOCK=thd.Lock();


rp.init_node('estimate')

FREQUENCY = 10e1
RATE = rp.Rate(FREQUENCY)
TIME_STEP = 1.0/FREQUENCY #Integration step

#This node publishes the estimate of the target position and subscribes to the agent position topic and to the bearing measurements vector(phi)
#Subscribers
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
    
def bearing_measurement_callback(msg):
    global bearing_measurement
    LOCK.acquire()
    bearing_measurement = np.array([msg.x, msg.y])
    LOCK.release()
rp.Subscriber(
    name='bearing_measurement',
    data_class=gms.Vector,
    callback=bearing_measurement_callback,
    queue_size=10)

#Publisher
estimate_pub = rp.Publisher(
    name='estimate',
    data_class=gms.Point,
    queue_size=10)


start = False
while not rp.is_shutdown() and not start:
    LOCK.acquire()
    if all([not data is None for data in [position, bearing_measurement]]):
           start = True
    #else:
        #rp.logwarn('waiting for position and measurements')
    LOCK.release()
    #Initial estimate publishing
    estimate_pub.publish(gms.Point(x=estimate[0], y=estimate[1]))
    RATE.sleep()
while not rp.is_shutdown():
    LOCK.acquire()
    #Estimate algorithm
    d_estimate=-estimate_gain*(np.eye(2)-np.outer(bearing_measurement,bearing_measurement)).dot(estimate-position)
    #rp.logwarn(d_estimate)
    #Integration
    estimate= estimate+d_estimate*TIME_STEP
    LOCK.release()
    #Estimate publishing
    #rp.logwarn(estimate)
    estimate_pub.publish(gms.Point(x=estimate[0], y=estimate[1]))
    RATE.sleep()
