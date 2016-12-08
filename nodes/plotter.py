#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms

import threading as thd
import copy as cp

import matplotlib.pyplot as plt

rp.init_node('plotter')

#XMIN = rp.get_param('xmin')
#XMAX = rp.get_param('xmax')
#YMIN = rp.get_param('ymin')
#YMAX = rp.get_param('ymax')

AGENT_COLOR = rp.get_param('agent_color')
ESTIMATE_COLOR = rp.get_param('estimate_color')
TARGET_COLOR = rp.get_param('target_color')

AGENT_NAMES = rp.get_param('agent_names').split()
TARGET_POSITION = rp.get_param('target_position')

RATE = rp.Rate(3.0e1)

LOCK = thd.Lock()

plt.ion()
plt.figure()
plt.scatter(*TARGET_POSITION, color=TARGET_COLOR)
#plt.xlim((XMIN, XMAX))
#plt.ylim((YMIN, YMAX))
plt.axis('equal')
plt.grid(True)
plt.draw()

agent_positions = {name: None for name in AGENT_NAMES}
agent_artists = {name: None for name in AGENT_NAMES}

estimates = {name: None for name in AGENT_NAMES}
estimate_artists = {name: None for name in AGENT_NAMES}

def agent_callback(msg, name):
    global agent_positions
    LOCK.acquire()
    agent_positions[name] = [msg.x, msg.y]
    LOCK.release()
for name in AGENT_NAMES:
    rp.Subscriber(
        name=name+'/position',
        data_class=gms.Point,
        callback=agent_callback,
        callback_args=name,
        queue_size=1)

def estimate_callback(msg, name):
    global estimates
    LOCK.acquire()
    estimates[name] = [msg.x, msg.y]
    LOCK.release()
for name in AGENT_NAMES:
    rp.Subscriber(
        name=name+'/estimate',
        data_class=gms.Point,
        callback=estimate_callback,
        callback_args=name,
        queue_size=1)

# def estimate_callback(msg):
#     global estimate
#     LOCK.acquire()
#     estimate = [msg.x, msg.y]
#     LOCK.release()
# rp.Subscriber(name='estimate',
#               data_class=gms.Point,
#               callback=estimate_callback)

while not rp.is_shutdown():
    ag_pos = {name: None for name in AGENT_NAMES}
    est = {name: None for name in AGENT_NAMES}
    LOCK.acquire()
    for name in AGENT_NAMES:
        if not agent_positions[name] is None:
            ag_pos[name] = cp.copy(agent_positions[name])
            agent_positions[name] = None
        if not estimates[name] is None:
            est[name] = cp.copy(estimates[name])
            estimates[name] = None
    LOCK.release()
    for name in AGENT_NAMES:
        if not ag_pos[name] is None:
            if not agent_artists[name] is None:
                agent_artists[name].remove()
            agent_artists[name] = plt.scatter(*ag_pos[name], color=AGENT_COLOR)
        if not est[name] is None:
            if not estimate_artists[name] is None:
                estimate_artists[name].remove()
            estimate_artists[name] = plt.scatter(*est[name], color=ESTIMATE_COLOR)
    plt.draw()
    RATE.sleep()
