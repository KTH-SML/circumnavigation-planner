#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import std_msgs.msg as smsp
import threading as thd
import numpy as np
import math

 

# Parameters and variables
position = None
estimate = None
neighbor_bearing_measurement = None
bearing_measurement = None
beta=None
DESIRED_DISTANCE = rp.get_param('desired_distance')  # from the .yaml file
alpha= rp.get_param('alpha') # from the .yaml file

# Lock
LOCK = thd.Lock()

rp.init_node('planner') 

#Counterclockwise_angle function
def Counterclockwise_angle(bearing_measurement,neighbor_bearing_measurement):	
	phi_i=np.array([bearing_measurement[0],bearing_measurement[1],0.0]) 
	phi_j=np.array([neighbor_bearing_measurement[0],neighbor_bearing_measurement[1],0.0])
	n_i=np.linalg.norm(phi_i)
	n_j=np.linalg.norm(phi_j)

	sp=np.inner(phi_i,phi_j)
	vp=np.cross(phi_i,phi_j)
	cos_beta=sp/(n_i*n_j)
	sin_beta=vp[2]/(n_i*n_j)

	beta=math.atan2(sin_beta,cos_beta)
	 
	if beta<0:
		beta=beta+2*math.pi
	
	return beta 

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

def estimate_callback(msg):
	global estimate
	LOCK.acquire()
	estimate = np.array([msg.x, msg.y])
	LOCK.release()
rp.Subscriber(
	name='estimate',
	data_class=gms.Point,
	callback=estimate_callback,
	queue_size=10)

RATE = rp.Rate(100.0)
start = False

#Publishers
cmdvel_pub = rp.Publisher(
	name='cmdvel',
	data_class=gms.Vector,
	queue_size=10)


while not rp.is_shutdown() and not start:
	LOCK.acquire()
	if all([not data is None for data in [position, estimate, neighbor_bearing_measurement, bearing_measurement]]):
		   start = True
	#else:
		#rp.logwarn('waiting for measurements)
	LOCK.release()
	RATE.sleep()
while not rp.is_shutdown():
	LOCK.acquire()
	#Bearing vector in the clockwise direction
	phi_bar=np.array([bearing_measurement[1],-bearing_measurement[0]])
	#Counterclockwise angle
	beta=Counterclockwise_angle(bearing_measurement,neighbor_bearing_measurement)
	#Control law
	est_dist = np.linalg.norm(estimate-position)
	vel = bearing_measurement*(est_dist-DESIRED_DISTANCE)+0.3*est_dist*phi_bar*(alpha+beta)
	#Velocity message
	cmdvel_msg = gms.Vector(x=vel[0], y=vel[1])
	LOCK.release()
	# Velocity publishing
	cmdvel_pub.publish(cmdvel_msg)
	RATE.sleep()
