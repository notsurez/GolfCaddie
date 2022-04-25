#!/usr/bin/env python
# coding: utf-8
import rospy
from std_msgs.msg import String, Int16, Float32
from geographiclib.geodesic import Geodesic
import math
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan
import BNO055
import time

#from sensor_msgs.msg import Imu

bno = BNO055.BNO055(serial_port='/dev/ttyUSB0')
#lat, long, angle, mag, signal(mode)
phone_str = ["-1", "-1", "0", "0", "u"]
caddie_gps = ["-1", "-1"]
caddie_lat = 0.0
caddie_lon = 0.0 
phone_lat = 0.0
phone_lon = 0.0
phone_lat1 = 43.141731
phone_lon1 = -75.226937
gps_distance = 0
gps_heading = 0
phone_joystick_angle = 0
phone_joystick_mag = 0
phone_mode = ""
fwd_azimuth = 0
back_azimuth = 0
distance = 0
joystick_move_cmd = Twist()
d_mode= 1
auto_move_cmd = Twist()
qx = 0
qy = 0
qz = 0
qw = 0
yawDeg = 0
pitchDeg = 0
rollDeg = 0
heading = 0


#Variables used in obstacle avoidance
global dists
dists = []
distance_obst = 0

'''
def imu_data_cb(data):
	global qx
	global qy
	global qz
	global qw
	imu_data = data.data
	qx = imu_data.orientation.x
	qy = imu_data.orientation.y
	qz = imu_data.orientation.z
	qw = imu_data.orientation.w
	print("qx: " + "," + qx + "," + "qy: " + "," + qy + "," + "qz: " + "," + qz + "," + "qw: "+ "," + qw)
	quat_to_ypr()
'''
def caddie_gps_cb(data):
	global caddie_lat
	global caddie_lon
	#global gps_msg_heading
	#global gps_heading_valid
	#Display the received data in the terminal
	#Data is data.Received in data
	caddie_gps = data.data #lat,long -> as the message

	#parse caddie_gps here
	gps_coord_caddie = caddie_gps.split(",")
	caddie_lat = gps_coord_caddie[0]
	caddie_lon = gps_coord_caddie[1]
	#gps_msg_heading = gps_coord_caddie[2]
	#gps_heading_valid = gps_coord_caddie[3]

	rospy.loginfo(rospy.get_caller_id() + "I heard %s", caddie_gps)
	#print(str(caddie_lat) + "," + str(caddie_lon))
	#get_bearing()

def phone_str_cb(data):
	global phone_joystick_mag
	global phone_joystick_angle
	global phone_lat
	global phone_lon	
	global phone_mode
	global d_mode
	phone_data = data.data
        print("Phone Data" + str(phone_data) +'\n')
	#parse phone data here
	phone_data = phone_data[1:]
	phone_data_p = phone_data.split(",")
	phone_lat = phone_data_p[0]
	phone_lat = phone_lat[0:9]
	phone_lon = phone_data_p[1]
	phone_lon = phone_lon[0:10]
	phone_joystick_angle = phone_data_p[2]
	phone_joystick_mag = phone_data_p[3]
	phone_mode = phone_data_p[4]
	phone_mode = phone_mode[0] #TODO make letters correspond to integers
	if (phone_mode == "u"):
		d_mode = 1
	elif(phone_mode == "U"):
		d_mode = 0
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", phone_data)
	#print(str(phone_lat) + "," + str(phone_lon) + "," + str(phone_joystick_angle) + "," + str(phone_joystick_mag) + "," + str(phone_mode))
	joystick_convert()

def laser_scan_cb(msg):
	global distance
	global dists
	dists = msg.ranges;
	#distance = dists_to_dist(dists)
	if(dists):
		print(str(dists[256]))
'''
def quat_to_ypr():
	global qx
	global qy
	global qz
	global qw
	global rollDeg
	global yawDeg
	global pitchDeg
	
	# Create Roll Pitch Yaw Angles from Quaternions 
	yy = qy * qy; # 2 Uses below

	roll = atan2(2 * (qw * qx + qy * qz), 1 - 2*(qx *  qx + yy));
	pitch = asin(2 * w * qy -  qx * qz);
	yaw = atan2(2 * (w * qz +  qx * qy), 1 - 2*(yy+qz * qz));

	#  Convert Radians to Degrees 
	rollDeg  = 57.2958 * roll;
	pitchDeg = 57.2958 * pitch;
	yawDeg   = 57.2958 * yaw;	
'''
def get_bearing():
	global phone_lat
	global phone_lon
	global caddie_lat
	global caddie_lon
	global fwd_azimuth
	global distance	

	#print("p_lat: " + "," + str(phone_lat) + "," + "p_lon: " + "," + str(phone_lon) + "," + "c_lat: " + "," + str(caddie_lat) + "," + "c_lon: " + "," + str(caddie_lon))
	fwd_azimuth = Geodesic.WGS84.Inverse(float(caddie_lat), float(caddie_lon), float(phone_lat1), float(phone_lon1))['azi1']
	distance = Geodesic.WGS84.Inverse(float(caddie_lat), float(caddie_lon), float(phone_lat1), float(phone_lon1))['s12']
	if fwd_azimuth < 0:
		fwd_azimuth = fwd_azimuth + 360

def map_vals(x, in_min, in_max, out_min, out_max):

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;


def joystick_convert():	
	#TODO: convert to twist message [linx liny linz] [angx angy amgz] to send on cmd_veloctiy topic	
	# for the topic we will publish linx for linear front and back movemnet and angz for rotation 
	#in the z for rotation left and right
	global phone_joystick_mag
	global phone_joystick_angle
	y = float(phone_joystick_mag)*math.sin(math.radians(float(phone_joystick_angle)) + math.pi/2)
	x = float(phone_joystick_mag)*math.cos(math.radians(float(phone_joystick_angle)) + math.pi/2)
	phone_joystick_mag1 = -map_vals(float(x), -100.0, 100.0, -2.5, 2.5)
	phone_joystick_angle1 = map_vals(float(y), -100.0, 100.0, -2.0, 2.0)
	#phone_joystick_mag1 = ((float(phone_joystick_mag) / 1.0) * 4.0) - 2.0
	#phone_joystick_angle1 = ((float(phone_joystick_angle) / 359.0) * 4.0 ) - 2.0

	joystick_move_cmd.linear.x = phone_joystick_mag1
	joystick_move_cmd.angular.z = phone_joystick_angle1

def goal_to_goal():
	global heading	
	global distance
	global gps_msg_heading
	angularz = 0
	linearx = 0
	heading, roll, pitch = bno.read_euler()

	#Correct for magnetic north
	heading = heading + 80
	'''
	#resolve quadrent issues
	if (heading >= 0 and heading <=90):
		heading += 180
	elif (heading > 90 and heading <= 180):
		heading += 180
	'''
	if (heading > 360):
		heading = heading - 360	

	facing_goal = False
	head_diff = heading - fwd_azimuth

	print("Heading: " + str(heading))
	print("fwd_azimuth: " + str(fwd_azimuth))
	print("distance: " + str(distance))
	print("heading difference: " + str(head_diff))
	_range = 25
	#sys, gyro, accel, mag = bno.get_calibration_status()
	if((heading > fwd_azimuth-_range) and (heading < fwd_azimuth+_range)):
		facing_goal = True
	else:
		facing_goal = False


	if(facing_goal):
		if(distance < 18):
			linearx = 0
			angular = 0
		else:
			linearx = 1.5
			angularz = 0
	#Negative, turn right
	elif(head_diff < 180):
		linearx = 0
		angularz = 1.5
	#positive, turn left
	elif(head_diff >= 180):
		linearx = 0
		angularz = -1.5

	auto_move_cmd.linear.x = linearx
	auto_move_cmd.angular.z = angularz
	 
def avoid_obstacles():
	global dists
	linearx = 0
	angularz = 0
	objInPath = False
	_range = 20
	if(dists):
		for i in range(256-70, 256+70):
			if(dists[i] < 2.5):
				objInPath = True

	if(not objInPath):
		linearx = 1.5
		angularz = 0
	else:
		#Change this to turn towards goal if possible
		linearx = 0
		angularz = 1.5
	auto_move_cmd.linear.x = linearx
	auto_move_cmd.angular.z = angularz
    
def auto_mode():
    global heading	
	global distance
	global gps_msg_heading
    global dists
	linearx = 0
	angularz = 0
	objInPath = False
    heading, roll, pitch = bno.read_euler()

	#Correct for magnetic north
	heading = heading + 80
	'''
	#resolve quadrent issues
	if (heading >= 0 and heading <=90):
		heading += 180
	elif (heading > 90 and heading <= 180):
		heading += 180
	'''
	if (heading > 360):
		heading = heading - 360	

	facing_goal = False
	head_diff = heading - fwd_azimuth

	#print("Heading: " + str(heading))
	#print("fwd_azimuth: " + str(fwd_azimuth))
	#print("distance: " + str(distance))
	#print("heading difference: " + str(head_diff))
	_range = 25
	#sys, gyro, accel, mag = bno.get_calibration_status()
	if((heading > fwd_azimuth-_range) and (heading < fwd_azimuth+_range)):
		facing_goal = True
	else:
		facing_goal = False


	if(facing_goal):
		if(distance < 18):
			linearx = 0
			angular = 0
		else:
			linearx = 1.5
			angularz = 0
	#Negative, turn right
	elif(head_diff < 180):
		linearx = 0
		angularz = 1.5
	#positive, turn left
	elif(head_diff >= 180):
		linearx = 0
		angularz = -1.5
        
    #obstacle avoidance
    _range = 20
	if(dists):
		for i in range(256-70, 256+70):
			if(dists[i] < 2.5):
				objInPath = True

    while(objInPath):
        if(not objInPath):
            linearx = 1.5
            angularz = 0
        else:
            #Change this to turn towards goal if possible
            linearx = 0
            angularz = 1.5
    

	auto_move_cmd.linear.x = linearx
	auto_move_cmd.angular.z = angularz
    
    
    

def controller():
	#initialize IMU
	#bno = BNO055.BNO055(serial_port='/dev/ttyUSB0')

	if not bno.begin():
		raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
	'''
	while True:
		try:
			if not bno.begin():
	    			raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
			break
		except Exception as e:
			print("Got error: {}".format(e))
			time.sleep(1)
	'''	
	Cal_Data = bno.get_calibration()
	bno.set_calibration(Cal_Data)

	time.sleep(1)

	#Declare node namep
	rospy.init_node('controller', anonymous=True)
	#Subscribe to all nodes here and get their callbacks
	sub = rospy.Subscriber('gps', String, caddie_gps_cb)
	sub = rospy.Subscriber('chatter', String, phone_str_cb)
	sub = rospy.Subscriber('/scan', LaserScan, laser_scan_cb)

	#Create Publisher('Topic name', dataType, size)
	distance_heading = rospy.Publisher('dist_heading', String, queue_size=1)
	velocity_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	drive_modes = rospy.Publisher('driveMode', Int16, queue_size=1) 
	phone_joy_pub = rospy.Publisher('cmd_joystick', Twist, queue_size=1)
	imu_pub	= rospy.Publisher('imu', Float32, queue_size=1)



	#Loop period.
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		#Create the string to be published here
		#hello_str = "hello world %s" % rospy.get_time()
		gps_string = (str(distance) + "," + str(fwd_azimuth))
		#Display the data to publish in the terminal
		rospy.loginfo(gps_string)
		rospy.loginfo(joystick_move_cmd)
		rospy.loginfo(d_mode)
		#Publish data
		distance_heading.publish(gps_string)
		velocity_pub.publish(auto_move_cmd)
		phone_joy_pub.publish(joystick_move_cmd)
		drive_modes.publish(d_mode)
		imu_pub.publish(heading)

		get_bearing()		
	
		#Autonomous driving
		#avoid_obstacles()
                goal_to_goal()
		rate.sleep()

if __name__ == '__main__':
	try:
		controller()
	except rospy.ROSInitException:
		pass
