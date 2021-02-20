#!/usr/bin/env python


import rospy
import serial
import utm
import roslib
roslib.load_manifest('gps_driver')
from gps_driver.msg import gps_message


if __name__=='__main__':

	gps_pub = rospy.Publisher('gps_message', gps_message, queue_size=10)
	msg=gps_message()
	rospy.init_node("gps_driver", anonymous=True)
	rate = rospy.Rate(1)
	serial_port ='/dev/ttyUSB0'
	serial_baud = 4800
	port = serial.Serial(serial_port,serial_baud,timeout=3.0)
	i=0
	f=open("stationary_data.txt",'w')
	while(not rospy.is_shutdown()):
		line = port.readline()
		if line[1:6]=='GPGGA':
			data=line[7:].split(',')
			if data[5]!='0':
				lat=float(data[1][:2])+float(data[1][2:])/60.0
				lat_dir=data[2]
				if lat_dir=='S':
					lat=-lat
				lon=float(data[3][:3])+float(data[3][3:])/60.0
				lon_dir=data[4]
				if lon_dir=='W':
					lon=-lon
				status=data[5]
				sat_num=data[6]
				alt=float(data[8])
				UTM_coordiante=utm.from_latlon(lat,lon)
				s_UTM="%f,%f,%d%s,%s,%s\n"%(UTM_coordiante[0],UTM_coordiante[1],UTM_coordiante[2],UTM_coordiante[3],data[5],data[6])
				f.write(s_UTM)		
				msg.latitude=lat
				msg.longitude=lon
				msg.altitude=alt
				msg.utm_easting=float(UTM_coordiante[0])
				msg.utm_northing=float(UTM_coordiante[1])
				msg.zone="%d%s"%(UTM_coordiante[2],UTM_coordiante[3])
				rate.sleep()
				gps_pub.publish(msg)
	f.close()
