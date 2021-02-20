#!/usr/bin/env python


import rospy
import serial
import roslib
from sensor_msgs.msg import Imu, MagneticField
from tf.transformations import quaternion_from_euler
import math
if __name__=='__main__':

	imu_data_pub = rospy.Publisher('imu_message', Imu, queue_size=1)
	imu_data=Imu()
	mag_data_pub = rospy.Publisher('mag_message', MagneticField, queue_size=1)
	mag_data= MagneticField()

	rospy.init_node("imu_driver", anonymous=True)
	rate = rospy.Rate(100)
	serial_port ='/dev/ttyUSB0'
	serial_baud = 115200
	port = serial.Serial(serial_port,serial_baud,timeout=3.0)
	i=0
	#f=open("up.txt",'w')
	while(not rospy.is_shutdown()):
		line = port.readline()
		if line[1:6]=='VNYMR':
			data=line[7:-5].split(',')

			yaw  =float(data[0])
			pitch=float(data[1])
			roll =float(data[2])

			MagX =float(data[3])
			MagY =float(data[4])
			MagZ =float(data[5])

			AccX =float(data[6])
			AccY =float(data[7])
			AccZ =float(data[8])

			GyroX =float(data[9])
			GyroY =float(data[10])
			GyroZ =float(data[11])
			quaternion = quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))
			s_imu_data="%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n"%(yaw,pitch,roll,MagX,MagY,MagZ,AccX,AccY,AccZ,GyroX,GyroY,GyroZ)
			rospy.loginfo(s_imu_data)
			#f.write(s_imu_data)
			imu_data.header.stamp = rospy.Time.now()
			imu_data.orientation.x = quaternion[0]
			imu_data.orientation.y = quaternion[1]
			imu_data.orientation.z = quaternion[2]
			imu_data.orientation.w = quaternion[3]
			imu_data.linear_acceleration.x = AccX
			imu_data.linear_acceleration.y = AccY
			imu_data.linear_acceleration.z = AccZ
			imu_data.angular_velocity.x = GyroX
			imu_data.angular_velocity.y = GyroY
			imu_data.angular_velocity.z = GyroZ
			imu_data_pub.publish(imu_data)

			mag_data.header.stamp = rospy.Time.now()
			mag_data.magnetic_field.x = MagX
			mag_data.magnetic_field.y = MagY
			mag_data.magnetic_field.z = MagZ
			mag_data_pub.publish(mag_data)

			rate.sleep()
			
	#f.close()
