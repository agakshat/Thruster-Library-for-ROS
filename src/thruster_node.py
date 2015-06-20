#!/usr/bin/env python
import rospy
import roscpp
import sys
import math

from sensor_msgs.msg import Imu
from pid.msg import pid_array
from Thruster import Thruster

GND="P9_1"

thruster1=Thruster(rospy.get_param("1_D1", GND),rospy.get_param("1_D2", GND),rospy.get_param("1_PWM", GND))
thruster2=Thruster(rospy.get_param("2_D1", GND),rospy.get_param("2_D2", GND),rospy.get_param("2_PWM", GND))
thruster3=Thruster(rospy.get_param("3_D1", GND),rospy.get_param("3_D2", GND),rospy.get_param("3_PWM", GND))
thruster4=Thruster(rospy.get_param("4_D1", GND),rospy.get_param("4_D2", GND),rospy.get_param("4_PWM", GND))
thruster5=Thruster(rospy.get_param("5_D1", GND),rospy.get_param("5_D2", GND),rospy.get_param("5_PWM", GND))
thruster6=Thruster(rospy.get_param("6_D1", GND),rospy.get_param("6_D2", GND),rospy.get_param("6_PWM", GND))

#thruster1=Thruster("P8_12","P8_14","P9_12")
#thruster2=Thruster()
#thruster3=Thruster()
#thruster4=Thruster()
#thruster5=Thruster()
#thruster6=Thruster()


pwmyaw=[0,0,0,0,0,0]
pwmpitch=[0,0,0,0,0,0]
pwmroll=[0,0,0,0,0,0]
pwmarr=[0,0,0,0,0,0]

def blank1():
	rospy.loginfo(" Thruster Node has received Data Successfully from PID_Calc.\n")

def blank2():
	rospy.loginfo(" Thruster Node has received data successfully from Imu.\n")

def getData():
	rospy.init_node('thruster',anonymous=True)
	thrustersub_pid=rospy.Subscriber('pid_calc',pid_array,blank1)
	thrustersub_imu=rospy.Subscriber('imu',Imu,blank2)
	pidarr=pid_array()
	imuMsg=Imu()
	global bppid
	bppid=pidarr[0]
	global fppid
	fppid=pidarr[1]
	global aypid
	aypid=pidarr[2]
	global cypid
	cypid=pidarr[3]
	global rrpid
	rrpid=pidarr[4]
	global lrpid
	lrpid=pidarr[5]
	global pitch_paramsp
	pitch_paramsp=pidarr[6]
	global yaw_paramsp
	yaw_paramsp=pidarr[7]
	global roll_paramsp
	roll_paramsp=pidarr[8]
	global yaw
	yaw = imuMsg.orientation.x
	global pitch
	pitch = imuMsg.orientation.y
	global roll
	roll = imuMsg.orientation.z

def setPitchBearing():
	if (pitch_paramsp<pitch):
		rospy.loginfo("We are above in pitch.\n")
		pwmpitch[0]=pwmpitch[1]=fppid
		pwmpitch[2]=pwmpitch[3]=(-1)*bppid
	elif (pitch_paramsp>pitch):
		rospy.loginfo("We are below in pitch.\n")
		pwmpitch[0]=pwmpitch[1]=(-1)*fppid
		pwmpitch[2]=pwmpitch[3]=bppid

def setYawBearing():
	if (yaw_paramsp<yaw):
		rospy.loginfo("We are clockwise in yaw.\n")
		pwmyaw[4]=cypid
		pwmyaw[5]=aypid*(-1)
	elif (yaw_paramsp>yaw):
		rospy.loginfo("We are anticlockwise in yaw.\n")
		pwmyaw[4]=(-1)*cypid
		pwmyaw[5]=aypid

def setRollBearing():
	if (roll_paramsp<roll):
		rospy.loginfo("We are right in roll.\n")
		pwmroll[3]=pwmroll[1]=rrpid
		pwmroll[2]=pwmroll[0]=(-1)*lrpid
	elif (roll_paramsp>roll):
		rospy.loginfo("We are left in roll.\n")
		pwmroll[3]=pwmroll[1]=(-1)*rrpid
		pwmroll[2]=pwmroll[0]=lrpid

def cumulatePWM():
	for i in range(6):
		pwmarr[i]=pwmyaw[i]+pwmpitch[i]+pwmroll[i]#+pwmmove[i]

def updateThruster():
	thruster1.moveThruster(1,0,pwmarr[0])
	thruster2.moveThruster(1,0,pwmarr[1])
	thruster3.moveThruster(1,0,pwmarr[2])
	thruster4.moveThruster(1,0,pwmarr[3])
	thruster5.moveThruster(1,0,pwmarr[4])
	thruster6.moveThruster(1,0,pwmarr[5])

if __name__=='__main__':
	getData()
	setPitchBearing()
	setYawBearing()
	setRollBearing()
	cumulatePWM()
	updateThruster()

