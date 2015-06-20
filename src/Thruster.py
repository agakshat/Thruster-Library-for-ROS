#Thruster Library
#Created on 05-12-2014 by Aayush Shaurya & Akshat Agarwal

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import time as time

class Thruster:

	global GND
	GND = "P9_1"
	COUNT_CONST=3

#	def __init__(self):
#		self.attachThruster(GND,GND,GND);

	def __init__(self, dir1_pin, dir2_pin, pwm_pin):
		self.attachThruster(dir1_pin, dir2_pin, pwm_pin);

	def attachThruster(self, dir1_pin, dir2_pin, pwm_pin):
		if(dir1_pin != GND and dir2_pin != GND and pwm_pin != GND):
			GPIO.setup(dir1_pin, GPIO.OUT)
			GPIO.setup(dir2_pin, GPIO.OUT)
			PWM.start(pwm_pin, 0)
		self.pwm = 0;
		self.speed = 0;
		self.meanSpeed = 100;
		self.dir1 = 0;
		self.dir2 = 0;
		self.dir1_pin = dir1_pin
		self.dir2_pin = dir2_pin
		self.pwm_pin = pwm_pin
		self.lockThruster();
		damping = 10;

	def getSpeed(self):
		return(pwm*100/mean_speed);

	def changePWM(self):
		if self.pwm>100:
			self.pwm=100
		elif self.pwm<-100:
			self.pwm=-100
		PWM.set_duty_cycle(self.pwm_pin, float(abs(self.pwm)))	

	def moveThruster(self, temp_dir1, temp_dir2, temp_pwm):
		self.dir1 = temp_dir1;
		self.dir2 = temp_dir2;
		self.pwm = temp_pwm;
		self.moveThruster();

	def moveThruster(self):

		if self.dir1 == 1 and self.dir2 == 0:
			self.pwm = pwm;
		elif self.dir1 == 0 and self.dir2 == 1:
			self.pwm = -pwm;

		if self.pwm >= 0:
			GPIO.output(self.dir1_pin, GPIO.HIGH)
			GPIO.output(self.dir2_pin, GPIO.LOW)
		else:
			GPIO.output(self.dir1_pin, GPIO.LOW)
			GPIO.output(self.dir2_pin, GPIO.HIGH)

		self.changePWM();

	def lockThruster(self):
			GPIO.setup(self.dir1_pin, GPIO.OUT)
			GPIO.setup(self.dir2_pin, GPIO.OUT)
			PWM.start("P8_13", 0.1)
			GPIO.output(self.dir1_pin, GPIO.HIGH)
			GPIO.output(self.dir2_pin, GPIO.HIGH)
			PWM.set_duty_cycle("P8_13", float(100))
			self.pwm = 0

	def freeThruster(self):
			GPIO.output(self.dir1_pin, GPIO.LOW)
			GPIO.output(self.dir2_pin, GPIO.LOW)
			PWM.set_duty_cycle(self.pwm_pin, float(0))
			self.pwm=0

	def setThrusterSpeed(self, temp_dir1, temp_dir2, temp_speed):
		
		if (temp_speed > 100):
			temp_speed = 100;
		if (temp_speed < -100):
			temp_speed = -100;
		
		self.speed = temp_speed;
		self.pwm = temp_speed*self.meanSpeed/100;
		self.dir1 = temp_dir1;
		self.dir2 = temp_dir2;
		self.moveThruster();

	def setThrusterSpeed(self, temp_speed):
		
		if (temp_speed > 100):
			temp_speed = 100;
		if (temp_speed < -100):
			temp_speed = -100;
		
		self.speed = temp_speed;
		self.pwm = temp_speed*self.meanSpeed/100;
		self.moveThruster();

	def changeSpeed(self, temp_speed):
		if (temp_speed > 100):
			temp_speed = 100;
		if (temp_speed < -100):
			temp_speed = -100;
		self.speed = temp_speed;
		self.changePWM(temp_speed*self.meanSpeed/100);

	def getDirection(self):
		if (self.dir1==self.dir2):
			return 0
		elif (self.dir1>self.dir2):
			return 1
		elif (self.dir2>self.dir1):
			return -1

	def isFree(self):
		return (self.getDirection()==0 and self.dir1==0)

	def isLocked(self):
		return (self.getDirection()==0 and self.dir1==1)

	def getSpeed(self):
		return(self.pwm*100/self.meanSpeed);

	def getPWM(self):
		return self.pwm;

	def startSmoothly(self, temp_speed):
		if (temp_speed > 100):
			temp_speed = 100;
		if (temp_speed < -100):
			temp_speed = -100;
		self.speed = temp_speed;
		self.startSmoothly();

	def startSmoothly(self):
		if (self.speed >= 0):
			GPIO.output(dir1_pin, GPIO.HIGH);
			GPIO.output(dir2_pin, GPIO.LOW);
		else:
			GPIO.output(dir1_pin, GPIO.LOW);
			GPIO.output(dir2_pin, GPIO.HIGH);

		i = 0;
		while (i < self.speed):
			self.changeSpeed(i);
			time.sleep(COUNT_CONST*damping/1000);
			#self.speed = i;
			i = i + COUNT_CONST
		self.pwm = self.speed*mean_speed/100

	def stopSmoothly(self):
		i=self.getSpeed()
		while (i>0):
			self.changeSpeed(i);
			time.sleep(COUNT_CONSR*damping/1000);
			i=i-COUNT_CONST;
		self.pwm = self.speed*mean_speed/100
		self.lockThruster()

