#include <Thruster.h>
#include <Arduino.h>
Thruster::Thruster()									//constructor
{	
	attachThruster(-1,-1,-1);
}

Thruster::Thruster(int Dir1, int Dir2, int Pwm )		// parametrised constructor
{	
	Serial.begin(9600);
	attachThruster(Dir1,Dir2,Pwm);
	//Serial.println("Attached thruster fuck you.");
}

void Thruster::attachThruster(int Dir1,int Dir2,int Pwm)
{
	pwm_pin= Pwm;
	dir1_pin= Dir1;
	dir2_pin= Dir2;
	if(dir1!=-1 && dir2!=-1 && pwm_pin!=-1)
	{	
		pinMode(pwm_pin,OUTPUT);
		pinMode(dir1_pin,OUTPUT);
		pinMode(dir2_pin,OUTPUT);
	}
	pwm		= 0;
	speed		= 0;
	mean_speed	= 255;
	lockThruster();	//Lock the Thruster initially
	damping=10;
	}


void Thruster::moveThruster(int Pwm)			//+ve for CW and -ve for CCW.
{
	
	if(Pwm>=0)
	{	
		digitalWrite(dir1_pin,HIGH);
		digitalWrite(dir2_pin,LOW);
	}
	else
	{	
		digitalWrite(dir1_pin,LOW);
		digitalWrite(dir2_pin,HIGH);
	}
	changePWM(Pwm);
//	Serial.println("PWM=");
//	Serial.print(Pwm);
}

void Thruster::moveThruster(int Dir1,int Dir2,int Pwm)
{
	if(Dir1==1 && Dir2==0)
		moveThruster(Pwm);
	if(Dir1==0 && Dir2==1)
		moveThruster(-Pwm);
}

void Thruster::stopThruster() 				//By default stop Thruster will lock Thruster
{	
	lockThruster();
}

void Thruster::lockThruster()
{
	digitalWrite(dir1_pin,HIGH);			//case of Thruster lock
	digitalWrite(dir2_pin,HIGH);
	analogWrite(pwm_pin,255);
	pwm=0;	
}

void Thruster::freeThruster()
{
	digitalWrite(dir1_pin,LOW);			//case of Thruster free
	digitalWrite(dir2_pin,LOW);
	analogWrite(pwm_pin,0);
	pwm=0;
}
		
void Thruster::setMeanSpeed(int Speed)			//Sets the meanspeed with which Thruster moves when speed=100%	
{	
	mean_speed=Speed;
}

void  Thruster::setThrusterSpeed(int Speed)				//+ve for CW and -ve for CCW. Speed in percentage of meanspeed
{	
	if(Speed>100)
		Speed=100;
	if(Speed<-100)
		Speed=-100;
	speed=Speed;
	
	moveThruster(Speed*mean_speed/100);
}

void  Thruster::setThrusterSpeed(int Dir1,int Dir2,int Speed)
{	
	if(Speed>100)
		Speed=100;
	if(Speed<-100)
		Speed=-100;
	speed=Speed;
	moveThruster(Dir1,Dir2,Speed*mean_speed/100);
}

void Thruster::changePWM(int Pwm)					//Just to change the PWM
{
	pwm = Pwm>255 ? 255 : (Pwm < -255 ? -255 :Pwm);
	analogWrite(pwm_pin,abs(pwm));
}

void Thruster::changeSpeed(int Speed)				//Just to change the speed (In percentage)
{	
	if(Speed>100)
		Speed=100;
	if(Speed<-100)
		Speed=-100;
	speed=Speed;
	changePWM(Speed*mean_speed/100);
}

int Thruster::getDirection()
{
	dir1=digitalRead(dir1_pin);
	dir2=digitalRead(dir2_pin);
	if (dir1==dir2)
		return 0;
	else 
		return((dir1>dir2)?1:-1);
}

int Thruster::isFree()
{
	return (getDirection()==0 && dir1==0);
}

int Thruster::isLocked()
{
	return (getDirection()==0 && dir1==1);
}

int Thruster::getSpeed()
{
	return(pwm*100/mean_speed);
}

int Thruster::getPWM()
{
  return pwm;
}

 void Thruster::startSmoothly(int Speed)
 {
    if(Speed>100)
		Speed=100;
	if(Speed<-100)
		Speed=-100;
	int i;
	if(Speed>=0)
	{	
		digitalWrite(dir1_pin,HIGH);
		digitalWrite(dir2_pin,LOW);
	}
	else
	{	
		digitalWrite(dir1_pin,LOW);
		digitalWrite(dir2_pin,HIGH);
	}
	for(i=0;i<=Speed;i=i+COUNT_CONST)
	{
		changeSpeed(i);
		delay(COUNT_CONST*damping);
		speed=i;
		pwm=speed*mean_speed/100;
	}
 }
 void Thruster::stopSmoothly()
 {  int i;
	speed=getSpeed();
	for(i=speed;i>=0;i=i-COUNT_CONST)
	{
		changeSpeed(i);
		delay(COUNT_CONST*damping);
		speed=i;
		pwm=speed*mean_speed/100;
	}
	lockThruster();
 }
