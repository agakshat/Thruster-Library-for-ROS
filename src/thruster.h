#ifndef  THRUSTER_H
#define  THRUSTER_H
#ifndef COUNT_CONST 
#define COUNT_CONST 3

#endif //for COUNT_CONST
class Thruster 
{
//    private:
		
	
	public:
// Variables
		
		int dir1_pin;					//stores pin no of direction1
		int dir2_pin;					//stores pin no of direction2
		int pwm_pin;					//stores pin no of pwm pin
		int dir1;		                //stores value of DIR1PIN as 1 or 0
		int dir2;                      //stores value of DIR2PIN as 1 or 0
		int pwm;						//pwm: Stores the pwm value given to the Thruster
		int mean_speed;					//mean_speed : The value to which Thruster moves when speed=100%
		float speed;					//speed		: Speed of the Thruster in percentage of meanspeed
		int damping;                 //to be changed later by trial
		
// Functions
     public:	
		Thruster();						//constructor
		Thruster(int Dir1,int Dir2,int Pwm);	//constructor with attachments of pins
		void attachThruster(int Dir1,int Dir2,int Pwm); //attachments of pins
		void moveThruster(int Pwm);			//+ve for CW and -ve for CCW.
        	void moveThruster(int Dir1,int Dir2,int Pwm); // dir1 and dir2 can be 1 or 0,pwm can only be +ve for CW
		void stopThruster();				//By default stop Thruster will lock Thruster
        	void lockThruster();				//To lock the Thruster
		void freeThruster();				//Free the Thruster
		void setMeanSpeed(int Speed);	//Sets the meanspeed with which Thruster moves when speed=100%	
		void setThrusterSpeed(int Speed);		//+ve for CW and -ve for CCW. Speed in percentage of meanspeed
		void setThrusterSpeed(int Dir1,int Dir2,int Speed); // dir1 and dir2 can be 1 or 0
		void changePWM(int Pwm);		//Just to change the PWM in whatever direction the Thruster was moving
		void changeSpeed(int Speed);	//Just to change the speed (In percentage) not the direction
		int getDirection();             //+1 for cw and -1 for ccw and 0 for free or locked
		int isFree();                  //+1 for free and 0 for not free
		int isLocked();                //+1 for locked and 0 for not locked
		int getSpeed();                // returns speed in % of mean speed
		int getPWM();                  // returns +ve for CW and -ve for CCW.
	    	void startSmoothly(int Speed);   //+ve for CW and -ve for CCW.
		void stopSmoothly();
};		
#endif
