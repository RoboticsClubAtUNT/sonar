//Author: Cat Tran
//Device: Arduino Uno
//Date: 04/10/2015
//Purpose: Control DC motors via a "Dual L298 H-bridge Motor Contoller" using TWO "HC-SR04 Ultrasonic sensor"
#include <NewPing.h>//this is a library for ultrasonic sensor
#define SAFEZONE  50//SAFEZONE is the distance in which robot will try yo turn when obstacle appear; is set to 50cm (calibratable)
#define STOPZONE  25//STOPZONE is the distance in which robot will stop imidiately if an obstacle is detected; is set to be 25cm (calibratable)
#define TURNSPEED  10//the angular speed constant at which the robot turns(calibratable)
class motor{
       public:
             int directionControl_1;
             int directionControl_2;
             int speedPWM;
            };
//assigning pins to control direction
motor motorA = {2,3,4};//motor A direction controls are in pin 2 and 3, and its speed PWM is pin 4
motor motorB = {8,9,10};//motor B direction controls are in pin 8 and 9, and its speed PWM is pin 10
int triggerPin[] = {22,24,26};//sonars trigger pins are 22...
int echoPin[] = {23,25,27};//sonar echo pins are 23...
static int accelerationConstant = 8;//multiples of 10
static int decelerationConstant = 0;//multiples of 10
NewPing sonarLeft(triggerPin[0], echoPin[0], 500);//creating a new object called "ultrasonicSensor()" which belong to class NewPing; 500 cm is maximum distance
NewPing sonarRight(triggerPin[1], echoPin[1], 500);
NewPing sonarMid(triggerPin[2], echoPin[2], 500);
//==============================================FUNCTIONS PROTOTYPE=========================================//
void accelerationMotorA(int A_DirectionControl_1, int A_DirectionControl_2, int A_speedPWM);
//=====================================================SETUP================================================//
void setup()
{
	Serial.begin(9600);
	//set all direction control pins and speed PWM pin as outputs
	pinMode(motorA.directionControl_1, OUTPUT);
	pinMode(motorA.directionControl_2, OUTPUT);
	pinMode(motorA.speedPWM, OUTPUT);
	pinMode(motorB.directionControl_1, OUTPUT);
	pinMode(motorB.directionControl_2, OUTPUT);
	pinMode(motorB.speedPWM, OUTPUT);
        int i;
        //initialize all triggerPin as output, echoPin as input
        for(i = 0; i < 3; i++)
        {           
	  pinMode(triggerPin[i], OUTPUT);
	  pinMode(echoPin[i], INPUT);
        }
}
//====================================================LOOP================================================//
int leftView, rightView, midView;
void loop()
{
	/*sonarLeft.ping_cm();
        sonarRight.ping_cm();
        sonarMid.ping_cm();
        accelerationMotorA(motorA.directionControl_1, motorA.directionControl_2, motorA.speedPWM);*/
        leftView = sonarLeft.ping_cm();
        delay(30);
        rightView = sonarRight.ping_cm();
        delay(30);
        midView = sonarMid.ping_cm();
        if(leftView < STOPZONE || rightView < STOPZONE || midView < STOPZONE)//when obstacle appear in STOPZONE, cut off power to the motors
        {
            motorA.speedPWM = 0;
            motorB.speedPWM = 0;
            Serial.print(leftView);
            Serial.print(" cm  ");            
            Serial.print(rightView);
            Serial.print(" cm  ");
            Serial.print(midView);
            Serial.print(" cm  \n");
            delay(50);
        }
        else//when obstacle is not within STOPZONE
        {
            if (leftView < SAFEZONE)//if obstacle is in left view then turn right
            {
                turnRight(motorA.directionControl_1, motorA.directionControl_2, motorB.directionControl_1, motorB.directionControl_2, motorA.speedPWM, motorB.speedPWM, leftView, rightView, midView);                
            }
            else//if obstacle not in left view then check for right view
            {
                if(rightView < SAFEZONE)//if obstacle is in right view the turn left
                {
                    turnLeft(motorA.directionControl_1, motorA.directionControl_2, motorB.directionControl_1, motorB.directionControl_2, motorA.speedPWM, motorB.speedPWM, leftView, rightView, midView);
                }
                else//if obstacle is not in right view either, chek for mid view
                {
                    if(midView < SAFEZONE)//if obstacle is in mid view, generate random turn
                    {
                        randomTurn(motorA.directionControl_1, motorA.directionControl_2, motorB.directionControl_1, motorB.directionControl_2, motorA.speedPWM, motorB.speedPWM, leftView, rightView, midView);
                    }
                    else                        
                        accelerationMotor(motorA.directionControl_1, motorA.directionControl_2, motorA.speedPWM, motorB.directionControl_1, motorB.directionControl_2, motorB.speedPWM);
                }
            }
        }
            
            
}
//============================================FUNCTIONS DEFINITIONS=======================================//
//----------------------------------------Convert turn velocity to left and right wheel velocity-----------------------------------------//
#define WHEEL_R  6.5//wheel radius in cm
#define WHEEL_L  15.24//distance betwwen two wheel in cm
//this function will make the robot turn at an angular velocity of turnVelocity; the linear velocity during the turn is zero
void differentialDrive(int turnVelocity, int leftWheel, int rightWheel)//leftWheel and rightWheel are the angular velocity of each wheel
{
  leftWheel = - (turnVelocity * WHEEL_L)/(2* WHEEL_R);
  rightWheel = (turnVelocity * WHEEL_L)/(2 * WHEEL_R);
}
//turn right funtion
void turnRight(int A_directionControl_1, int A_directionControl_2, int B_directionControl_1, int B_directionControl_2, int A_speedPWM, int B_speedPWM, int left, int right, int mid)
{
        analogWrite(A_speedPWM, 10 * TURNSPEED);  
        digitalWrite(A_directionControl_1, HIGH);
        digitalWrite(A_directionControl_2, LOW);
        analogWrite(B_speedPWM, 10 * TURNSPEED);  
        digitalWrite(B_directionControl_1, LOW);
        digitalWrite(B_directionControl_2, HIGH);
        Serial.print(left);
        Serial.print(" ");
        Serial.print(right);
        Serial.print(" ");
        Serial.print(mid);
        Serial.println(" cm\n");       
        delay(50);
}
//turn left funtion
void turnLeft(int A_directionControl_1, int A_directionControl_2, int B_directionControl_1, int B_directionControl_2, int A_speedPWM, int B_speedPWM, int left, int right, int mid)
{
        analogWrite(A_speedPWM, 10 * TURNSPEED);  
        digitalWrite(A_directionControl_1, LOW);
        digitalWrite(A_directionControl_2, HIGH);
        analogWrite(B_speedPWM, 10 * TURNSPEED);  
        digitalWrite(B_directionControl_1, HIGH);
        digitalWrite(B_directionControl_2, LOW);
        Serial.print(left);
        Serial.print(" ");
        Serial.print(right);
        Serial.print(" ");
        Serial.print(mid);
        Serial.println(" cm\n");
        delay(50);
}
//generate random turn funtion
void randomTurn(int A_directionControl_1, int A_directionControl_2, int B_directionControl_1, int B_directionControl_2, int A_speedPWM, int B_speedPWM, int left, int right, int mid)
{
        
}
//------------------------------------------acceleration--deceleration---------------------------------------------------------------------------//
void accelerationMotor(int A_DirectionControl_1, int A_DirectionControl_2, int A_speedPWM, int B_DirectionControl_1, int B_DirectionControl_2, int B_speedPWM)
{    
    if (sonarLeft.ping_cm() >= SAFEZONE)//if obstacle is not within SAFEZONE
    {      
        if(accelerationConstant <= 24)//increment acelerationConstant if it is not maximum, that is 25
        {
		accelerationConstant++;
        }
        analogWrite(4, 255);//10 * accelerationConstant);  
        digitalWrite(A_DirectionControl_1, HIGH);
        digitalWrite(A_DirectionControl_2, LOW);
        analogWrite(B_speedPWM, 10 * accelerationConstant);  
        digitalWrite(B_DirectionControl_1, HIGH);
        digitalWrite(B_DirectionControl_2, LOW);
        Serial.print(accelerationConstant);
        Serial.print(" ");
        Serial.print(sonarLeft.ping_cm());          
        Serial.println("cm acceleration");
        delay(100);           
        decelerationConstant = accelerationConstant;//deceleration will pick up where acceleration was once obstacle enters SAFEZONE
    }
    else if(sonarLeft.ping_cm() < SAFEZONE)//if obstacle is within SAFEZONE    
    {
	if(decelerationConstant > 5)//decrement decelerationConstant if it is not minimum, that is 5
	{
		decelerationConstant--;
	}
	else if(decelerationConstant <= 5)//when decelerationConstant is smaller than 5, just set it to 0
	{
		decelerationConstant = 0;
	}
	analogWrite(4, 10 * decelerationConstant);
	digitalWrite(A_DirectionControl_1, HIGH);
	digitalWrite(A_DirectionControl_2, LOW);
	Serial.print(decelerationConstant);
	Serial.print(" ");
	Serial.print(sonarLeft.ping_cm());
	Serial.println("cm");
	delay(50);               
	accelerationConstant = decelerationConstant;//acceleration will pick up where deceleration was once obstacle is out of SAFEZONE
    }
}
