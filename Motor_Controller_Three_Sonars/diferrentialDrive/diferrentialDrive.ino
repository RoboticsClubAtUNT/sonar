#define WHEEL_R  6.5//wheel radius in cm
#define WHEEL_L  15.24//distance betwwen two wheel in cm
//this function will make the robot turn at an angular velocity of turnVelocity; the linear velocity during the turn is zero
void diferrentialDrive(int turnVelocity, int leftWheel, int rightWheel)//leftWheel and rightWheel are the angular velocity of each wheel
{
  leftWheel = - (turnVelocity * WHEEL_L)/(2* WHEEL_R);
  rightWheel = (turnVelocity * WHEEL_L)/(2 * WHEEL_R);
}
