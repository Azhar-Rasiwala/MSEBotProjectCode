//Header file for climbing routines and climb motor operation

#ifndef CLIMBING_H_INCLUDED
#define CLIMBING_H_INCLUDED

//Motor Driver output pins
const int frontMotorA = 15;
const int frontMotorB = 23;
double upwardClimbSpeed = 250;        //250 is max speed (determined through testing)

//Setup the PWM signal to the motor
void setupClimbing (void)
{
  ledcAttachPin(frontMotorA, 5); //Channel 5,6 are unused 
  ledcAttachPin(frontMotorB, 6);

  ledcSetup(5, 300, 8);           //300 Hz, 8 bit resolution 
  ledcSetup(6, 300, 8);
}

//Run motor forwards
void startMotion()
{
  ledcWrite(5, dForwardSpeed);
  ledcWrite(6, 0);
}

//Stop motor 
//WARNING: Stopping the motor while the bot is suspended in the air will cause it to drop back down
void stopMotion()
{
  ledcWrite(5, 0);
  ledcWrite(6, 0);
}

#endif // CLIMBING_H_INCLUDED
