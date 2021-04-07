#ifndef CLIMBING_H_INCLUDED
#define CLIMBING_H_INCLUDED

const int frontMotorA = 15;
const int frontMotorB = 23;
double upwardClimbSpeed = 250;        //250 is max speed (determined through testing)

void setupClimbing (void)
{
  ledcAttachPin(frontMotorA, 5);
  ledcAttachPin(frontMotorB, 6);

  ledcSetup(5, 300, 8);           // only value found to work for now. Can be changed from 300
  ledcSetup(6, 300, 8);
}

void startMotion()
{
  ledcWrite(5, dForwardSpeed);
  ledcWrite(6, 0);
}

void stopMotion()
{
  ledcWrite(5, 150);
  ledcWrite(6, 0);
}

#endif // CLIMBING_H_INCLUDED
