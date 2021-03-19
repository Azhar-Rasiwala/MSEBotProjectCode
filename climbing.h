#ifndef CLIMBING_H_INCLUDED
#define CLIMBING_H_INCLUDED


int frontMotorA = 15;
int frontMotorB = 23;

void setup() {
  setupMotion();

}

void loop() {
delay(1000);
  stopMotion();
  delay(1000);
  setupMotion();

}

double dForwardSpeed;

void setupMotion (void){
  ledcAttachPin(frontMotorA, 5);
  ledcAttachPin(frontMotorB, 6);

  ledcSetup(5, 300, 8);       // only value found to work for now. Can be changed from 300
  ledcSetup(6, 300, 8);

  dForwardSpeed = 250;        //250 is max speed (determined through testing)
  ledcWrite(1,dForwardSpeed);
  ledcWrite(2,0);
}

void stopMotion(){
  ledcWrite(5,0);
  ledcWrite(6,0);
}

#endif // CLIMBING_H_INCLUDED
