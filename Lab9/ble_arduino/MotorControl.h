#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H
// * Motor PWM Control Functions

// Motor driver pins
#define ABIN1_L 13
#define ABIN2_L 12
#define ABIN1_R 7
#define ABIN2_R 6

const float CALI_FACTOR = 1.15;   // Correct the drifting between the two wheels

void stop() {
  analogWrite(ABIN1_L, 0);
  analogWrite(ABIN2_L, 0); 
  analogWrite(ABIN1_R, 0);
  analogWrite(ABIN2_R, 0);
}

void forward(int speed) {
  analogWrite(ABIN1_L, int(speed*CALI_FACTOR));
  analogWrite(ABIN2_L, 0);
  analogWrite(ABIN1_R, speed);
  analogWrite(ABIN2_R, 0);
}

void backward(int speed) {
  analogWrite(ABIN1_L, 0);
  analogWrite(ABIN2_L, speed);          
  analogWrite(ABIN1_R, 0);
  analogWrite(ABIN2_R, int(speed*CALI_FACTOR));
}

// Clockwise turns
void cturn(int speed) {
    analogWrite(ABIN1_L, int(speed*CALI_FACTOR));
    analogWrite(ABIN2_L, 0);            
    analogWrite(ABIN1_R, 0);
    analogWrite(ABIN2_R, speed);
}

// Counter-clockwise turns
void ccturn(int speed) {
    analogWrite(ABIN1_L,0);
    analogWrite(ABIN2_L,int(speed*CALI_FACTOR));
    analogWrite(ABIN1_R,speed);
    analogWrite(ABIN2_R,0);
}

// Turn right/left while forwarding
void turn(int lspeed, int rspeed) {
    analogWrite(ABIN1_L,int(lspeed*CALI_FACTOR));
    analogWrite(ABIN2_L,0);
    analogWrite(ABIN1_R,rspeed);
    analogWrite(ABIN2_R,0);
}

#endif // MOTORCONTROL_H