int ABIN1_L = 13;
int ABIN2_L = 12;
int ABIN1_R = 7;
int ABIN2_R = 6;

int time_start;
int move_duration_ms = 1000; //in millisecond

const int PWM_START_LOWEST = 40;
const int PWM_MOVE_LOWEST = 30;
const int PWM_TURN_LOWEST = 180;
const float CALI_FACTOR = 1.35;

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

void cturn(int speed) {
    analogWrite(ABIN1_L, int(speed*CALI_FACTOR));
    analogWrite(ABIN2_L, 0);            
    analogWrite(ABIN1_R, 0);
    analogWrite(ABIN2_R, speed);
}

void ccturn(int speed) {
    analogWrite(ABIN1_L,0);
    analogWrite(ABIN2_L,int(speed*CALI_FACTOR));            
    analogWrite(ABIN1_R,speed);
    analogWrite(ABIN2_R,0);
}

void setup() {
  pinMode(ABIN1_L, OUTPUT);  // sets the pin as output
  pinMode(ABIN2_L, OUTPUT);
  pinMode(ABIN1_R, OUTPUT);
  pinMode(ABIN2_R, OUTPUT);

  delay(5000);
}


void loop() {
  if ((millis()-time_start) < move_duration_ms) {
    // Lowest PWM value to get the car moving
    analogWrite(ABIN1_L, PWM_START_LOWEST);
    analogWrite(ABIN2_L, 0);
    analogWrite(ABIN1_R, PWM_START_LOWEST);
    analogWrite(ABIN2_R, 0);

    // Lowest PWM value to make the car turn 
    analogWrite(ABIN1_L, PWM_TURN_LOWEST);
    analogWrite(ABIN2_L, 0);
    analogWrite(ABIN1_R, 0);
    analogWrite(ABIN2_R, PWM_TURN_LOWEST);

    // Calibration factor to make the car move in straight line
    // Calibration factor: 1.35
    analogWrite(ABIN1_L, 135);
    analogWrite(ABIN2_L, 0);
    analogWrite(ABIN1_R, 100);
    analogWrite(ABIN2_R, 0);
  }
  else {
    analogWrite(ABIN1_L, 0);
    analogWrite(ABIN2_L, 0);
    analogWrite(ABIN1_R, 0);
    analogWrite(ABIN2_R, 0);
  }
}
