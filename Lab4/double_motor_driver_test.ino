int ABIN1_L = 13;
int ABIN2_L = 12;
int ABIN1_R = 7;
int ABIN2_R = 6;

void setup() {
  pinMode(ABIN1_L, OUTPUT);  // sets the pin as output
  pinMode(ABIN2_L, OUTPUT);
  pinMode(ABIN1_R, OUTPUT);
  pinMode(ABIN2_R, OUTPUT);

  delay(5000);
}

void loop() {
  analogWrite(ABIN1_L, 0);
  analogWrite(ABIN2_L, 100);

  analogWrite(ABIN1_R, 0);
  analogWrite(ABIN2_R, 100);
}