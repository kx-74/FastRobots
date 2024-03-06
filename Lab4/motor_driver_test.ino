int ABIN1 = 13;
int ABIN2 = 12;

void setup() {
  pinMode(ABIN1, OUTPUT);  // sets the pin as output
  pinMode(ABIN2, OUTPUT);
  delay(5000);
}

void loop() {
  analogWrite(ABIN1, 0);
  analogWrite(ABIN2, 255);

  delay(2000);

  analogWrite(ABIN1, 255);
  analogWrite(ABIN2, 0);

  delay(2000);

  analogWrite(ABIN1, 0);
  analogWrite(ABIN2, 50);

  delay(2000);

  analogWrite(ABIN1, 50);
  analogWrite(ABIN2, 0);

  delay(2000);
}