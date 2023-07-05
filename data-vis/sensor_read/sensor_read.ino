void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int sensor1 = analogRead(5);
  int sensor2 = analogRead(6);
  int sensor3 = analogRead(12);
  int sensor4 = analogRead(14);
  int sensor5 = analogRead(18);
  // print out the value you read:
  Serial.printf("%d\t\t%d\t\t%d\t\t%d\t\t%d\n", sensor1, sensor2, sensor3, sensor4, sensor5);
  delay(20);  // delay in between reads for stability
}
