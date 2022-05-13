#define MOTOR0_PIN 14

const int PHz = 250;
const int Pcyc = 1000000/PHz;
const float pwmbit = 4095 /Pcyc;

void setup() {
  delay(3000);
  analogWriteResolution(12);
  delay(1000);
  analogWriteFrequency(MOTOR0_PIN, PHz);
  //delay(1000);
  //analogWrite(MOTOR0_PIN, 1300*pwmbit);
  //delay(1000);
  //analogWrite(MOTOR0_PIN, 0);
  analogWrite(MOTOR0_PIN, 2000*pwmbit);
  //Serial.println("a");
  delay(5000);
  //Serial.println("b");
  analogWrite(MOTOR0_PIN, 1000*pwmbit);
  delay(5000);
  ///////////////////////////////////////////////////
}

void loop() {
  //Serial.println("c");
  //delay(500);
  analogWrite(MOTOR0_PIN, 1100*pwmbit);
  delay(1000);
  analogWrite(MOTOR0_PIN, 1500*pwmbit);
  delay(1000);
  
}
