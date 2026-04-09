#include <Servo.h>

Servo finger;

const int flexPin = A0;

float filtered = 0;
float alpha = 0.15;  //smoothing factor

float servoPos = 90;

void setup() {
  finger.attach(9);
  filtered = analogRead(flexPin);
  Serial.begin(9600);
  finger.write(0);
}

void loop() {

  // read sensor
  int raw = analogRead(flexPin);
  Serial.println(raw);
  filtered = alpha * raw + (1 - alpha) * filtered;

  float target = map(filtered, 100, 850, 0, 180);

  // constraints
  target = constrain(target, 0, 180);
  
  servoPos += (target - servoPos) * 0.2;
  finger.write(servoPos);

  delay(10); // ~100 Hz update
}
