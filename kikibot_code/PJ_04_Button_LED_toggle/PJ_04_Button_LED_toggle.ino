// generated by KidsBlock
#include <Arduino.h>

int newKey = 0;

int oldKey = 0;

int mode = 0;


void setup() {
  pinMode(4, OUTPUT);
  pinMode(5, INPUT);
}

void loop() {
  newKey=(!digitalRead(5));
  if (!(newKey == oldKey) && newKey == 1) {
    mode++;
    mode=(mode % 2);
    digitalWrite(4, mode);
  }
  oldKey=newKey;
  delay(0.05 * 1000);
}
