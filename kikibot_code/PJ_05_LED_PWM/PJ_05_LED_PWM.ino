// generated by KidsBlock
#include <Arduino.h>

int item = 0;


void setup() {
  ledcSetup(1, 490, 8);
  ledcAttachPin(4, 1);
}

void loop() {
  for (int index = 0; index < 255; index++) {
    item++;
    ledcWrite(1, item);
    delay(0.01 * 1000);
  }
  for (int index = 0; index < 255; index++) {
    ledcWrite(1, item);
    item--;
    delay(0.01 * 1000);
  }
}
