#include "precheck.h"

void Precheck::init() {
  pinMode(ERROR_LED, OUTPUT);
}

void Precheck::fail() {
  while(1){
    digitalWrite(ERROR_LED, HIGH);
    delay(400);
    digitalWrite(ERROR_LED, LOW);
    delay(400);
  }
}