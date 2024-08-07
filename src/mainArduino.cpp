#include <Arduino.h>
#include <Wire.h>

#define LEFT 1
#define RIGHT 2
#define UP 3
#define START 5
#define END 6

void setup() {
	pinMode(13,OUTPUT);
}

void loop() {
	digitalWrite(13,HIGH);
	delay(1000);
	digitalWrite(13,LOW);
	delay(1000);
}
