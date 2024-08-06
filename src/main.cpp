/****************************************************************

Hardware Connections:

IMPORTANT: The APDS-9960 can only accept 3.3V!
 
 ESP32 Pin  APDS-9960 Board  Function
 
 3.3V         VCC              Power
 GND          GND              Ground
 D21          SDA              I2C Data
 D22          SCL              I2C Clock
 25           INT              Interrupt

****************************************************************/

#include <Wire.h>
#include <SparkFun_APDS9960.h>
#include "LiquidCrystal.h"

// Pins
#define APDS9960_INT    25 // Interrupt pin

void interruptRoutine();
void handleGesture();

// Global Variables
SparkFun_APDS9960 apds = SparkFun_APDS9960();
int isr_flag = 0;
int commandSequence[50] = {0};
int qntGest = 0;
int cursorLCD = 0;
int startFLAG = 0, endFLAG = 0;

// Initializing LCD
LiquidCrystal lcd(19, 23, 18, 17, 16, 15);

void setup() {

    // Set interrupt pin as input
    pinMode(APDS9960_INT, INPUT);
    
    // set up the LCD's number of columns and rows:
    lcd.begin(16, 2);

    // Print a message to the LCD.
    lcd.print("hello, world!");

    // Initialize Serial port
    Serial.begin(9600);
    Serial.println();
    Serial.println(F("--------------------------------"));
    Serial.println(F("SparkFun APDS-9960 - GestureTest"));
    Serial.println(F("--------------------------------"));
    
    // Initialize interrupt service routine
    attachInterrupt(APDS9960_INT, interruptRoutine, FALLING);

    // Initialize APDS-9960 (configure I2C and initial values)
    if ( apds.init() ) {
        Serial.println(F("APDS-9960 initialization complete"));
    } else {
        Serial.println(F("Something went wrong during APDS-9960 init!"));
    }
    
    // Start running the APDS-9960 gesture sensor engine
    if ( apds.enableGestureSensor(true) ) {
        Serial.println(F("Gesture sensor is now running"));
    } else {
        Serial.println(F("Something went wrong during gesture sensor init!"));
    }

	// Turn on the display:
    lcd.display();
}

void loop() {
    if( isr_flag == 1 ) {
        detachInterrupt(APDS9960_INT);
        handleGesture();
        isr_flag = 0;
        attachInterrupt(APDS9960_INT, interruptRoutine, FALLING);
    }
}

void interruptRoutine() {
    isr_flag = 1;
}

void handleGesture() {
    if(apds.isGestureAvailable()){
		int gesture = apds.readGesture();
		Serial.println((int)gesture);
		Serial.println((int)startFLAG);
		Serial.println((int)endFLAG);


        if(gesture == DIR_UP && startFLAG == 1 && endFLAG == 0){
			Serial.println("UP");
			commandSequence[qntGest] = 1;
			qntGest++;
		}
		else if(gesture == DIR_RIGHT && startFLAG == 1 && endFLAG == 0) {
			Serial.println("RIGHT");
			commandSequence[qntGest] = 2;
			qntGest++;
		}
		else if(gesture == DIR_LEFT && startFLAG == 1 && endFLAG == 0){
			Serial.println("LEFT");
			commandSequence[qntGest] = 3;
			qntGest++;			
		}
		else if(gesture == DIR_DOWN && startFLAG == 1 && endFLAG == 0 && qntGest > 1){
			Serial.println("DOWN");
			qntGest--;
			commandSequence[qntGest] = 0;
		}
		else if((gesture == DIR_NEAR) && (startFLAG == 0) && (endFLAG == 0)){
			Serial.println("NEAR");
			commandSequence[qntGest] = 10;
			qntGest++;
			startFLAG = 1;
		}
		else if((gesture == DIR_FAR) && (startFLAG == 1) && (endFLAG == 0)){
			Serial.println("FAR");
			commandSequence[qntGest] = -10;
			qntGest++;
			endFLAG = 1;
		}
		else {
			Serial.println("NONE");
		}
          
    }
}