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
int startFLAG = 0, finishFLAG = 0;

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
}

void loop() {
    if( isr_flag == 1 ) {
        detachInterrupt(APDS9960_INT);
        handleGesture();
        isr_flag = 0;
        attachInterrupt(APDS9960_INT, interruptRoutine, FALLING);
    }
    // Turn on the display:
    lcd.display();
}

void interruptRoutine() {
    isr_flag = 1;
}

void handleGesture() {
    if(apds.isGestureAvailable()){
        switch ( apds.readGesture() ) {
            case DIR_UP:
                Serial.println("UP");
                commandSequence[qntGest] = 1;
                qntGest++;
                break;
            case DIR_LEFT:
                Serial.println("LEFT");
                commandSequence[qntGest] = 2;
                qntGest++;
                break;
            case DIR_RIGHT:
                Serial.println("RIGHT");
                commandSequence[qntGest] = 3;
                qntGest++;
                break;
            case DIR_DOWN:
                Serial.println("DOWN");
                qntGest--;
                commandSequence[qntGest] = 0;
                break;
            case DIR_NEAR:
                Serial.println("NEAR");
                commandSequence[qntGest] = -10;
                qntGest++;
                break;
            case DIR_FAR:
                Serial.println("FAR");
                commandSequence[qntGest] = 10;
                qntGest++;
                break;
            default:
                Serial.println("NONE");
          }
    }
}