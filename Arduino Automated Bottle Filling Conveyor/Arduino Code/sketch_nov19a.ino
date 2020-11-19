#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
// LCD Display usage and the example to write the characters in this GitHub Link : https://github.com/adafruit/STEMMA_LiquidCrystal/tree/master/examples to understand in detailed Arduino Link http://www.arduino.cc/en/Reference/LiquidCrystal
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

#define echoPin 7 // Echo Pin Ultrasonic // to understand Ultrasonic https://create.arduino.cc/projecthub/abdularbi17/ultrasonic-sensor-hc-sr04-with-arduino-tutorial-327ff6
#define trigPin 8 // Trigger Pin Ultrasonics
#define relay 10 // Onboard LED
#define Glass 9 // Bottle detection
#define solenoid 13 // Solenoid detection
// Conveyor DC Motor use and understanding the circuit diagram https://www.tutorialspoint.com/arduino/arduino_dc_motor.htm#:~:text=Following%20is%20the%20schematic%20diagram%20of%20the,interface%20to%20Arduino%20Uno%20board.&text=Pin%20IN1%20of%20the%20IC,PWM%20pin%202%20of%20Arduino.
byte armsUp[8] = {
  0b00100,
  0b01010,
  0b00100,
  0b10101,
  0b01110,
  0b00100,
  0b00100,
  0b01010
}; // make some custom characters: on LCD Display these are ASCII and Binary value for characterisation link to understand this : https://github.com/adafruit/STEMMA_LiquidCrystal/blob/master/examples/CustomCharacter/CustomCharacter.ino

int maximumRange = 200; // Maximum range needed depends on the motor used it can be 200+ and more
int minimumRange = 0; // Minimum range needed and change as suitable for conveyor for industry usage
long duration, distance; // Duration used to calculate distance to understand this 

void setup() {
 lcd.createChar(4, armsUp); 
  lcd.begin(16, 2);
  lcd.write(4);
   lcd.print(" Production Line Automation ");
    for (int positionCounter = 15; positionCounter < 84; positionCounter++) {
    // scroll one position left:
    lcd.scrollDisplayLeft(); 
    // wait a bit: and change time if bottle is not hold at right position
    delay(400);
  }
  delay(1000);
  lcd.clear();
   delay(2000);
 pinMode(trigPin, OUTPUT);
 pinMode(solenoid, OUTPUT);
 pinMode(Glass, INPUT);
 pinMode(echoPin, INPUT);
 pinMode(relay, OUTPUT); // Use indicator like led or buzzer whatever suitable for you
}

void loop()
{

 digitalWrite(trigPin, LOW); // Change delay while debugging
 delayMicroseconds(2); 

 digitalWrite(trigPin, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trigPin, LOW);
 duration = pulseIn(echoPin, HIGH);
 
 //Calculate the distance (in cm) based on the speed of sound. Ultrasonic sound to distance conversion formula HC-SR04 Ultrasonic to stop the conveyor
 distance = duration/58.2;
 
 int Detect_glass =digitalRead(Glass);
 
 if(Detect_glass == HIGH)  //no glass
 {
    digitalWrite(relay, HIGH);  //conveyor
    lcd.setCursor(0, 0);
    lcd.print("Conveyor Start"); 
    digitalWrite(solenoid, LOW);
 if(relay==HIGH)
    delay(40); //int a = a++; 
 }
 
  if(Detect_glass == LOW)
 {
 lcd.setCursor(0, 0); // Understand solenoid valve usage here: https://bc-robotics.com/tutorials/controlling-a-solenoid-valve-with-arduino/ and https://create.arduino.cc/projecthub/robotgeek-projects-team/control-a-solenoid-with-arduino-710bdc
    lcd.print("Conveyor  Stop"); 
    digitalWrite(relay, LOW);   //conveyor off
    delay(1000);
    digitalWrite(solenoid, HIGH);  //solenoid on water pouring continuously else it will stop right away if solenoid off the valve will be too
    
    //sonar value for level detector change according to your suitability
    if(distance<=9)
    {
digitalWrite(solenoid, LOW);  //water off because solenoid put the valve down/close
 delay(1000);
 digitalWrite(relay, HIGH); //conveyor on
 delay(3000); // change delay as convinient to you also debug every single step while pouring the fluid
    }
 }

 delay(50);
}
