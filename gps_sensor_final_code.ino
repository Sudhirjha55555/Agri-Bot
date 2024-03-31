#define light_FR  14    //LED Front Right   pin A0 for Arduino Uno
#define light_BR  15    //LED Back Right    pin A1 for Arduino Uno
#define horn_Buzz 16    //Horn Buzzer       pin A2 for Arduino Uno
#define parking_Light 17    //Parking Light       pin A3 for Arduino Uno

#define ENA_m1 5        // Enable/speed motor Front Right 
#define ENB_m1 6        // Enable/speed motor Back Right

#define IN_11  2        // L298N #1 in 1 motor Front Right
#define IN_12  3        // L298N #1 in 2 motor Front Right
#define IN_13  4        // L298N #1 in 3 motor Back Right
#define IN_14  7        // L298N #1 in 4 motor Back Right

int command;            //Int to store app command state.
int speedCar = 100;     // 50 - 255.
int speed_Coeff = 4;
boolean lightFront = false;
boolean lightBack = false;
boolean horn = false;
boolean parkingLight = false;

// Function prototypes
void goAhead();
void goBack();
void goRight();
void goLeft();
void stopRobot();

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

static const int RXPin = 2, TXPin = 3;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
  
  // Call the GPS calibration function
  GPS_calibration();
  
  // Initialize motor control pins
  pinMode(light_FR, OUTPUT);
  pinMode(light_BR, OUTPUT);
  pinMode(horn_Buzz, OUTPUT);
  pinMode(parking_Light, OUTPUT);
  
  pinMode(ENA_m1, OUTPUT);
  pinMode(ENB_m1, OUTPUT);

  pinMode(IN_11, OUTPUT);
  pinMode(IN_12, OUTPUT);
  pinMode(IN_13, OUTPUT);
  pinMode(IN_14, OUTPUT);
}

void GPS_calibration() {
  // Keep reading GPS data until we have valid location data
  while (!gps.location.isValid()) {
    smartDelay(1000);
    if (millis() > 5000 && gps.charsProcessed() < 10) {
      Serial.println(F("No GPS data received: check wiring"));
      return; // Exit function if no valid GPS data after 5 seconds
    }
  }

  // Print latitude and longitude once
  Serial.print("Latitude: ");
  Serial.print(gps.location.lat(), 6);
  Serial.print(", Longitude: ");
  Serial.println(gps.location.lng(), 6);

  // Enter an infinite loop to prevent further execution
  while (true) {
    // Do nothing, just stay in this loop indefinitely
  }
}

void loop() {
  // Empty loop as all functionality is contained within the GPS_calibration function
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void goAhead(){ 
    digitalWrite(IN_11, LOW);
    digitalWrite(IN_12, HIGH);
    analogWrite(ENA_m1, speedCar);

    digitalWrite(IN_13, HIGH);
    digitalWrite(IN_14, LOW);
    analogWrite(ENB_m1, speedCar);
}

void goBack(){ 
    digitalWrite(light_BR, HIGH); 
    digitalWrite(IN_11, HIGH);
    digitalWrite(IN_12, LOW);
    analogWrite(ENA_m1, speedCar);

    digitalWrite(IN_13, LOW);
    digitalWrite(IN_14, HIGH);
    analogWrite(ENB_m1, speedCar);
}

void goRight(){ 
    digitalWrite(IN_11, HIGH);
    digitalWrite(IN_12, LOW);
    analogWrite(ENA_m1, speedCar);

    digitalWrite(IN_13, HIGH);
    digitalWrite(IN_14, LOW);
    analogWrite(ENB_m1, speedCar);
}

void goLeft(){
    digitalWrite(IN_11, LOW);
    digitalWrite(IN_12, HIGH);
    analogWrite(ENA_m1, speedCar);

    digitalWrite(IN_13, LOW);
    digitalWrite(IN_14, HIGH);
    analogWrite(ENB_m1, speedCar);      
}

void stopRobot(){  
    digitalWrite(IN_11, LOW);
    digitalWrite(IN_12, LOW);
    analogWrite(ENA_m1, speedCar);

    digitalWrite(IN_13, LOW);
    digitalWrite(IN_14, LOW);
    analogWrite(ENB_m1, speedCar); 
}
